/*
 * ai_infer.c
 * ---------------------------------------------------------------------------
 * DIV2IFSCN 部署版端到端推理（STM32H743 / Cortex-M7）。
 *
 * 实际跑哪套权重由 ai_config.h 的 AI_PRECISION 在编译期决定：
 *
 *  [AI_PRECISION_FLOAT32]         → 委托 freertos.c 的 DIV2IFSCN_Inference（纯 float32 oracle）
 *  [AI_PRECISION_INT8_HYBRID]     → FC1 int8(CMSIS-NN) + FC2 float32(CMSIS-DSP)   ★推荐部署
 *  [AI_PRECISION_FC1F32_FC2I8]    → FC1 float32 + FC2 int8(单 scale)
 *  [AI_PRECISION_FC1I8_FC2I8]     → FC1 int8 + FC2 int8(单 scale)
 *  [AI_PRECISION_FULL_INT8]       → 全 int8：FC1 int8 + FC2 int8 + Z_final 分段量化
 *
 * 推理链（所有模式共用）：
 *   input(10) ─[模糊前端 float32]─▶ H0(50)
 *   H0(50) ─[FC1]─▶ hid_pre(100) ─[sigmoid]─▶ hidden(100)
 *   Z_final(150) = concat(H0, hidden)
 *   Z_final(150) ─[FC2]─▶ logits(4) ─[argmax]─▶ class
 *
 * 缓冲全部放在默认 .bss/.data（散列文件落入 DTCM），与权重同域。
 *
 * 关于 Z_final 范围：当前 DIT2 模型 p_consequent=±0.3，H0 absmax≈±1.83，
 *   hidden∈[0,1]，二者同量级，已无旧模型（p=±3, H0=±10.4）的 ~10× 范围失配。
 *   - 单 scale 即可把 Z_final 整体量化（无需分段）。
 *   - 分段量化（FULL_INT8）进一步为 H0 段 / hidden 段 各自标定 scale，降低量化误差。
 *   各模式 PC 预校验精度（1605 测试集，已与 PyTorch 94.70% 对齐，见 pc_int8_validate.py）：
 *     FLOAT32=94.70% / INT8_HYBRID=90.97% / FC1f32+FC2i8=91.59%
 *     / FC1i8+FC2i8=90.03% / FULL_INT8=88.72%。
 *   精度最优推荐 INT8_HYBRID（PC 90.97% / 硬件 90.90%）；ai_config.h 当前默认即为 HYBRID。
 *   若切到 FULL_INT8（PC 88.79% / 硬件 88.66%，最省 Flash/计算），务必先重跑
 *   ML/quantize_ptq.py 用当前 model_weights.h 重建 int8 产物（否则三方错配会崩到 ~42%）。
 * ---------------------------------------------------------------------------
 */
#include "ai_infer.h"
#include "ai_config.h"
#include "model_q_params.h"
#include "fuzzy_frontend.h"
#include "arm_math.h"          /* arm_mat_mult_f32 / arm_max_f32 */
#include <math.h>              /* expf / roundf */

#pragma message("AI_DEPLOY_PRECISION = " AI_PRECISION_NAME)

#if (AI_PRECISION == AI_PRECISION_FLOAT32)
/* ---------- 纯 float32：复用已验证 oracle（freertos.c），不参与 int8 编译 ---------- */
int AI_Inference(const float *input, float *outputs_out)
{
    extern int DIV2IFSCN_Inference(const float *input, float *outputs);
    return DIV2IFSCN_Inference(input, outputs_out);
}

#else /* ===================== 以下为所有 int8 相关模式 (1/2/3/4) ===================== */

#include "model_weights_ext.h"   /* beta / W_hid_in / b_hid (extern, 由 freertos.c 定义) */

#if (FC1_IS_INT8 || FC2_IS_INT8)
#include "model_weights_q.h"     /* W_hid_in_q / b_hid_q / beta_q / beta_h0_q / beta_hid_q / b_out_q */
#include "arm_nnfunctions.h"     /* arm_fully_connected_s8 */

/* 对称 int8 量化：value/scale，四舍五入到 int8 并裁剪。zero_point=0。 */
static inline int8_t quantize_sym_f32(float v, float scale)
{
    int32_t q = (int32_t)roundf(v / scale);
    if (q > 127)  q = 127;
    if (q < -128) q = -128;
    return (int8_t)q;
}

/* CMSIS-NN per-tensor 全连接（int8）：output[o] = requant( ∑ input[i]*weight[o*in+i] + bias[o] ) */
static inline void fc_s8(const int8_t *input, const int8_t *weight, const int32_t *bias,
                         int in_dim, int out_dim, int8_t *output,
                         int32_t mult, int32_t shift)
{
    cmsis_nn_context ctx = {NULL, 0};
    cmsis_nn_fc_params fc_params;
    cmsis_nn_per_tensor_quant_params quant_params;
    cmsis_nn_dims input_dims  = {1, 1, in_dim,  1};   /* w = in */
    cmsis_nn_dims filter_dims = {in_dim, 1, 1, 1};    /* n = accum_depth = in */
    cmsis_nn_dims bias_dims   = {out_dim, 1, 1, 1};
    cmsis_nn_dims output_dims = {1, 1, 1, out_dim};   /* c = out */
    fc_params.input_offset  = AI_ZP;   /* 0 */
    fc_params.filter_offset = 0;
    fc_params.output_offset = AI_ZP;   /* 0 */
    fc_params.activation.min = AI_ACT_MIN;  /* -128 */
    fc_params.activation.max = AI_ACT_MAX;  /*  127 */
    quant_params.multiplier = mult;
    quant_params.shift      = shift;
    arm_fully_connected_s8(&ctx, &fc_params, &quant_params,
                           &input_dims, input,
                           &filter_dims, weight,
                           &bias_dims, bias,
                           &output_dims, output);
}
#endif /* FC1_IS_INT8 || FC2_IS_INT8 */

int AI_Inference(const float *input, float *outputs_out)
{
    /* ---------- 共享缓冲 ---------- */
    static float H0[AI_NUM_RULES];
    static float hidden[AI_HIDDEN_DIM];
    static float Z_final[AI_Z_DIM];
    static float outputs[AI_OUTPUT_DIM];

    /* ---------- 1. 模糊前端（float32）→ H0 ---------- */
    FuzzyFrontend_ComputeH0(input, H0);

    /* ---------- 2. FC1：int8 或 float32 ---------- */
#if (FC1_IS_INT8)
    {
        static int8_t H0q[AI_NUM_RULES];
        static int8_t hid_pre_q[AI_HIDDEN_DIM];
        for (int i = 0; i < AI_NUM_RULES; i++) {
            H0q[i] = quantize_sym_f32(H0[i], SCALE_IN1_F);
        }
        fc_s8(H0q, W_hid_in_q, b_hid_q, AI_NUM_RULES, AI_HIDDEN_DIM, hid_pre_q,
              FC1_MULT, FC1_SHIFT);
        for (int o = 0; o < AI_HIDDEN_DIM; o++) {
            float hid_pre = (float)hid_pre_q[o] * SCALE_OUT1_F;
            hidden[o] = 1.0f / (1.0f + expf(-hid_pre));
        }
    }
#else /* FC1 float32：H0(1,50) @ W_hid_in(50,100)[out][in] + b_hid → hidden */
    {
        for (int o = 0; o < AI_HIDDEN_DIM; o++) {
            float acc = b_hid[o];
            for (int i = 0; i < AI_NUM_RULES; i++) {
                acc += H0[i] * W_hid_in[o * AI_NUM_RULES + i];
            }
            hidden[o] = 1.0f / (1.0f + expf(-acc));
        }
    }
#endif

    /* ---------- 3. 特征拼接 Z_final = [H0(50) | hidden(100)] ---------- */
    for (int i = 0; i < AI_NUM_RULES; i++) {
        Z_final[i] = H0[i];
    }
    for (int i = 0; i < AI_HIDDEN_DIM; i++) {
        Z_final[AI_NUM_RULES + i] = hidden[i];
    }

    /* ---------- 4. FC2：int8 或 float32 ---------- */
#if (FC2_IS_INT8)
  #if (FULL_INT8)   /* 分段：H0 段 + hidden 段 各自 FC2 后相加（共用 scale_out2） */
    {
        static int8_t h0z_q[AI_NUM_RULES];
        static int8_t hidz_q[AI_HIDDEN_DIM];
        static int8_t out2_h0_q[AI_OUTPUT_DIM];
        static int8_t out2_hid_q[AI_OUTPUT_DIM];
        for (int i = 0; i < AI_NUM_RULES; i++) {
            h0z_q[i] = quantize_sym_f32(H0[i], SCALE_Z_H0_F);
        }
        for (int i = 0; i < AI_HIDDEN_DIM; i++) {
            hidz_q[i] = quantize_sym_f32(hidden[i], SCALE_Z_HID_F);
        }
        fc_s8(h0z_q,  beta_h0_q,  b_out_q, AI_NUM_RULES,  AI_OUTPUT_DIM, out2_h0_q,  FC2H0_MULT,  FC2H0_SHIFT);
        fc_s8(hidz_q, beta_hid_q, b_out_q, AI_HIDDEN_DIM, AI_OUTPUT_DIM, out2_hid_q, FC2HID_MULT, FC2HID_SHIFT);
        /* 两段用同一 scale_out2，直接相加再 dequant */
        for (int k = 0; k < AI_OUTPUT_DIM; k++) {
            outputs[k] = (float)(out2_h0_q[k] + out2_hid_q[k]) * SCALE_OUT2_F;
        }
    }
  #else            /* 单 scale：Z_final 整体量化 */
    {
        static int8_t z_q[AI_Z_DIM];
        static int8_t out2_q[AI_OUTPUT_DIM];
        for (int j = 0; j < AI_Z_DIM; j++) {
            z_q[j] = quantize_sym_f32(Z_final[j], SCALE_Z_F);
        }
        fc_s8(z_q, beta_q, b_out_q, AI_Z_DIM, AI_OUTPUT_DIM, out2_q, FC2_MULT, FC2_SHIFT);
        for (int k = 0; k < AI_OUTPUT_DIM; k++) {
            outputs[k] = (float)out2_q[k] * SCALE_OUT2_F;
        }
    }
  #endif
#else /* FC2 float32：Z_final @ beta → logits */
    {
        arm_matrix_instance_f32 mat_Z, mat_beta, mat_out;
        arm_mat_init_f32(&mat_Z,    1, AI_Z_DIM,      Z_final);
        arm_mat_init_f32(&mat_beta, AI_Z_DIM, AI_OUTPUT_DIM, (float32_t *)beta);
        arm_mat_init_f32(&mat_out,  1, AI_OUTPUT_DIM, outputs);
        arm_mat_mult_f32(&mat_Z, &mat_beta, &mat_out);
    }
#endif

    /* ---------- 5. 取最大类 ---------- */
    float max_val;
    uint32_t best_idx;
    arm_max_f32(outputs, AI_OUTPUT_DIM, &max_val, &best_idx);

    if (outputs_out != NULL) {
        for (int i = 0; i < AI_OUTPUT_DIM; i++) {
            outputs_out[i] = outputs[i];
        }
    }
    return (int)best_idx;
}

#endif /* AI_PRECISION_FLOAT32 ? */
