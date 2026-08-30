/*
 * fuzzy_frontend.c
 * ---------------------------------------------------------------------------
 * DIV2IFSCN 模糊前端实现。逻辑与 freertos.c 中的 DIV2IFSCN_Inference
 * 第一部分（H0 计算）严格对齐，区别在于：
 *   - gamma 映射直接查 fuzzy_lut.h 预计算的 gamma_map_lut[1024]，
 *     无需运行时 powf 初始化；
 *   - 抽成独立模块，供 ai_infer.c 调用，便于单测与复用。
 * 全部 float32，依赖硬 FPU。
 * ---------------------------------------------------------------------------
 */
#include "fuzzy_frontend.h"
#include "model_weights_ext.h" /* extern: c_centers / sigma_upper / sigma_lower /
                                  p_consequent_factors / ivifs_gamma / 维度 */
#include "fuzzy_lut.h"       /* gamma_map_lut[1024], GAMMA_LUT_SIZE */
#include "arm_math.h"        /* arm_sqrt_f32 */

/* 线性插值版 gamma 映射：y = gamma_map_lut[idx] + frac*(...)，与 freertos.c
 * 的 fast_gamma_Test 行为一致。输入 x 应已在 [0,1]。 */
static inline float fast_gamma(float x)
{
    if (x < 0.0f) {
        x = 0.0f;
    } else if (x > 1.0f) {
        x = 1.0f;
    }
    float pos = x * (float)(GAMMA_LUT_SIZE - 1);
    int idx = (int)pos;
    if (idx >= GAMMA_LUT_SIZE - 1) {
        return gamma_map_lut[GAMMA_LUT_SIZE - 1];
    }
    float fraction = pos - (float)idx;
    return gamma_map_lut[idx] + fraction * (gamma_map_lut[idx + 1] - gamma_map_lut[idx]);
}

void FuzzyFrontend_ComputeH0(const float *input, float *H0)
{
    const float gamma = ivifs_gamma;
    /* 预计算 1/gamma，避免循环内重复除法 */
    const float inv_gamma = 1.0f / gamma;

    for (int i = 0; i < (int)num_fuzzy_rules; i++) {
        float p_mu_H = 1.0f;
        float p_mu_L = 1.0f;
        float p_1_minus_nu_H = 1.0f;
        float p_1_minus_nu_L = 1.0f;
        float consequent = 0.0f;

        for (int j = 0; j < (int)input_dim; j++) {
            int idx = i * (int)input_dim + j;
            float diff = input[j] - c_centers[idx];
            float diff_sq = diff * diff;

            /* 1/(sigma^2) 直接算，避免额外预计算缓冲（前端只跑一次/样本） */
            float inv_su = 1.0f / (sigma_upper[idx] * sigma_upper[idx]);
            float inv_sl = 1.0f / (sigma_lower[idx] * sigma_lower[idx]);

            float test_mu_H = expf(-diff_sq * inv_su);
            float test_mu_L = expf(-diff_sq * inv_sl);

            /* 区间二型 nu 映射：nu = (1 - (1 - mu^gamma))^(1/gamma) */
            float test_nu_H = fast_gamma(test_mu_L);
            float test_nu_L = fast_gamma(test_mu_H);

            p_mu_H *= test_mu_H;
            p_mu_L *= test_mu_L;
            p_1_minus_nu_H *= (1.0f - test_nu_H);
            p_1_minus_nu_L *= (1.0f - test_nu_L);

            consequent += input[j] * p_consequent_factors[idx];
        }

        float nu_rule_H = 1.0f - p_1_minus_nu_H;
        float nu_rule_L = 1.0f - p_1_minus_nu_L;

        float sq_mu_L, sq_mu_H, sq_nu_L, sq_nu_H;
        arm_sqrt_f32(p_mu_L, &sq_mu_L);
        arm_sqrt_f32(p_mu_H, &sq_mu_H);
        arm_sqrt_f32(1.0f - nu_rule_L, &sq_nu_L);
        arm_sqrt_f32(1.0f - nu_rule_H, &sq_nu_H);

        float score = (sq_mu_L + sq_mu_H + sq_nu_L + sq_nu_H) * 0.5f;
        H0[i] = score * consequent;
    }
}
