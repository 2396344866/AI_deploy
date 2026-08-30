/*
 * ai_infer.h
 * ---------------------------------------------------------------------------
 * DIV2IFSCN 端到端推理（部署版），精度由 ai_config.h 的 AI_PRECISION 在编译期决定。
 *
 * 支持的五种模式（切换见 ai_config.h 的 AI_PRECISION）：
 *   0 AI_PRECISION_FLOAT32      → 纯 float32（委托 freertos.c 的 DIV2IFSCN_Inference oracle）
 *   1 AI_PRECISION_INT8_HYBRID  → FC1 int8(CMSIS-NN) + FC2 float32(CMSIS-DSP)   ★推荐部署
 *   2 AI_PRECISION_FC1F32_FC2I8 → FC1 float32 + FC2 int8(单 scale)
 *   3 AI_PRECISION_FC1I8_FC2I8  → FC1 int8 + FC2 int8(单 scale)
 *   4 AI_PRECISION_FULL_INT8    → 全 int8：FC1 int8 + FC2 int8 + Z_final 分段量化
 *
 * 推理链：input(10) →[模糊前端]→ H0(50) →[FC1]→ hidden(100) →[concat]→ Z_final(150) →[FC2]→ logits(4) → argmax
 *
 * input   : 长度 AI_INPUT_DIM(10) 的归一化特征（float32）
 * outputs : 输出缓冲，长度 AI_OUTPUT_DIM(4)，返回各类 logits（float32）
 * 返回    : 预测类别索引 0..3
 * ---------------------------------------------------------------------------
 */
#ifndef AI_INFER_H
#define AI_INFER_H

#include <stdint.h>
#include "ai_config.h"   /* AI_PRECISION / AI_PRECISION_NAME：决定实际跑哪套权重 */

/*
 * 单次推理（精度由 ai_config.h 的 AI_PRECISION 在编译期决定）。
 */
int AI_Inference(const float *input, float *outputs);

/*
 * 诊断结果结构体（上电自检 FaultDiag_ML_Test 写、NetworkTask 读）。
 * 提到本头：selftest.c 与 freertos.c 共享该类型（CubeMX 不碰本头）。
 */
typedef struct {
    uint32_t num_test_samples;
    float    overall_accuracy;
    float    macro_precision;
    float    macro_recall;
    float    macro_f1;
    float    total_time_ms;
    uint8_t  data_is_ready;   /* 标志位：告诉网络任务数据是否已准备好 */
} TestResults_t;

#endif /* AI_INFER_H */
