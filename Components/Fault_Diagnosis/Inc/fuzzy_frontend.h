/*
 * fuzzy_frontend.h
 * ---------------------------------------------------------------------------
 * DIV2IFSCN 模糊前端（区间二型模糊隶属度 → H0[50] 规则输出）
 * 保持 float32：本模块是 expf / sqrt / 查表 等非 matmul 预处理，
 * 不参与 int8 量化；在 STM32H743 硬 FPU(fpv5-d16) 上直接跑 float32。
 * 使用 model_weights.h 的浮点权重 + fuzzy_lut.h 预计算 gamma 查表
 * （免去 MCU 端 powf 初始化）。
 * ---------------------------------------------------------------------------
 */
#ifndef FUZZY_FRONTEND_H
#define FUZZY_FRONTEND_H

#include <stdint.h>

/*
 * 计算 50 条模糊规则的 H0 输出。
 * input : 长度 AI_INPUT_DIM(10) 的归一化特征
 * H0    : 输出缓冲，长度 AI_NUM_RULES(50)
 */
void FuzzyFrontend_ComputeH0(const float *input, float *H0);

#endif /* FUZZY_FRONTEND_H */
