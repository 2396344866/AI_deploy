/*
 * ai_config.h
 * ---------------------------------------------------------------------------
 * AI 部署精度模式（编译期宏）—— 总开关，决定「运行时真正执行哪套权重路径」。
 *
 * 用法：
 *   1. 烧录前修改本文件里的 AI_PRECISION；
 *   2. 重新编译（CubeIDE / MDK / 命令行均可）；
 *   3. 重新烧录。固件「实际运行哪套精度」即随之改变，业务逻辑无需改动。
 *
 * 五种模式（端侧可切换，用于上板实测对比）：
 *   0 AI_PRECISION_FLOAT32      → 纯 float32（委托 freertos.c 的 DIV2IFSCN_Inference oracle）。
 *   1 AI_PRECISION_INT8_HYBRID  → FC1 int8(CMSIS-NN) + FC2 float32(CMSIS-DSP)。
 *   2 AI_PRECISION_FC1F32_FC2I8 → FC1 float32 + FC2 int8(单 scale)。受 Z_final 范围失配拖累。
 *   3 AI_PRECISION_FC1I8_FC2I8  → FC1 int8 + FC2 int8(单 scale)。同上。
 *   4 AI_PRECISION_FULL_INT8    → 全 int8：FC1 int8 + FC2 int8 + Z_final 分段量化（缓解范围失配）。
 *
 * 关于「并存」与 Flash：
 *   - 本宏只决定「运行时真正执行哪条部署路径」。
 *   - freertos.c 的 DIV2IFSCN_Inference（float32 oracle）始终随工程编译，
 *     作为 A-B 对照基准，保留即可。
 *   - 当你确认只烧录 float32 版时，把 AI_INCLUDE_INT8_WEIGHTS 设 0，
 *     int8 权重（约 5KB Flash）即不编入。int8 模式被强制为 1。
 * ---------------------------------------------------------------------------
 */
#ifndef AI_CONFIG_H
#define AI_CONFIG_H

/* ===== 精度模式枚举 ===== */
#define AI_PRECISION_FLOAT32       0
#define AI_PRECISION_INT8_HYBRID   1   /* FC1 i8 + FC2 f32  (混合1) */
#define AI_PRECISION_FC1F32_FC2I8  2   /* FC1 f32 + FC2 i8  (混合2) */
#define AI_PRECISION_FC1I8_FC2I8   3   /* FC1 i8 + FC2 i8  (混合3, 单scale) */
#define AI_PRECISION_FULL_INT8     4   /* 全int8 (混合4, 分段) */

/* ===== 模型维度（单一来源，float32 / int8 共用） ===== */
#define AI_INPUT_DIM    10   /* 输入特征维 */
#define AI_NUM_RULES    50   /* 模糊规则数 = H0 维 */
#define AI_HIDDEN_DIM   100  /* 隐层维 */
#define AI_Z_DIM        150  /* Z_final = H0(50) + hidden(100) */
#define AI_OUTPUT_DIM   4    /* 诊断类别数 */

/* 默认：全 int8（AI_PRECISION_FULL_INT8；PC 预校验 88.72% / 硬件 88.66%）。
   追求最高精度可改 AI_PRECISION_INT8_HYBRID（PC 90.97% / 硬件 90.90%）。改这里即可切换。 */
#ifndef AI_PRECISION
#define AI_PRECISION  AI_PRECISION_FULL_INT8
#endif

/* ===== 由 AI_PRECISION 派生的逐层开关（供 ai_infer.c 使用） ===== */
#define FC1_IS_INT8  (AI_PRECISION == AI_PRECISION_INT8_HYBRID || \
                      AI_PRECISION == AI_PRECISION_FC1I8_FC2I8  || \
                      AI_PRECISION == AI_PRECISION_FULL_INT8)
#define FC2_IS_INT8  (AI_PRECISION == AI_PRECISION_FC1F32_FC2I8 || \
                      AI_PRECISION == AI_PRECISION_FC1I8_FC2I8  || \
                      AI_PRECISION == AI_PRECISION_FULL_INT8)
#define FULL_INT8     (AI_PRECISION == AI_PRECISION_FULL_INT8)   /* 分段 Z_final */

/* ===== int8 量化权重是否编入固件 =====
 * 0 = 不编入（仅当确认只烧 float32 版时）；1 = 编入（默认）。
 * 任意 int8 模式被强制为 1。 */
#ifndef AI_INCLUDE_INT8_WEIGHTS
#define AI_INCLUDE_INT8_WEIGHTS  1
#endif

#if (AI_PRECISION != AI_PRECISION_FLOAT32)
  #undef  AI_INCLUDE_INT8_WEIGHTS
  #define AI_INCLUDE_INT8_WEIGHTS  1
#endif

/* ===== 运行期/编译期可读的模式名（启动日志 & 编译信息确认用） ===== */
#if (AI_PRECISION == AI_PRECISION_FLOAT32)
  #define AI_PRECISION_NAME  "FLOAT32"
#elif (AI_PRECISION == AI_PRECISION_INT8_HYBRID)
  #define AI_PRECISION_NAME  "INT8_HYBRID"
#elif (AI_PRECISION == AI_PRECISION_FC1F32_FC2I8)
  #define AI_PRECISION_NAME  "FC1F32_FC2I8"
#elif (AI_PRECISION == AI_PRECISION_FC1I8_FC2I8)
  #define AI_PRECISION_NAME  "FC1I8_FC2I8"
#else
  #define AI_PRECISION_NAME  "FULL_INT8"
#endif

#endif /* AI_CONFIG_H */
