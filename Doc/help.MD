# 模型推断加速方案（DIV2IFSCN on STM32H743）

> 本文档说明 `AI_deploy` 中故障诊断模型在 MCU 上的推理加速手段与实测性能。模型为手写模糊广度学习分类器 **DIV2IFSCN**，实现见 `Core/Src/freertos.c` 的 `DIV2IFSCN_Inference()`；权重见 `Components/AI/Inc/model_weights.h`。
> ⚠️ 注意：本项目使用 **CMSIS-DSP 的 f32 路径**，**未做 int8 量化**，也未调用 CMSIS-NN 的 int8(s8) 算子；权重为 float32。

## 1. 为什么能跑这么快（三层加速）

### 1.1 LUT 替代昂贵的 powf（核心加速）
隶属度计算里每个规则需要 `powf(1 - powf(x, γ), 1/γ)`（γ=0.6），纯 `powf` 在 M7 上极慢。实现改为**首次推理时预计算 1024 点查找表** `gamma_map_lut[1024]`：

```c
// 首次推理初始化（is_params_initialized 守护，只跑一次）
for (int i = 0; i < GAMMA_LUT_SIZE; i++) {
    float x = (float)i / (GAMMA_LUT_SIZE - 1);
    gamma_map_lut[i] = powf(1.0f - powf(x, ivifs_gamma), pre_inv_gamma);
}
// 推理时线性插值替代 powf
static inline float fast_gamma_Test(float x) {
    if (x < 0.0f) x = 0.0f; else if (x > 1.0f) x = 1.0f;
    float pos = x * (GAMMA_LUT_SIZE - 1);
    int idx = (int)pos;
    if (idx >= GAMMA_LUT_SIZE - 1) return gamma_map_lut[GAMMA_LUT_SIZE - 1];
    float fraction = pos - (float)idx;
    return gamma_map_lut[idx] + fraction * (gamma_map_lut[idx + 1] - gamma_map_lut[idx]);
}
```

把每规则 4 次 `powf` 降到 1 次插值查表，是本项目最大的单点加速。

### 1.2 预计算倒数，除法变乘法
`inv_sigma_upper_sq[i] = 1/(sigma_upper[i]²)` 在初始化阶段算好，推理内层循环用乘法替代除法。

### 1.3 CMSIS-DSP f32 算子替代手写循环
- 增强层矩阵乘：`arm_mat_mult_f32(&W(100×50), &H0(50×1), &H_enh(100×1))`
- 偏置加：`arm_add_f32`
- 输出层矩阵乘：`arm_mat_mult_f32(&Z_final(1×150), &beta(150×4), &outputs(1×4))`
- 取最大类：`arm_max_f32(outputs, 4, &max, &idx)`
- 开方：`arm_sqrt_f32`

## 2. 内存布局：权重驻 DTCM
7 个权重数组全部 `__attribute__((section(".dtcmram")))`：
- DTCM 紧耦合、零等待、CPU 专用、DMA 不可访问；
- 权重只被 CPU 密集读取、不 DMA，故适合放 DTCM，规避 D-Cache miss 与总线仲裁延迟；
- 总权重 7700 floats ≈ 30.1 KB。

## 3. 实测性能

| 指标 | 数值 | 来源 |
|---|---|---|
| 测试样本数 | 1605 | `test_dataset_processed.h` `NUM_TEST_SAMPLES` |
| 总体准确率 | 0.9520 | `StartInferenceTask` 日志 `acc=` |
| 单样本推理 | ~0.5 ms | 日志 `per=` |
| 模型权重体积 | ~30.1 KB（float32，DTCM） | `model_weights.h` |

## 4. 验证方法（误差闭环）
`StartInferenceTask` 在启动后对整个测试集跑一遍批量推理，统计 `tp/fp/fn` 计算 macro-Precision/Recall/F1 与 `overall_accuracy`，经 LoggerTask 异步刷串口。若要验证 LUT 路径相对纯 `powf` 的误差，可在初始化里保留一份纯 `powf` 对照，逐样本比最大偏差。
