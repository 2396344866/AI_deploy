# DIV2IFSCN 部署指南（int8 混合 / STM32H743 / Keil MDK）

权重量化已在 PC 自检通过；硬件指标以你实测为准（见 DOC_PRECISION_COMPARISON.md）。

## 结论
| 层 | 实现 | 算子 |
|----|------|------|
| 模糊前端 H0(50) | float32 | expf / arm_sqrt_f32 / gamma LUT |
| FC1(5000MAC) | int8 per-tensor | arm_fully_connected_s8 |
| 激活 | float32 | 1/(1+expf) |
| Z_final(150) | float32 | concat |
| FC2(600MAC) | int8 可选 / f32 备选 | arm_fully_connected_s8 / arm_mat_mult_f32 |

- FC1 与 FC2 均可 int8（per-tensor）；FULL_INT8 硬件
- ±0.002 = PC f32 vs MCU f32 推理偏差（FPU 差异），非量化误差
- Flash -45% 未达成：f32 oracle 与 int8 权重并存（见 §内存）

## 源文件
```
Components/AI/Inc: model_weights.h, model_weights_ext.h, model_weights_q.h,
  model_q_params.h, fuzzy_lut.h, fuzzy_frontend.h, ai_infer.h, ai_config.h
Components/AI/Src: fuzzy_frontend.c, ai_infer.c
ML/: quantize_ptq.py, quant_roundtrip_check.py, train_div2ifscn.py(可选)
```

## 工程接入（Keil MDK）
1. 加 fuzzy_frontend.c + ai_infer.c 到工程；Inc 已在 include 路径
2. CMSIS-NN vendored V1.1.0 已编入（仅用 arm_fully_connected_s8）
3. 推理入口：CubeMX 原生 Task_Inference 直接调 AI_Inference()（无需独立线程）
4. D-Cache：权重在 .dtcmram 常量，无一致性问题
5. 编译无 multiple definition（static const / extern 规避）

## 精度模式（ai_config.h, 编译期）
- 默认 AI_PRECISION = FULL_INT8（ai_config.h:45）
- INT8_HYBRID 精度最优（PC 90.97% / 硬件 90.90%）；FULL_INT8 最省（88.66%）
- AI_INCLUDE_INT8_WEIGHTS：int8 模式强制 1；纯 f32 可设 0 省 ~5KB
- 确认：构建 #pragma AI_DEPLOY_PRECISION；.map 引用 arm_fully_connected_s8 即含 int8
- 注：StartInferenceTask 始终跑 f32 oracle（A-B 对照），不受 AI_PRECISION 影响

## 量化流水线（PC）
```
python quantize_ptq.py [--calib-csv real.csv]
python quant_roundtrip_check.py
```
- 标定用类内近中心分布，勿均匀 [0,1]（会放大模糊尖峰、scale 过粗）

## FC2 int8 演进
- 旧模型(γ=0.6, p=±3)：H0 absmax±10.4，与 hidden 差 ~10× → FC2 int8 精度 95.2%→70%
- 当前 DIT2(γ=0.7, p=±0.3)：H0 absmax±1.83，同量级 → FC2 int8 可行（FULL_INT8 88.66%）

## 硬件验证
1. f32 vs PC f32：DIV2IFSCN_Inference 跑 test_features_processed，比对 logits（亚千分位）
2. int8 vs PC f32：AI_Inference，比对一致率
3. 延迟：Task_Inference 内 HAL_GetTick() 打点；<0.5ms（FC1 int8 更省 MAC/带宽）

## 内存
```
W_hid_in_q 5KB(DTCM) | 运行时<1.5KB | beta 等~24KB | FC+模块 ~数KB(Flash)
```
- Flash -45% 未兑现：f32+int8 并存；移除 oracle 可省 15KB(FC1 20→5KB)

## 易错
- fc_params.activation={AI_ACT_MIN,AI_ACT_MAX}(-128/127)；设 {0,0} 输出全夹 0
- FC1：filter_dims.n=输入(50)，output_dims.c=输出(100)
- H0 量化 round(H0/SCALE_IN1_F) 裁 int8；zero_point=0
- beta 经 model_weights_ext.h extern；多 TU：q/lut 用 static const；model_weights.h 仅 freertos.c
- D-Cache：DTCM 无需维护；带缓存 SRAM+DMA 补 SCB_*Cache*
