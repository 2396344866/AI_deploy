# 模型部署与实测指南（Deployment Guide）

> 适用项目：`AI_deploy`（STM32H743VIT6 + ESP32-S3 CAM）  
> 用途：归档「两端 AI 模型」的部署方案与实测性能数据 —— STM32 故障诊断（INT8）与 ESP32-S3 图像目标检测（FOMO/INT8）。  
> 关联文档：`peripheral_plan.md`（外设与 NN 分组）、`OTA.md`（整机 OTA）、`Readme.md`（工程总览）。

---

## 1. 部署架构与分工

| 模型 | 部署平台 | 任务类型 | 状态 |
|---|---|---|---|
| 故障诊断（DIT2IFSCN） | **STM32 H743**（本地、已成型） | 10→50→100+50→4 类，INT8 全连接 | 已部署，继续保留在 STM32 |
| 球磨机图像目标检测 | **ESP32-S3 CAM**（GC2145） | FOMO 目标检测，INT8 | 待迁移（新增） |

- STM32 故障诊断：权重随 APP 固件编译进内部 Flash（`.rodata`），由 `ai_infer.c` 调用 CMSIS-NN（`arm_fully_connected_s8`）推理。
- ESP32-S3 图像检测：Edge Impulse 导出 Arduino/C++ 库，经 USART6 把目标结论回传 STM32（协议见 `peripheral_plan.md §11.6`）。

---

## 2. STM32 H743 故障诊断（INT8）—— 代码事实与实测

### 2.1 结构与精度（PC 预校验，1605 样本，对齐 PyTorch 94.70%）
- 结构：input10 → H0(50) → Z_final(150) → 4 类；FC1 5000 MAC / FC2 600 MAC。
- 权重 7700 floats ≈ 30.1 KB，放 `.dtcmram`（零等待、CPU 专用）。
- 加速：gamma LUT 替 powf / 预计算 inv_sigma / CMSIS-DSP f32 算子。

| 方案 | 精度 |
|---|---|
| f32（全浮点） | 94.70% |
| HYBRID（FC1 f32 + FC2 i8） | 90.97% |
| FC1f32 + FC2i8 | 91.59% |
| FC1i8 + FC2i8 | 90.03% |
| **FULL_INT8** | **88.72%** |

- 当前 DIT2 模型：γ=0.7，p=±0.3，H0 absmax ±1.83（无范围失配，FC2 int8 可行）。
- **实测推理耗时**：Release 约 0.8 s/次（1605 样本）；调试模式曾出现 `macro_precision` 观察为 0 的假象，串口实测 `macro_precision = 0.91128` 正常（debug 观测误差，非代码缺陷）。

### 2.2 部署要点
- 推理工作区 `.tensor_arena` 以 `__attribute__((section(".tensor_arena")))` 放在 RAM（参考 tflite_learn_*.cpp）。
- 工程分组：`FaultDiag_NN` 仅含 `arm_fully_connected_s8.c` + `arm_nn_vec_mat_mult_t_s8.c` + `ai_infer.c`（详见 `peripheral_plan.md §11.9`）。

###  ️2.3 STM32 目标检测（TFLite INT8）—— 实测
> 注：本节是 **STM32 端跑目标检测算法**（与 §2.1 故障诊断、§3 ESP32-S3 检测并列，三项独立）。

- 方案：**TFLite INT8 + NN 加速 + EON 优化**（CMSIS-NN / EON Compiler）。
- **F1 = 0.84**，较 PC 端 FP32 损失约 **8%**（FP32 基准约 0.917）。
- **内存**：544 KB → **154 KB**（优化后降幅约 72%）。
- **单帧推理 81 ms ≈ 12 FPS 吞吐**。

| 指标 | STM32 目标检测 (INT8) | 备注 |
|---|---|---|
| 精度 F1 | 0.84 | 较 PC FP32 损失 8% |
| 内存 | 544 → 154 KB | EON 优化后 |
| 单帧推理 | 81 ms | ≈ 12 FPS |

---

## 3. ESP32-S3 图像目标检测（FOMO / INT8）—— Edge Impulse Profile 实测

> 来源：Edge Impulse 在 **ESP32（ESP-EYE 目标设备）** 导出的 profile 数据（球磨机图像目标检测，FOMO）。

| 指标 | Quantized (INT8) | Unoptimized (float32) |
|---|---|---|
| **Image 延迟** | 15 ms | 15 ms |
| **Object detection 延迟** | 1,011 ms | 4,693 ms |
| **Total 延迟** | **1,026 ms** | **4,708 ms** |
| **RAM** | 4.0 KB | 4.0 KB |
| **Flash** | 153.7 KB | 544.7 KB |
| **Activation / 中间张量** | 67.8 KB | 99.8 KB |
| **Accuracy** | — | 76.92% |

### 解读与结论
- **INT8 比 float32 快约 4.6×**（1026 ms vs 4708 ms），Flash 占用从 544.7 KB 降到 153.7 KB —— **ESP32-S3 侧务必用 INT8（EON）版本**，与 STM32 故障诊断的 INT8 路线一致。
- INT8 表内未列 Accuracy 数值；EI 量化通常损失极小。若验收时精度不足，再考虑混合精度或回退 float32（代价是 4.7× 延迟与 3.5× Flash）。
- ESP32-S3 有 8 MB PSRAM，tensor arena 优先放 PSRAM（`EI_TENSOR_ARENA_LOCATION`），避免挤占内部 RAM。GC2145 在 EI 官方 firmware 无现成配置，需自填 `camera_pins` + SCCB 地址。
- 与 STM32 分工：ESP32 图像检测 INT8 ≈ 1.0 s/次（含 detect 全流程），与 STM32 故障诊断各自独立运行，互不抢占。

---

## 4. STM32 vs ESP32-S3 目标检测对比（INT8）

> 两者均为 INT8 量化 + EON/NN 优化路线，但分属不同端、不同模型结构，仅供横向参考。

| 维度 | STM32 H743 目标检测 | ESP32-S3 图像检测（FOMO） | 说明 |
|---|---|---|---|
| 平台 | STM32H743VIT6（480 MHz M7） | ESP32-S3（N16R8，8MB PSRAM） | 同项目双端 |
| 方案 | TFLite INT8 + NN 加速 + EON | Edge Impulse FOMO + INT8 | 均量化 |
| 精度 | **F1 = 0.84**（较 FP32 损失 8%） | 未单列（FP32 基准 76.92%） | STM32 以 F1 衡量，ESP32 以 acc |
| 内存占用 | 544 → **154 KB** | 4 KB RAM / 153.7 KB Flash | STM32 指运行内存，ESP32 指 Flash |
| 单帧推理 | **81 ms ≈ 12 FPS** | 1,026 ms ≈ 1 FPS | STM32 快约 12.7× |
| 角色 | 本地高速推理（若需） | 摄像头前端检测 + 回传结论 | 见 §1 分工 |

**结论**：
- STM32 端目标检测推理吞吐（12 FPS）显著高于 ESP32-S3（~1 FPS），若后续需要「在 STM32 本地做视觉检测」具备性能余量；ESP32-S3 侧受相机采集 + FOMO 全流程限制，约 1 FPS。
- 两者内存/Flash 同量级（150 KB 级），量化收益一致；ESP32 的 8 MB PSRAM 可容纳更大 arena，STM32 需盯紧内部 RAM 预算。
- 当前分工仍以「ESP32-S3 做图像检测、STM32 做故障诊断」为主，本表仅作能力对比，不改动 §1 分工结论。

---

## 5. 下一步
- STM32 侧：USART6 接收状态机（`peripheral_plan.md §11.6`）接收 ESP32 结论。
- ESP32 侧：GC2145 取 帧 + EdgeImpulse FOMO 推理 + 按 §11.6 帧格式上报。
- 双端联调：关闭 `EdgeImpluse_NN` 仍可编译 STM32；开启两者可做对照测试（见 `peripheral_plan.md §11.9`）。
