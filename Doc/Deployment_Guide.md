# 模型部署与实测指南（Deployment Guide）

> 适用项目：`AI_deploy`（STM32H743VIT6 + ESP32-S3 CAM）  
> 用途：归档「两端 AI 模型」的部署方案与实测性能数据 —— STM32 故障诊断（INT8）与 ESP32-S3 图像目标检测（FOMO/INT8）。  
> 关联文档：`peripheral_plan.md`（外设与 NN 分组）、`OTA.md`（整机 OTA）、`Readme.md`（工程总览）。

---

## 1. 部署架构与分工

| 模型 | 部署平台 | 任务类型 | 状态 |
|---|---|---|---|
| 故障诊断（DIT2IFSCN） | **STM32 H743**（本地、已成型） | 10→50→100+50→4 类，INT8 全连接 | ✅ 已部署，继续保留在 STM32 |
| 球磨机图像目标检测 | **ESP32-S3 CAM**（GC2145） | FOMO 目标检测，INT8 | ✅ 已部署（ESP32-S3 端实测） |

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
- **实测推理耗时**：Release 约 0.8 s/次（1605 样本）→ **单样本 0.498 ms ≈ 0.50 ms**；
  调试模式曾出现 `macro_precision` 观察为 0 的假象，串口实测 `macro_precision = 0.91128` 正常（debug 观测误差，非代码缺陷）。

#### 2.1.1 INT8 vs FP32 精度对比（1605 样本，用户实测 2026-09-02）

| 指标 | 数值 | 含义 |
|---|---|---|
| **FP32 模型精度** | **94.70%** | FP32 模型「预测对」的比例（绝对正确率） |
| **INT8 模型精度** | **90.90%**（acc）/ 0.91128（macro_precision） | INT8 模型「预测对」的比例 |
| **INT8 ↔ FP32 预测一致率** | **94.5%** | 两者「预测结果相同」的样本占比（**相对一致性**） |

> ⚠ **三个指标不可互换**，面试/答辩务必分清：
> - 「**精度**」= 模型预测 vs **真实标签**（绝对正确率）
> - 「**一致率**」= INT8 预测 vs **FP32 预测**（相对一致性，**与标签无关**）
>
> 一致率 94.5% **高于** INT8 精度 90.90% 是正常的：即使两者都判错，只要判成同一类，
> 一致率仍然计数。故「量化几乎无损」这类结论**不能**用一致率支撑，必须看精度差
> （本例 94.70% − 90.90% = **3.80 个百分点**）。
>
> **待补**：一致率的原始记录（样本数 / 判定脚本 / 输出文件）。当前仅有数值，无归档，
> 对外引用前建议补齐，否则被追问「怎么算的」无法自证。

### 2.2 部署要点
- 推理工作区 `.tensor_arena` 以 `__attribute__((section(".tensor_arena")))` 放在 RAM（参考 tflite_learn_*.cpp）。
- 工程分组：`FaultDiag_NN` 仅含 `arm_fully_connected_s8.c` + `arm_nn_vec_mat_mult_t_s8.c` + `ai_infer.c`（详见 `peripheral_plan.md §11.9`）。

###  ️2.3 STM32 目标检测（TFLite INT8）—— 实测
> 注：本节是 **STM32 端跑目标检测算法**（与 §2.1 故障诊断、§3 ESP32-S3 检测并列，三项独立）。

- 方案：**TFLite INT8 + NN 加速 + EON 优化**（CMSIS-NN / EON Compiler）。
- **F1 = 0.84**，较 PC 端 FP32 损失约 **8%**（FP32 基准约 0.917）。
- **Flash 占用**：544 KB → **154 KB**（优化后降幅约 72%；注意此处是 Flash，非运行内存——RAM 恒定 4.0 KB）。
- **单帧推理 81 ms ≈ 12 FPS 吞吐**。

| 指标 | STM32 目标检测 (INT8) | 备注 |
|---|---|---|
| 精度 F1 | 0.84 | 较 PC FP32 损失 8% |
| Flash 占用 | 544 → 154 KB | EON 优化后（RAM 恒定 4.0 KB） |
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

> ⚠ 上表为 **Edge Impulse 在 ESP-EYE 目标设备上导出的 profile 估算值**，非本机实测；
> 其中 `Accuracy 76.92%` 是 **float32 未优化版**的精度，INT8 列 EI 未给。

#### 3.1 ESP32-S3 板级实测精度（INT8，用户实测 2026-09-02）

| 指标 | 实测值 | 说明 |
|---|---|---|
| **Precision** | **0.92** | 非背景类 |
| **F1 Score** | **0.93** | 非背景类 |

- **与 EI profile 的关系**：profile 表未列 INT8 精度，本节实测补齐该空缺。
- **与 `Components/Object_Detection/目标检测性能.md` 的关系**：该 md 记录的是 **STM32 端**跑同一 FOMO 的
  指标（验证集 P=0.95/R=0.88/F1=0.92，检测 P=0.90/R=0.78/F1=0.84）。两端同模型不同指标，
  **引用时必须显式标注平台**，勿混用（见 §4 警示）。
- **待补**：实测样本数 / 数据集划分 / 置信度阈值 / Recall —— 补齐后方可对外引用。

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
| 精度（检测） | **F1 = 0.84 / P = 0.90**（较 FP32 损失 8%） | **F1 = 0.93 / P = 0.92**（板级实测，§3.1） | 同模型两端指标不同 |
| 精度（验证集分类） | F1 = 0.92 / P = 0.95 | — | 见 `目标检测性能.md` |
| 存储占用 | **Flash 544.7 → 153.7 KB** | Flash 153.7 KB / RAM 4.0 KB | ⚠ **削减的是 Flash，不是 RAM**（RAM 恒定 4.0 KB） |
| 单帧推理 | **81 ms ≈ 12 FPS** | **1,026 ms ≈ 1 FPS**（detect 1,011 ms） | STM32 快约 12.7× |
| 角色 | 本地高速推理（若需） | 摄像头前端检测 + 回传结论 | 见 §1 分工 |

> ⚠ **引用警示（2026-09-02 补）**：本表两列的 81 ms 与 1,026 ms 是**同一个 FOMO 模型在两个目标设备上的
> EI profile 延迟**（两表 RAM/Flash/Activation/fp32 精度完全相同，只有延迟不同）。
> 对外表述（简历/答辩）时务必**显式标注平台** —— 曾出现把 ESP32-S3 的 `1,011 ms` 误当作
> STM32 H743 推理延迟的情况，导致"1 秒推理与 200Hz 姿态环如何共存"的自相矛盾。

**结论**：
- STM32 端目标检测推理吞吐（12 FPS）显著高于 ESP32-S3（~1 FPS），若后续需要「在 STM32 本地做视觉检测」具备性能余量；ESP32-S3 侧受相机采集 + FOMO 全流程限制，约 1 FPS。
- 两者内存/Flash 同量级（150 KB 级），量化收益一致；ESP32 的 8 MB PSRAM 可容纳更大 arena，STM32 需盯紧内部 RAM 预算。
- 当前分工仍以「ESP32-S3 做图像检测、STM32 做故障诊断」为主，本表仅作能力对比，不改动 §1 分工结论。

---

## 5. 下一步
- STM32 侧：USART6 接收状态机（`peripheral_plan.md §11.6`）接收 ESP32 结论。
- ESP32 侧：GC2145 取 帧 + EdgeImpulse FOMO 推理 + 按 §11.6 帧格式上报。
- 双端联调：关闭 `EdgeImpluse_NN` 仍可编译 STM32；开启两者可做对照测试（见 `peripheral_plan.md §11.9`）。
