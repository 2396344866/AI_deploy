# AI_deploy · STM32H743 工业边缘 AI 故障诊断 + ESP32-S3 图像协处理

> 主控制器 **STM32H743VIT6**（FreeRTOS / CMSIS-RTOS2）运行铜磨矿故障诊断 INT8 模型；**ESP32-S3 CAM**（N16R8 + GC2145）作为图像协处理器跑球磨机 FOMO 目标检测。工业级日志分级 + 看门狗 + 黑匣子。

## 硬件架构
| 角色 | 器件 | 职责 |
|------|------|------|
| 主控 MCU | STM32H743VIT6 (Cortex-M7, 480MHz) | 故障诊断 INT8 推理、系统调度、POST/看门狗/黑匣子 |
| 图像协处理 | ESP32-S3 CAM (N16R8 + GC2145) | 球磨机 FOMO 目标检测（Edge Impulse EON Compiler, INT8, kTensorArenaSize=155008B） |
| 外部 Flash | W25Q64 | 日志黑匣子 / 模型权重点 |
| 传感器 | MPU6050 | 姿态（若 profile 含 SENSOR） |
| 调试接口 | ST-Link V2 + UART1(PA9/PA10, 921600) | 烧录 + 控制台日志 |

## 软件栈
- **RTOS**：FreeRTOS（CMSIS-RTOS2 封装）；任务对象 CubeMX 生成，应用逻辑在 `Components/` 与 `Core/Src` 的 USER CODE 块。
- **日志 Logger**：六级（FATAL/ERROR/WARN/INFO/DEBUG/TRACE）+ 环形缓冲 + 级别过滤 + 异步刷 UART1 + `flush_to_flash` 黑匣子；Channel B 保证通道承载系统里程碑（绕过运行级必落线）。
- **调试门控**：三层正交门控——L1 `APP_PROFILE_*`/`APP_ENABLE_X`（功能包含）、L2 `LOG_COMPILE_MAX_LEVEL`/`LOG_RUNTIME_DEFAULT_LEVEL`（级别）、L3 `DBG_LOG_<TASK>`/`DBG_TELEMETRY_ENABLE`（逐任务文本与遥测）。
- **AI 推理**：POST/Postest 架构；故障诊断 INT8 模型（macro_precision≈0.911）；FOMO 部署在 ESP32-S3（INT8 基线 F1≈0.84 / 12 FPS / 154KB）。
- **故障恢复**：IWDG1（≈4.1s）+ TIM7 心跳喂狗（500ms 按任务心跳新鲜度）；关键失败刷黑匣子 + `NVIC_SystemReset`。

## 目录结构
```
AI_deploy/
├── Core/                  # CubeMX 生成（main/freertos/stm32h7xx_it/usart…）；仅改 USER CODE 块
├── Components/
│   ├── BSP/               # 板级驱动：LOG / LED / W25Q64 / ESP / IMU
│   ├── Logger/            # 统一日志（分级/时间戳/环形缓冲/黑匣子）
│   ├── Debug/             # 看门狗心跳、调试开关、Error/<TASK>_error.md 故障档案、Test/ 测试方案
│   ├── Fault_Diagnosis/   # 故障诊断推理模块（INT8 权重/数据）
│   ├── POSTest/           # Postest.c（POST 自检表，与 APP_ENABLE_X 同一真相源）
│   ├── Motor/             # 双电机闭环（TB6612FNG + 级联 PI）
│   ├── Object_Detection/  # FOMO 目标检测（ESP32-S3 侧对齐）
│   ├── EdgeImpulse/       # EON Compiler 导出
│   ├── CMSIS_DSP/ FreeRTOS_Port/   # 第三方/移植
├── Drivers/ Middlewares/  # HAL / CMSIS / FreeRTOS 内核（勿手改）
├── MDK-ARM/               # Keil 工程（*.uvprojx 禁止手改 IncludePath/FilePath）
├── Doc/                   # 文档（调试手册 / 排错索引 / 设计说明 / 部署指南）
└── ML/                    # 模型训练/校验脚本与数据
```
> 自定义代码只放 `Components/`，勿放 `Core/`（防 CubeMX 重生覆盖）。Keil 工程文件（`.uvprojx`）不手改——新增 `.c` 需用户在 Keil 源组手动加。

## 构建与烧录（Keil 自维护）
1. 打开 `MDK-ARM/STM32H743VIT6.uvprojx`。
2. 确认源组就位：`POSTest/Src`、`Debug/Src`（含 `watchdog_heartbeat.c`）、`Logger/`、`BSP/*`、`Fault_Diagnosis/` 等。
3. `Project → Rebuild all target files` → **0 Error / 0 Warning**。
4. `Flash → Download` → 复位；UART1（921600 8N1）看 `POSTEST` 进度。
> 详细排错（编译/链接/uvprojx 故障）见 `Components/Debug/Error/Error_Readme_idx.md`。

## 宏配置速查
| 关注点 | 文件 | 关键宏 |
|--------|------|--------|
| 功能包含（总闸） | `Components/Debug/Inc/app_config.h` | `APP_PROFILE_*` / `APP_ENABLE_X` |
| 日志级别 | `Components/Logger/Inc/logger.h` | `LOG_ENABLED` / `LOG_COMPILE_MAX_LEVEL` / `LOG_RUNTIME_DEFAULT_LEVEL` |
| 逐任务调试 | `Components/Debug/Inc/dbg_config.h` | `DBG_LOG_<TASK>` / `DBG_TELEMETRY_ENABLE` / `DBG_LOG_POSTEST` |

> 速查图/表与运行期调级（`debugX` 命令）见 **`Doc/调试手册.md`**。

## 测试
- POST 上电自检方案与五要素操作步骤：``Components/POSTest/POST.md``（已含 2026-08-30 LOGGER profile 全项 PASS 实测）。
- 其他模块测试：``Components/Debug/Test/*.md``。

## 文档导航（Doc/）
| 文档 | 内容 |
|------|------|
| `调试手册.md` | DEBUG/LOGGER/APP 三层宏配置速查（图/表为主） |
| `Components/Debug/Error/Error_Readme_idx.md` | 编译/链接/uvprojx 故障**索引**（细节下沉 `Components/Debug/Error/`） |
| `BSP.md` | 板级驱动设计（UART/Flash/LED/ESP） |
| `Pinout.md` | 引脚与时钟配置源 |
| `Deployment_Guide.md` | STM32 故障诊断 + ESP32-S3 图像检测部署方案与实测性能 |
| `OTA.md` / `UWB_BU03_移植指南.md` / `ESP32S3_接收模块_设计说明.md` | 各专项 |
| `GitHub上传指南.md` | 推送到 GitHub 流程与 .gitignore 约定 |

## 约束与约定（铁律）
- **CubeMX 边界**：`Core/` 生成文件严禁手改；要配置去 `.ioc`，等"已完成"再对接。
- **Keil 边界**：不手改 `.uvprojx`（IncludePath/FilePath）；新增源文件由用户加源组。
- **故障归档**：出 bug 先查 `Components/Debug/Error/<TASK>_error.md` + `post_error.md`（运行期）、`compile_link_error.md`（编译链接）、`uvprojx_error.md`（uvprojx）、`crash_error.md`（死机取证）。
- **私密凭据**：`secrets.h` 已被 `.gitignore` 排除，真实 MQTT 密码派生值走此文件，勿提交。

> 更细的组件规范与接口说明见 `Doc/Readme.md`。
