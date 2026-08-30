# 运行期故障归档索引（Error/）

> 本目录是 **运行期（runtime）故障** 的归属地：固件已编译、烧录后在板子上跑出来的
> IMU/I2C/UART/姿态算法/陀螺硬件/启动卡死等问题，按 FreeRTOS 任务拆分归档。
> 总导航入口仍然是 `Components/Debug/error.md`（本目录的父级），它列出每个事件到本目录文件的映射。
> 本文件是 **「如何新增一个任务的调试/故障归档」规范 + 目录**，供后续人照做。

## 一、三类文档的命名与分工（铁律）

每个 FreeRTOS 任务（`StartXxxTask`）对应三份并行文档，命名与任务一一对应：

| 文档类型 | 路径 | 内容 | 是否进编号体系 |
|----------|------|------|----------------|
| 联调工作笔记（非故障） | `Components/Debug/debug/<TASK>_debug.md` | 状态/接线/设计/命令/栈堆/Bug 链（已修复） | 否 |
| 运行期故障归档 | `Components/Debug/Error/<TASK>_error.md` | 板子跑出来的异常，用 **事件 E N** 编号 | 是（E 编号） |
| 测试方案 | `Components/Debug/Test/<TASK>_test.md` | 目标/环境/用例表/量化 KPI | 否 |

> **全局参考（非按任务，不放本目录）**：跨任务的唯一参考源统一收口到：
> - `Components/Debug/Ref/` —— 门控总表 `log_gating.md`、VOFA 44 通道映射 `vofa_telemetry.md`；启动死机取证见 `Error/crash_error.md`。
> - `Components/Debug/tools/` —— 调试脚本 `verify_dbg_frame.py`（遥测帧校验）。
> 它们与 per-task 三件套正交：per-task 文档讲"某任务怎么调/哪里坏/怎么测"，`ref/` 讲"全系统共用的开关/通道/取证方法论"。新增全局参考直接进 `ref/`，勿在 Debug 根散落。

> 现有实例：`debug/motor_debug.md`（联调）、`Error/motor_error.md`（故障）、`Test/motor_test.md`（测试）。
> 命名与 `freertos.c` 的 `StartXxxTask` 对齐：`MOTOR`→`motor`、`SENSOR`→`sensor`、`SCREEN`→`screen`、
> `LOGGER`→`logger`、`DIAG`→`diag`、`NET`/`FLASH`/`ESP32` 同理。

## 二、层级边界（绝不混编）

| 故障类型 | 归属文件 | 编号 |
|----------|----------|------|
| 运行期板子行为故障（IMU/I2C/UART/姿态/硬件坏/业务偏离） | 本目录 `Error/<TASK>_error.md` | **事件 E N**（全局连续，不按任务重置） |
| 编译期 / 工程配置故障（uvprojx 打不开、Build 报错、CubeMX 翻倍、L6200E 等） | `Doc/Keil_MDK_ARM_工程排错记录.md` | **问题 N** |
| 启动卡死 / HardFault 通用取证方法论 | `Error/crash_error.md` | 按机理分节（具体实例仍记 E 事件） |
| AI 误判诚实归档（跨任务） | `Components/Debug/error.md` 底部集中保留 | — |

## 三、新增一个任务调试/故障归档的步骤

1. **联调笔记**：在 `Components/Debug/debug/` 建 `<TASK>_debug.md`，写状态/接线/设计/命令/栈堆；
   首行注明「本文件是 <TASK> 联调工作笔记，非故障归档；运行期故障归 `Error/<TASK>_error.md`」。
2. **故障归档**：在 `Components/Debug/Error/` 建 `<TASK>_error.md`；
   新故障用全局连续 **事件 E N**（接着已有最大编号，不重置）；每个事件含「现象 / 调试器定位 / 根因链 / 修复方案 / 验证步骤 / 状态」。
3. **测试方案**：在 `Components/Debug/Test/` 建 `<TASK>_test.md`，按三段式：
   ① 目标/环境/数据策略/判据/流程；② 用例表（正常值/边界值/异常值/性能压力/抗噪）；③ 量化 KPI（功能覆盖/性能/鲁棒性）。
4. **登记**：在 `Components/Debug/error.md` 的映射表加一行，把新事件指向 `Error/<TASK>_error.md#事件-eN`。
5. **交叉引用**：若其它文档（设计说明/手册）提及该故障，链到 `Error/<TASK>_error.md#事件-eN`，不再写死 `error.md`。

## 四、当前目录映射（8 任务全量，按任务分类）

> **分类决策（按任务，非按元器件）**：本工程调试开关本就按任务命名（`DBG_LOG_<TASK>` 一一对应 8 个
> `StartXxxTask`），且**每个任务天然拥有其硬件元器件**（SENSOR 拥有 MPU6050、MOTOR 拥有电机驱动、
> NET 拥有 ESP-01S、FLASH 拥有 W25Q64、ESP32 拥有 ESP32-S3 CAM、SCREEN 拥有淘晶驰串口屏、LOGGER 拥有
> 日志引擎/黑匣子、DIAG 拥有 INT8 推理模型）。故「按任务」即与代码同构、且自动覆盖「按元器件」，
> 避免 MPU6050 在 SENSOR/遥测/ESP32 图像流水线多处重复归档。**工业做法同理**：AUTOSAR 按 SWC、
> Zephyr 按 subsystem 组织，不按物料号。

| 任务 | 故障文件 | 覆盖事件 | 测试方案 | 联调笔记 |
|------|----------|----------|----------|----------|
| SENSOR（IMU/姿态/陀螺/I2C） | `Error/sensor_error.md` | E1, E4, E6, E7, E11, E12, E14, E15, E19 | `Test/sensor_test.md` | `sensor_debug.md`（待建） |
| MOTOR（电机/控制环/命令分发） | `Error/motor_error.md` | E8, E9, E13 | `Test/motor_test.md` | `debug/motor_debug.md` |
| SCREEN（淘晶驰串口屏 UART4） | `Error/screen_error.md` | E2 | `Test/screen_test.md` | `screen_debug.md`（待建） |
| LOGGER（控制台/遥测/启动 boot/黑匣子） | `Error/logger_error.md` | E3, E5, E10, E17 | `Test/debug_test.md`（§2.B 引擎机制） | `logger_debug.md`（待建） |
| DIAG（故障诊断 INT8 推理） | `Error/diag_error.md` | E18 | `Test/diag_test.md` | `diag_debug.md`（待建） |
| NET（ESP-01S WiFi/MQTT/阿里云） | `Error/net_error.md`（模板） | 暂无（占位不编造） | `Test/net_test.md` | `net_debug.md`（待建） |
| FLASH（W25Q64 自检/黑匣子） | `Error/flash_error.md`（模板） | 暂无（占位不编造） | `Test/flash_test.md` | `flash_debug.md`（待建） |
| ESP32（ESP32-S3 CAM 图像结论帧） | `Error/esp32_error.md`（模板） | 暂无（占位不编造） | `Test/esp32_test.md` | `esp32_debug.md`（待建） |

> **覆盖说明**：
> - 已记录运行期故障的任务 = SENSOR/MOTOR/SCREEN/LOGGER/DIAG（5 个），对应 `Error/<TASK>_error.md` 已建。
> - **NET/FLASH/ESP32 目前无已记录的运行期故障 → 按用户要求不编造故障条目**，已建 `Error/net_error.md` / `Error/flash_error.md` / `Error/esp32_error.md` 三个**模板占位**（含「暂无记录」声明 + 重点排查方向）；新增事件时直接在对应文件追加「事件 E N」、接全局连续编号即可。
> - 测试方案已覆盖全部 8 任务（`Test/<TASK>_test.md`）；联调笔记仅 MOTOR 已建，其余按需补（模板见 `debug/联调笔记模板.md`）。
> - 注：E16（uvprojx 全空白+177Error）已迁入 `Doc/Keil_MDK_ARM_工程排错记录.md`（问题 4），不在本目录。
