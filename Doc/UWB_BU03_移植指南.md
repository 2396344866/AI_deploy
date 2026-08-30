# UWB（安信可 BU03 / DW3000）移植指南

> 适用：把 RCT6（STM32F103RCT6）井下轨道运输车的 UWB 定位子系统，移植到 AI_deploy（STM32H743）磨矿给料机器人。
> 状态：本文件为**设计 / 移植说明**。AI_deploy 当前仓内 UWB 代码待按本指南落地；简历或 README 中若提及 UWB，请以本指南落地后的状态为准。

## 1. 模块与原理

- 安信可 **BU03** 是基于 **DW3000** 的 UWB 模组，板载 RF / 电源管理 / 协处理器，主机通过 **UART-AT** 交互（模块内置 AT 固件）。
- 测距方式：**TWR（双向测距）**，由模块内部完成；主机仅解析距离结果帧（如 `DIST:x.xxx`，单位米）。
- 定位：**3 个固定基站（anchor）+ 1 个车载标签（tag）**，主机取得 3 个距离后，用 **Gauss-Newton 最小二乘** 解算二维坐标（见 `uwb_2d.c`）。

## 2. 待移植的源文件（来自 RCT6）

| 文件 | 作用 | 移植工作量 |
|---|---|---|
| `bsp_uwb.h / bsp_uwb.c` | BU03 UART-AT 驱动（Init / StartRanging / ReadDistance） | 低：替换底层 USART 接口 |
| `uwb_2d.h / uwb_2d.c` | 二维三边定位（Gauss-Newton） | 极低：纯 C + `math.h`，几乎原样搬 |
| `uwb_task.c` | FreeRTOS 定位任务（周期测距 → 解算 → 写状态） | 中：接入 H743 系统状态与任务框架 |

RCT6 侧关键 API（移植时对齐）：

```c
void BSP_UWB_Init(void);
int  UWB_StartRanging(void);          /* 发 AT+KW=RANGE,1 启动测距 */
int  UWB_ReadDistance(float* dist_m); /* 解析 "DIST:x.xxx" 文本 */
int  UWB_Trilaterate2D(const Point2D* anchors, const float* d, Point2D* out);
```

## 3. 移植步骤（H743 + FreeRTOS）

1. **UART 分配**：在 CubeMX 选一个空闲 UART（如 USART2 / UART4），波特率与 BU03 固件一致（RCT6 用 `UWB_USART_BAUD`）。生成后对接 AI_deploy 已有的 `BSP_LOG`（UART + DMA + 空闲中断接收）。
2. **bsp_uwb 适配**：把 `BSP_LOG_Init / SendString / Read` 换成 AI_deploy 的 `BSP_LOG` API；去掉 `#include "stm32f10x.h"`，改为工程公共头。
3. **uwb_2d 直搬**：`uwb_2d.c/.h` 不依赖 MCU 外设，仅 `#include <math.h>`；新建 `Components/UWB/Inc|Src` 放入即可，无需改写算法。
4. **uwb_task 接入**：
   - 新建 `Components/UWB/Src/uwb_task.c`，周期（如 `UWB_PERIOD_MS = 200`）向 3 基站发起测距；
   - 基站坐标 `s_anchors[3]` 按现场标定填入（RCT6 示例：`(0,0) / (6,0) / (3,5)` 米）；
   - 解算结果写入 H743 系统状态（如 `g_state.position`），与电机 / AI 诊断任务共享；
   - 在 `freertos.c` 中用 `osThreadNew` 启动该任务，并保留 `Health_Report` 看门狗上报。
5. **宏开关**：在 `board_config.h` 增加 `UWB_USART`、`USE_UWB_SUBSYS`，保持与 RCT6 一致的编译期开关语义。

## 4. F103 → H743 差异注意

- USART 走 HAL / CubeMX 配置，但 AT 指令文本协议不变；
- FreeRTOS 版本（CMSIS-RTOS **v2**）的任务创建 API 与 RCT6（v1）略有差异，按 AI_deploy 现有任务框架接入；
- 三边定位数学完全一致，无需重写。

## 5. 验证清单

- [ ] BU03 上电，串口能收到 `DIST:...` 帧；
- [ ] 单基站测距值与卷尺实测偏差 < 10 cm；
- [ ] 三基站同时测距，解算坐标与已知标签位置吻合；
- [ ] 定位结果经 logger 写入 W25Q64 黑匣子，掉电可查。

## 6. 与磨矿给料机器人的关系

UWB 提供**绝对位置**（校正编码器里程计长时漂移），与 MPU6050 姿态、编码器里程计经 EKF / 互补滤波融合，是给料机器人"到达指定给料点"的定位基础。磨矿车间金属多、粉尘振动，UWB 多径严重，锚点布置与 NLOS 处理需在现场标定。
