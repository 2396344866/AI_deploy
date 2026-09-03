# ESP32-S3 图像结论接收模块 · 设计说明

> 目的：让后续维护者在不必通读 `Pinout.md` 与 `待处理.md` 的情况下，快速理解
> `Components/BSP/ESP32S3/` 这个模块「做什么、怎么接、协议是什么、踩过哪些坑」。
> 权威细节仍以 `Pinout.md §12.6` 与 `待处理.md §3` 为准，本文是浓缩索引。

---

## 0. 一句话定位

H743 **只收** ESP32-S3 摄像头的「图像结论帧」（FOMO 目标检测结果），**不做任何 DCMI / 图像算力**。
这是方案 B（Pinout §12 定稿）：把图像推理完全下放给 ESP32-S3，H743 侧零图像负担。

⚠️ **与 ESP-01S 是完全不同的两块板，别混**：
| 模块 | 串口 | 波特率 | 角色 |
|------|------|--------|------|
| ESP32-S3 | USART6 (PC6/PC7) | 921600 | 摄像头协处理器 → 发图像**结论帧** |
| ESP-01S | USART2 (PA2/PA3) | 115200 | WiFi → 阿里云 MQTT / OTA，发 **AT 指令** |

两者各自有独立的 BSP 模块、独立的接收回调，仅在 `Components/BSP/uart_rx_dispatcher.c`
里按 `huart` 实例被统一分发（见下文 §5）。

---

## 1. 文件清单

| 文件 | 作用 |
|------|------|
| `Components/BSP/ESP32S3/Inc/esp32s3.h` | 公开接口 + 帧格式宏 + 目标/结果结构体 |
| `Components/BSP/ESP32S3/Src/esp32s3.c` | 接收状态机 + HAL 回调 + 解析任务 + 初始化 |
| `Core/Src/freertos.c` | `MX_FREERTOS_Init` 调 `ESP32S3_BSP_Init()`；命令行 `X` 打印统计 |

> 本模块为 `Components/BSP` 自建子目录，**CubeMX 重生成不会覆盖**。
> 加进工程：Keil 把 `esp32s3.c` 入树 + `Components/BSP/ESP32S3/Inc` 入 Include Paths。

---

## 2. 硬件与 CubeMX 决策

- USART6 引脚 (PC6/PC7)、AF7、IRQ、NVIC（优先级 5）**已由 `.ioc` 生成** → **用户只需物理交叉接线**：
  `PC6 ↔ ESP32 TX`、`PC7 ↔ ESP32 RX`、共地。
- **波特率代码强制 921600**：`.ioc` 可能生成 115200，模块在 `ESP32S3_BSP_Init()` 里
  `huart6.Init.BaudRate = 921600U; HAL_UART_Init(&huart6)` 覆盖，位于自建模块故重生成不丢。
  建议把 `.ioc` 的 USART6 也同步成 921600 作单一数据源。
- **不用 DMA**：`.ioc` 未配 USART6 的 DMA2，故采用 `HAL_UARTEx_ReceiveToIdle_IT`
  （纯中断 + IDLE 切帧）。小帧率 FOMO 结果绰绰有余，且免用户再开 CubeMX。
  （若以后要更高吞吐，可在 CubeMX 给 USART6 配 DMA2，本端改 `HAL_UART_Receive_DMA` 范式。）

---

## 3. 串口协议（Pinout §12.6）

字节布局（大端 CRC，低字节在前）：
```
AA 55 | L(1B) | CMD(1B) | payload(L-1 B) | CRC16_LO | CRC16_HI
```
- **L** = `CMD + payload` 的总字节数（即帧内从 CMD 到 payload 末尾）。
- **CMD=0x01** 检测结果帧：payload = `N(1B) + N×(cls(1B) + cx(2B,0~1000) + cy(2B,0~1000) + conf(1B,0~100))`
  —— 每个目标 7 字节；`cx/cy` 是归一化中心 ×1000。
- **CRC** = CRC-16/MODBUS：多项式 `0x8005`（反射 `0xA001`）、初值 `0xFFFF`、
  输入/输出均反射、`xorout=0`。计算区间 = `CMD + payload`。
  **ESP32 侧固件必须用完全相同的算法，否则 `s_crc_err` 暴涨、帧被丢弃。**

示例帧（1 个目标 cls=0, cx=500, cy=300, conf=80）：
```
AA 55 08 01 01 00 F4 01 2C 50 <CRC_LO> <CRC_HI>
```
（L=0x08 = CMD(1)+payload(7)；payload=01 00F4 012C 50）

---

## 4. 接收状态机（esp32s3.c）

状态：`ST_H1 → ST_H2 → ST_LEN → ST_CMD → ST_PAY → ST_CRC_LO → ST_CRC_HI → 回 ST_H1`
- 帧头容错：`AA AA 55…` 也能对齐（连续两个 AA 时第二个 AA 当 H1 重试）。
- `ST_PAY` 收集 `CMD+`载荷 到 `s_crcbuf`，收满 `L` 字节进入 CRC 校验。
- CRC 正确 → `esp32s3_emit()` 打包成 `esp32s3_result_t` 并入队（ISR 安全 `osMessageQueuePut`）；
  `n>ESP32S3_MAX_OBJ` 截断，避免越界。
- CRC 失败 → `s_crc_err++`（计数，不阻塞）。

---

## 5. ISR 铁律（error.md E2）

`ESP32S3_UART_RxCallback()`（由 `uart_rx_dispatcher.c` 的 `HAL_UARTEx_RxEventCallback` 转发）在
**USART6 ISR 上下文**运行，**只允许**：
1. 把收到的字节喂进状态机；
2. 重启 `HAL_UARTEx_ReceiveToIdle_IT`；
3. `osThreadFlagsSet(s_esp_task, ESP32S3_FLAG_RX)` 唤醒解析任务。

**绝不**在 ISR 里调用 `LOG_I` / 任何阻塞发送。所有日志与计数打印都在任务上下文。

> **为什么有 dispatcher？** `HAL_UARTEx_RxEventCallback` 是全局唯一弱符号，
> ESP32-S3(USART6) 与 ESP-01S(USART2) 都用 `ReceiveToIdle_IT`，若各模块都定义会链接冲突。
> 故 `Components/BSP/uart_rx_dispatcher.c` 集中定义，按 `huart==&huart6 / &huart2` 分发。

---

## 6. 任务与队列模型

- `ESP32S3_Task`（优先级 `osPriorityNormal`，栈 640）是 `Queue_ImgResultHandle` 的**唯一消费者**：
  取出帧 → 写 `s_latest` 快照 → 打 `LOG_I("ESP32", "DET n=..")`（受 `DBG_LOG_ESP32` 控制）→ 统计 CRC 错增量。
- 其它任务（故障诊断 / 控制）用 `ESP32S3_GetLatest(&res)` 读最近一帧快照，不碰队列。
- 公开 API：`ESP32S3_BSP_Init()` / `ESP32S3_GetLatest()` / `ESP32S3_PrintStats()`。

---

## 7. 调试（统一走 UART1 到 PC）

- 开关：`dbg_config.h` 的 `DBG_LOG_ESP32`（默认 0，发布态零开销）。置 1 重编即开帧日志。
- 串口助手发 **`X`**（freertos.c 命令分发）→ `ESP32S3_PrintStats()` 打印
  `f_ok / crc_err / ovf / last_n` 到 PC。
- 完全并入 `Components/Debug` 既有 `LOG_I` + per-task 开关体系，**没有新建独立调试通道**。

---

## 8. 对接 / 联调清单

1. Keil：加 `esp32s3.c` + Include Path；`DBG_LOG_ESP32=1` 开调试。
2. 物理接线：PC6↔ESP32 TX、PC7↔ESP32 RX、共地。
3. ESP32 固件侧按 §3 帧格式上报（CRC-16/MODBUS 必须一致）。
4. 烧录后双方串口对通，VOFA / 串口助手看 `DET n=..`；`X` 看统计。
5. CRC 一直错 → 先核对 ESP32 侧 CRC 算法与计算区间（CMD+payload，非整帧）。
