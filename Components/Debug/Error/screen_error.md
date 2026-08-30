# SCREEN 任务运行期故障归档（串口屏 UART4）

> 本文件是 **SCREEN 任务**（StartScreenTask + 淘晶驰串口屏 UART4）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/error.md` 一致）。
> 编译期故障归 `Doc/Keil_MDK_ARM_工程排错记录.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

## 事件 E2：上电只打印 System Init Success! 后再无输出（2026-08-19）

### 现象
- 上电后仅打印 `System Init Success!`，`osKernelStart()` 之后任务**完全不运行**，无任何后续日志/波形。
- 与 E1 不同：E1 是"跑一阵后卡死"，E2 是"一开始就起不来"，属独立故障。

### 调试器定位（关键）
- 暂停后 PC 长期停留在 **`UART4_IRQHandler`** 内（`Core/Src/stm32h7xx_it.c`）。
- 根因：该 ISR 在 IDLE 中断里调用 `UART4_Printf()` → 内部 `HAL_UART_Transmit()` **轮询发送**，
  在中断上下文中长时间等待 TXE/TC；串口屏上电持续吐数据 → UART4 ISR 被反复触发、抢占一切
  （含 SysTick 与所有任务）→ **调度器被饿死，任务永不调度**。

### 根因链
1. 淘晶驰屏上电即灌数据 → UART4 收到帧 → 进 `UART4_IRQHandler`。
2. ISR 内 `UART4_Printf("Received %d bytes")` 做轮询 TX（屏高频发时几乎常驻 ISR，最长约 1ms/次）。
3. ISR 优先级(6) 高于 SysTick(15)/任务 → 任务上下文得不到 CPU → 表现为"System Init Success! 后死寂"。

### 修复方案
- **治本（代码，`stm32h7xx_it.c` USER CODE 块）**：删除 ISR 内 `UART4_Printf`，
  改为 `osSemaphoreRelease(g_semScreenUpdateHandle)` 通知 `Task_Screen` 在任务上下文处理 `rx_buf`；
  ISR 内**绝不**做阻塞式发送。
- 配套：`USART1_IRQn` 优先级 8→6，使命令 RX（IDLE→`osMessageQueuePut`）响应更快、与 DMA 通信中断同档。
- 验证：上电后 UART4 即使被屏持续灌数据，任务也应正常起转、控制台/波形恢复。

### 状态
- [x] 根因定位（UART4 ISR 轮询发送饿死调度器）
- [x] 删除 ISR 内 `UART4_Printf`，改为信号量通知（本次实现）
- [x] `USART1_IRQn` 优先级 8→6（本次实现）
- [ ] 用户 Keil 重编 0 错误、烧录复测：System Init Success! 后任务正常输出
