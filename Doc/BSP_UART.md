# BSP_UART 串口驱动说明

## 0. 当前状态（2026-08-11）
- **USART1 已升级为 CubeMX 托管 DMA + 空闲中断**（RX 方向）：DMA 句柄 `hdma_usart1_rx`（DMA1_Stream4, Circular, Low）由 CubeMX 在 `usart.c` 生成并 `__HAL_LINKDMA`，ISR `DMA1_Stream4_IRQHandler` 由 CubeMX 在 `stm32h7xx_it.c` 生成；应用层只在 USER CODE 块里写。重新 Generate Code **不丢、不重名**。
- 早期一次"手写 DMA 句柄"因 `__HAL_DMA_GET_COUNTER(&huart1.hdmarx)` 多写 `&`（二级指针）触发 18 个 error，已废弃该手写方案，改为 CubeMX 托管（见 §8 复盘）。
- UART4 的 DMA + 空闲中断保持 CubeMX 管理不变（见 §5），两个串口写法现已完全一致。

## 1. 结论：串口 RX 应该用 UART + DMA + 空闲中断
- **推荐**：接收（RX）走 **DMA + 空闲线（IDLE）中断**，而非每字节进一次 `HAL_UART_Receive_IT`。
- 原因：DMA 接管整帧搬运，CPU 只在“总线空闲（一帧结束）”时进一次中断取数据；每字节中断在高速 / 长帧时会严重占 CPU，且容易丢数据。
- 发送（TX）：调试 / 日志场景用轮询即可（`fputc` / `log_backend_putc` 在 `StartLoggerTask` 低优先级任务里刷）；需要高吞吐 TX 再上 DMA TX。

## 2. 工程里两个串口
| 串口 | 引脚 | 当前用途 | RX 方案 |
|---|---|---|---|
| **USART1** | PA9(TX)/PA10(RX) | 日志控制台（TX）+ 命令接收（RX） | ✅ CubeMX DMA + 空闲中断（见 §4） |
| **UART4** | PA0(TX)/PA1(RX) | 通用串口 | ✅ CubeMX DMA + 空闲中断（见 §5） |

## 3. 空闲线接收原理（经典套路）
1. 启动 `HAL_UART_Receive_DMA(buf, size)`，DMA 持续把 RX 数据搬进 `buf`（循环模式）。
2. 一帧发完，发送方停止发，总线出现**空闲（IDLE）**。
3. `USARTx_IRQHandler` 检测到 `UART_FLAG_IDLE`：
   - 计算已收长度 = `size - __HAL_DMA_GET_COUNTER(huartx.hdmarx)`；
   - `HAL_UART_DMAStop()` 暂停 DMA；
   - 把这一帧交给业务回调（解析 / 入队）；
   - 再次 `HAL_UART_Receive_DMA()` 重启接收。
4. 如此循环，CPU 仅在帧边界被打断一次。

> ⚠️ **关键易错点**：`hdmarx` 是 `DMA_HandleTypeDef *` 指针，调用宏时**不要加 `&`**。
> 正确：`__HAL_DMA_GET_COUNTER(huart1.hdmarx)`；错误：`&huart1.hdmarx`（二级指针，会触发大量类型 error）。

## 4. USART1 实现（CubeMX 托管 DMA + 空闲中断，已完成）
> 原则：DMA 句柄 / MSP / 流中断统一交给 CubeMX 生成，用户代码只写“应用层 + USER CODE 块”。

### 4.1 CubeMX 配置（已做）
打开 `STM32H743VIT6.ioc` → USART1 → DMA Settings → Add：
- Request：`USART1_RX`；Stream：`DMA1_Stream4`（与 UART4 的 Stream0/1、SPI1 的 Stream2/3 错开）
- Direction：`Peripheral To Memory`；Mode：`Circular`；Priority：`Low`；Data Width：`Byte`
- Generate Code 后自动生成：`hdma_usart1_rx` 句柄、`usart.c` 的 `__HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx)` + NVIC、`stm32h7xx_it.c` 的 `DMA1_Stream4_IRQHandler()`。

### 4.2 应用层代码（全部在 USER CODE 块，regen 安全）
- `Components/BSP/BSP_USART.h`：
  ```c
  #define USART1_RX_BUF_SIZE 128
  void BSP_UART1_RxStart(void);
  void BSP_UART1_IdleHandler(void);
  void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len);
  ```
- `Components/BSP/BSP_USART.c`：
  ```c
  static uint8_t s_uart1_rx[USART1_RX_BUF_SIZE];

  void BSP_UART1_RxStart(void)
  {
      __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   /* 必须在启动 DMA 之前使能 */
      HAL_UART_Receive_DMA(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);
  }

  void BSP_UART1_IdleHandler(void)
  {
      if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
      {
          __HAL_UART_CLEAR_IDLEFLAG(&huart1);
          uint16_t rx_len = USART1_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx); /* 不加 & */
          HAL_UART_DMAStop(&huart1);
          if (rx_len > 0U) BSP_UART1_OnFrame(s_uart1_rx, rx_len);
          HAL_UART_Receive_DMA(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);  /* 重启 */
      }
  }

  __weak void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len)
  {
      HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 0xFFFF);  /* 默认回显，便于验证 */
  }
  ```
- `Core/Src/main.c` `USER CODE BEGIN 2`（在 `MX_USART1_UART_Init()` 后）：
  ```c
  BSP_UART1_RxStart();   /* 取代旧的 HAL_UART_Receive_IT(&huart1, &rx1_buf, 1) */
  ```
- `Core/Src/stm32h7xx_it.c` `USART1_IRQHandler` 的 `USER CODE BEGIN USART1_IRQn 0`：
  ```c
  BSP_UART1_IdleHandler();   /* 取一帧交给回调，再重启 DMA */
  ```

## 5. UART4 实现 —— CubeMX 已配置（参考范本）
- `.ioc` 里 UART4 已开 DMA：`Dma.UART4_RX.0`（DMA1_Stream0，CIRCULAR）/ `Dma.UART4_TX.1`（DMA1_Stream1，NORMAL）。
- `Core/Src/usart.c` 的 `HAL_UART_MspInit()` 已生成 `hdma_uart4_rx/tx` 初始化 + `__HAL_LINKDMA`。
- `Core/Src/main.c` `USER CODE BEGIN 2`：`HAL_UART_Receive_DMA(&huart4, rx_buf, RX4_BUFFER_SIZE)` + `__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE)`。
- `Core/Src/stm32h7xx_it.c` `UART4_IRQHandler` 的 `USER CODE`：检测 IDLE → 取长度 → 处理 → 重启 DMA。
- **USART1 按 §4 配好后，两个串口写法完全一致，便于维护。**

## 6. API
```c
void UART4_Printf(const char *fmt, ...);                 /* UART4 格式化发送（轮询） */
void BSP_UART1_RxStart(void);                            /* 启动 USART1 的 DMA 循环接收 + 空闲中断 */
void BSP_UART1_IdleHandler(void);                        /* USART1 空闲中断处理（供 ISR 调用） */
void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len); /* 收帧回调（弱符号，业务层覆盖） */
```
接收业务处理：在自己的文件里 **重新定义** `BSP_UART1_OnFrame()`（不要加 `__weak`），即可在不改 BSP 的前提下解析命令，例如：
```c
void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len) {
    if (len >= 2 && data[0] == 'L' && data[1] == '?') logger_set_level(LOG_LVL_DEBUG);
    /* 或 xQueueSend(rx_queue, data, ...) 投给任务处理 */
}
```

## 7. 注意事项
- DMA 流分配：DMA1 Stream0/1 = UART4，Stream2/3 = SPI1，Stream4 = USART1_RX（本项目约定，勿冲突）。
- `RX4_BUFFER_SIZE` / `USART1_RX_BUF_SIZE` 默认 128；超长帧会被拆成多帧（DMA 循环 + 空闲），回调需自行做帧拼接 / 分隔。
- 空闲中断必须在启动 DMA **之前** `__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE)`，否则不触发。
- HardFault 等异常上下文不要依赖 DMA 接收；日志落盘用的是轮询写 Flash（`w25q_crashlog_save`）。

## 8. 根因记录（废弃手写 DMA 的原因，供复盘）
- 报错行：`__HAL_DMA_GET_COUNTER(&huart1.hdmarx)`。
- `huart1.hdmarx` 类型是 `DMA_HandleTypeDef *`（指针），`&huart1.hdmarx` 变成**二级指针**；宏 `__HAL_DMA_GET_COUNTER` 内部 `IS_DMA_STREAM_INSTANCE((__HANDLE__)->Instance)` 因类型不符全部展开失败 → 18 个 error（同源，非 18 个独立问题）。
- 正确写法：`__HAL_DMA_GET_COUNTER(huart1.hdmarx)`（UART4 的正确代码即如此）。
- 教训：手写 DMA 句柄容易踩这类类型坑；**让 CubeMX 生成 DMA 基础设施最稳**，用户只写应用层 + USER CODE。
