

---

# BSP 驱动层说明

> 本文档整合了串口（USART/UART）、Flash（W25Q64）及 BSP 目录规范的说明，供二次开发与维护参考。

---

## 目录

1. [BSP 目录规范](#1-bsp-目录规范)
2. [串口驱动（USART1 / UART4）](#2-串口驱动usart1--uart4)
3. [W25Q64 Flash 驱动](#3-w25q64-flash-驱动)
4. [API 速查](#4-api-速查)
5. [关键约束与注意事项](#5-关键约束与注意事项)
6. [根因记录（供复盘）](#6-根因记录供复盘)

---

## 1. BSP 目录规范

### 1.1 目录布局

```
BSP/
├── LED/                     # 板载 LED 指示灯驱动（RED/GREEN/BLUE 三色状态灯）
│   ├── Inc/led.h               # LED_R/G/B 电平写 + 翻转宏（无初始化，引脚 CubeMX 配）
│   └── Src/led.c
├── LOG/                     # 日志串口驱动（UART4 打印 + USART1 DMA 空闲收帧；原 BSP_USART）
│   ├── Inc/BSP_LOG.h
│   └── Src/BSP_LOG.c
├── W25Q64/                  # SPI Flash 驱动（崩溃黑匣子存储）
│   ├── Inc/BSP_W25Q64.h
│   └── Src/BSP_W25Q64.c
├── IMU/                     # IMU 子系统（算法层，无 BSP_ 前缀）
│   ├── Inc/
│   │   ├── attitude.h          # 姿态解算 + 外环 PID 控制器
│   │   ├── imu_mpu6050.h       # MPU6050 I2C 底层
│   │   ├── imu_filter.h        # 输入滤波（去极值均值 / 一阶滞后）
│   │   ├── madgwick.h          # Madgwick AHRS 融合（C 移植）
│   │   └── vofa_telemetry.h    # VOFA+ firewater 遥测
│   └── Src/                # 对应 .c
├── ESP/                     # 无线模块聚合（ESP-01S + ESP32-S3 共用 HAL_UARTEx_RxEventCallback 分发）
│   ├── Inc/
│   │   ├── esp01s.h           # ESP-01S AT / 阿里云 MQTT 上云（USART2, 115200）
│   │   └── esp32s3.h          # ESP32-S3 图像结论帧接收（USART6, 921600）
│   ├── Src/
│   │   ├── esp01s.c
│   │   ├── esp32s3.c
│   │   └── uart_rx_dispatcher.c  # 唯一 HAL_UARTEx_RxEventCallback 强定义，按 huart 分发
│   └── uart_rx_routing.svg   # UART RX 路由图（ESP01S/ESP32-S3/LOG 三路如何分流，供 AI/维护参考）
└── README.md
```

> - `BSP/<Module>/Inc|Src`：板级外设/功能模块（LED/LOG/W25Q64/ESP），命名 `BSP_<Periph>` 仅当裸名与 CubeMX 生成头（gpio/usart/spi/i2c/tim）撞名时才加。
> - `BSP/IMU`：算法子系统（驱动+融合+控制），命名 `<module>`（无 `BSP_` 前缀）。
> - `BSP/ESP`：无线模块聚合，子文件按器件（esp01s/esp32s3）+ 共享分发器（uart_rx_dispatcher.c）组织； Inc 放对应 .h。

### 1.2 命名约定

| 类别       | 命名                                        | 示例                            |
| -------- | ----------------------------------------- | ----------------------------- |
| 板级外设头/源  | `BSP_<Periph>.h/.c`                       | `BSP_W25Q64.h`                |
| IMU/算法模块 | `<module>.h/.c`                           | `attitude.h`、`madgwick.c`     |
| 头文件保护宏   | `_BSP_<PERIPH>_H` / `_<MODULE>_H`         | `_BSP_W25Q64_H`、`_ATTITUDE_H` |
| 对外函数     | `BSP_*` / `Attitude_*` / `Madgwick_*` / 等 | 见各头文件                         |

### 1.3 头文件编写规范

1. **保护宏**：`#ifndef _XXX_H` / `#define _XXX_H` / `#endif /* _XXX_H */`，大写、单个尾部下划线。
2. **段落文档注释**：头顶部用 `/* ===... === */` 写清「职责 + 依赖 + 二次开发提示」。
3. **依赖最小化**：尽量只 `#include <stdint.h>`；确实需要 HAL 句柄时再 `#include "main.h"`。
4. **extern 句柄**：外设句柄（如 `hi2c1`）在 CubeMX 生成文件中定义，本层用 `extern` 引用，不把定义写进 BSP。

---

## 2. 串口驱动（USART1 / UART4）

### 2.1 当前状态（2026-08-11）

- **USART1** 已升级为 **CubeMX 托管 DMA + 空闲中断**（RX 方向）。
  - DMA 句柄 `hdma_usart1_rx`（DMA1_Stream4, Circular, Low）由 CubeMX 在 `usart.c` 生成并 `__HAL_LINKDMA`。
  - ISR `DMA1_Stream4_IRQHandler` 由 CubeMX 在 `stm32h7xx_it.c` 生成。
  - 应用层只在 USER CODE 块里写，重新 Generate Code **不丢、不重名**。
- **UART4** 的 DMA + 空闲中断保持 CubeMX 管理不变，两个串口写法现已完全一致。

### 2.2 推荐方案：UART + DMA + 空闲中断

- **接收（RX）**：走 **DMA + 空闲线（IDLE）中断**，而非每字节进一次 `HAL_UART_Receive_IT`。
  - DMA 接管整帧搬运，CPU 只在“总线空闲（一帧结束）”时进一次中断取数据。
  - 每字节中断在高速 / 长帧时会严重占 CPU，且容易丢数据。
- **发送（TX）**：调试/日志场景用轮询即可（`fputc` / `log_backend_putc` 在低优先级任务里刷）；需要高吞吐 TX 再上 DMA TX。

### 2.3 工程中的两个串口

| 串口         | 引脚               | 当前用途                | RX 方案             |
| ---------- | ---------------- | ------------------- | ----------------- |
| **USART1** | PA9(TX)/PA10(RX) | 日志控制台（TX）+ 命令接收（RX） | CubeMX DMA + 空闲中断 |
| **UART4**  | PA0(TX)/PA1(RX)  | 淘晶驰串口屏              | CubeMX DMA + 空闲中断 |

### 2.4 空闲线接收原理

1. 启动 `HAL_UART_Receive_DMA(buf, size)`，DMA 持续把 RX 数据搬进 `buf`（循环模式）。
2. 一帧发完，发送方停止发，总线出现**空闲（IDLE）**。
3. `USARTx_IRQHandler` 检测到 `UART_FLAG_IDLE`：
   - 计算已收长度 = `size - __HAL_DMA_GET_COUNTER(huartx.hdmarx)`；
   - `HAL_UART_DMAStop()` 暂停 DMA；
   - 把这一帧交给业务回调（解析 / 入队）；
   - 再次 `HAL_UART_Receive_DMA()` 重启接收。
4. 如此循环，CPU 仅在帧边界被打断一次。

> **关键易错点**：`hdmarx` 是 `DMA_HandleTypeDef *` 指针，调用宏时**不要加 `&`**。  
> 正确：`__HAL_DMA_GET_COUNTER(huart1.hdmarx)`；错误：`&huart1.hdmarx`（二级指针，触发大量类型 error）。

### 2.5 USART1 实现细节

#### CubeMX 配置（已做）

- Request：`USART1_RX`；Stream：`DMA1_Stream4`
- Direction：`Peripheral To Memory`；Mode：`Circular`；Priority：`Low`；Data Width：`Byte`
- 自动生成：`hdma_usart1_rx` 句柄、`__HAL_LINKDMA`、NVIC、`DMA1_Stream4_IRQHandler()`

#### 应用层代码

**`BSP_LOG.h`**

```c
#define USART1_RX_BUF_SIZE 128
void BSP_LOG_UART1_RxStart(void);
void BSP_LOG_UART1_OnRxEvent(uint16_t size);   /* HAL_UARTEx_RxEventCallback(USART1) 入口 */
void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len);
```

**`BSP_LOG.c`**

```c
static uint8_t s_uart1_rx[USART1_RX_BUF_SIZE];

void BSP_LOG_UART1_RxStart(void)
{
    /* 控制台走 IT 范式：先清 CubeMX 配的 DMA 残留状态，再启动 IT 收帧。
       USART1 为 115200 调试控制台，无需 DMA；IT 由 CPU 同核写 RAM，无 D-Cache 一致性问题。 */
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_DMAStop(&huart1);
    if (huart1.hdmarx != NULL) {
        __HAL_DMA_DISABLE(huart1.hdmarx);
        huart1.hdmarx = NULL;            /* 与 CubeMX DMA 解耦，USART1 RX 完全走 IT */
    }
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);
}

/* HAL_UARTEx_RxEventCallback(USART1) 入口；size = 本次 IDLE 前收到的字节数，∈[0,BUF_SIZE] */
void BSP_LOG_UART1_OnRxEvent(uint16_t size)
{
    if (size > 0U && size <= (uint16_t)USART1_RX_BUF_SIZE) {
        /* IT 模式：数据已由 HAL ISR 线性写入 s_uart1_rx[0,size)，无 D-Cache 问题。
           先拷出线性帧再重武装（重武装复位 pRxBuffPtr，避免与 OnFrame 复用缓冲冲突）。 */
        static uint8_t frame[USART1_RX_BUF_SIZE];
        for (uint16_t i = 0U; i < size; i++) frame[i] = s_uart1_rx[i];
        BSP_LOG_UART1_OnFrame(frame, size);   /* 强定义在 freertos.c，推 g_cmd_qHandle */
    }
    /* 重武装：ToIdle 收完 UART 回 READY、HAL 关闭 IDLE IE，必须重启用才能收下一帧 */
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);
}

__weak void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 0xFFFF);
}
```

**`main.c`（USER CODE BEGIN 2）**

```c
BSP_LOG_UART1_RxStart();
```

**`stm32h7xx_it.c`（USART1_IRQHandler USER CODE 0 块留空，由生成的 `HAL_UART_IRQHandler(&huart1)` 驱动）**

```c
/* USER CODE BEGIN USART1_IRQn 0 */
/* (空：HAL_UART_IRQHandler 内部处理 IDLE -> 调 HAL_UARTEx_RxEventCallback) */
/* USER CODE END USART1_IRQn 0 */
HAL_UART_IRQHandler(&huart1);
/* RxEventCallback 在 uart_rx_dispatcher.c 按 huart 分发：&huart1 -> BSP_LOG_UART1_OnRxEvent */
```

### 2.6 接收业务处理

在自己的文件里**重新定义** `BSP_LOG_UART1_OnFrame()`（不要加 `__weak`），例如：

```c
void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len) {
    if (len >= 2 && data[0] == 'L' && data[1] == '?') logger_set_level(LOG_LVL_DEBUG);
    /* 或 xQueueSend(rx_queue, data, ...) 投给任务处理 */
}
```

---

## 3. W25Q64 Flash 驱动

### 3.1 硬件事实

- 模块引脚：**VCC / GND / DO / CLK / DI / CS**（6 脚标准 SPI）
- 型号：**W25Q64BV**（无 IO2/IO3，Quad/Octal SPI 不可用）
- 当前走 **标准 SPI + DMA**

### 3.2 SPI 时钟

- SPI1/2/3 内核时钟 = 160 MHz
- 当前 `BaudRatePrescaler = _4` → 有效 SCK = **40 MHz**
- 启动时打印：`Effective SPI1 SCK = 40000 kHz`

### 3.3 传输策略：混合 SPI（命令/地址轮询 + 数据 DMA）

1. 拉低 CS。
2. 轮询发送指令 + 24 位地址。
3. 数据段用 `HAL_SPI_Transmit_DMA` / `HAL_SPI_Receive_DMA` 批量收发。
4. 等待 DMA 完成信号量 `g_semFlashDmaDoneHandle`。
5. 拉高 CS。

> 命令/地址不用 DMA 是因为长度短且变长，DMA 配置开销大于收益。

### 3.4 D-Cache 一致性（H7 关键）

| 方向          | 操作                       | 函数                               |
| ----------- | ------------------------ | -------------------------------- |
| TX（写 Flash） | DMA 启动前把 CPU 写的脏数据刷回 RAM | `SCB_CleanDCache_by_Addr()`      |
| RX（读 Flash） | DMA 启动前丢弃 CPU 缓存中的旧副本    | `SCB_InvalidateDCache_by_Addr()` |

缓冲要求：

- `__ALIGNED(32)`：Cache 行对齐（H7 L1 cache line = 32 字节）
- 放在 AXI SRAM / D2 SRAM（DTCM 无法被 DMA 访问）
- 本项目 TX/RX 缓冲：`g_flashTxBuf` / `g_flashRxBuf`，大小 256 字节

### 3.5 RTOS 同步

- 信号量 `g_semFlashDmaDoneHandle`（二值信号量，初始 0）
- DMA 传输完成中断链：`DMA TC → SPI EOT → HAL_SPI_Tx/RxCpltCallback → osSemaphoreRelease()`
- 任务里 `osSemaphoreAcquire(g_semFlashDmaDoneHandle, osWaitForever)` 阻塞等待
- **重要**：所有 Flash DMA 操作必须在 FreeRTOS 调度器启动后的任务上下文中执行

### 3.6 API

```c
#include "BSP_W25Q64.h"

uint32_t W25QXX_ReadJedecID(void);                       // 读 JEDEC ID，W25Q64BV = 0xEF4017
void     W25QXX_SectorErase(uint32_t SectorAddr);        // 擦除 4KB 扇区，地址须 4KB 对齐
void     W25QXX_BulkErase(void);                         // 整片擦除
void     W25QXX_PageWrite(uint8_t *pTxBuffer,
                          uint32_t WriteAddr,
                          uint16_t NumByteToWrite);      // 页写，最大 256 字节，不可跨页
void     W25QXX_BufferRead(uint8_t *pRxBuffer,
                           uint32_t ReadAddr,
                           uint16_t NumByteToRead);      // 缓冲读
int      w25q_crashlog_save(const uint8_t *data,
                            uint32_t len);              // 崩溃日志落盘（轮询，HardFault 安全）
```

### 3.7 崩溃黑匣子

- 地址：`0x7FF000`（W25Q64 末扇区，8MB 最后 4KB）
- 布局：
  - 字节 0–3：magic `0x43424144`（即 `"CRSH"`）
  - 字节 4–7：日志长度 `len`（小端）
  - 字节 256 起：日志体，按页（256B）轮询写入
- 实现：`w25q_crashlog_save()` 完全轮询，不调用 RTOS/DMA
- 调用点：`HardFault_Handler` 中调用 `logger_flush_to_flash()`

---

## 4. API 速查

### 串口

```c
void UART4_Printf(const char *fmt, ...);                 /* UART4 格式化发送（轮询） */
void BSP_LOG_UART1_RxStart(void);                            /* 启动 USART1 的 DMA 循环接收 + 空闲中断 */
void BSP_LOG_UART1_OnRxEvent(uint16_t size);                 /* USART1 收帧入口（HAL_UARTEx_RxEventCallback 分发） */
void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len); /* 收帧回调（弱符号，业务层覆盖） */
```

### Flash

```c
uint32_t W25QXX_ReadJedecID(void);
void     W25QXX_SectorErase(uint32_t SectorAddr);
void     W25QXX_BulkErase(void);
void     W25QXX_PageWrite(uint8_t *pTxBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void     W25QXX_BufferRead(uint8_t *pRxBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
int      w25q_crashlog_save(const uint8_t *data, uint32_t len);
```

---

## 5. 关键约束与注意事项

### 串口

| 项目      | 约束                                                                 |
| ------- | ------------------------------------------------------------------ |
| DMA 流分配 | DMA1 Stream0/1 = UART4，Stream2/3 = SPI1，Stream4 = USART1_RX（勿冲突）   |
| 接收缓冲区   | `RX4_BUFFER_SIZE` / `USART1_RX_BUF_SIZE` 默认 128；超长帧会被拆成多帧，需业务层做帧拼接 |
| 空闲中断    | 必须在启动 DMA **之前** `__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE)`，否则不触发 |
| 异常上下文   | HardFault 等异常上下文不要依赖 DMA 接收；日志落盘用的是轮询写 Flash                       |

### Flash

| 项目    | 约束                                           |
| ----- | -------------------------------------------- |
| 页大小   | 256 字节，`W25QXX_PageWrite` 单次最大 256B，跨页需调用方拆分 |
| 扇区大小  | 4096 字节，写之前必须擦除                              |
| 擦除寿命  | 典型 10 万次；崩溃日志每次擦写末扇区，非高频场景可接受                |
| CS 时序 | DMA 完成后、拉高 CS 之前必须确认 EOT，否则数据截断              |
| 信号量   | Flash DMA 操作**必须**在任务上下文中执行，不能在中断中调用         |

---

## 6. 根因记录（供复盘）

### 废弃手写 DMA 句柄的原因

- **报错行**：`__HAL_DMA_GET_COUNTER(&huart1.hdmarx)`
- **根因**：`huart1.hdmarx` 类型是 `DMA_HandleTypeDef *`（指针），`&huart1.hdmarx` 变成**二级指针**；宏内部 `IS_DMA_STREAM_INSTANCE((__HANDLE__)->Instance)` 因类型不符全部展开失败 → 18 个 error。
- **正确写法**：`__HAL_DMA_GET_COUNTER(huart1.hdmarx)`
- **教训**：手写 DMA 句柄容易踩类型坑；**让 CubeMX 生成 DMA 基础设施最稳**，用户只写应用层 + USER CODE。

---

*本文档最后更新：2026-08-22*
