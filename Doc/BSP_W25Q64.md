# W25Q64 Flash 驱动说明

## 1. 硬件事实（已确认）
- 模块引脚：**VCC / GND / DO / CLK / DI / CS**（6 脚标准 SPI）。
- Flash 型号：**W25Q64BV**。
- 结果：
  - **没有 IO2/IO3**，Quad/Octal SPI 不可用。
  - XIP 需要把 4 根线改接到 H743 的 QUADSPI 外设（硬件改线，当前不做）。
  - 当前走 **标准 SPI + DMA**。

## 2. SPI 时钟
- SPI1/2/3 内核时钟来自时钟树：`PLL1Q / 2 = 160 MHz`。
- 当前 `BaudRatePrescaler = _4` → 有效 SCK = **40 MHz**。
- `W25QXX_Test()` 启动时会打印：
  ```
  [n][I][FLASH] Effective SPI1 SCK = 40000 kHz (SPI123CLK 160 MHz / 4)
  ```
- 已验证：40 MHz 下读回数据与写入一致，D-Cache 一致性处理正确。

## 3. 传输策略：混合 SPI（命令/地址轮询 + 数据 DMA）
这是 SPI NOR Flash 的标准做法：
1. 拉低 CS。
2. 用轮询发送指令（如 `0x02` 页写、`0x03` 读数据）+ 24 位地址。
3. 数据段用 `HAL_SPI_Transmit_DMA` / `HAL_SPI_Receive_DMA` 批量收发。
4. 等待 DMA 完成信号量 `g_semFlashDmaDoneHandle`。
5. 拉高 CS。

为什么命令/地址不用 DMA：
- 长度短且变长，DMA 配置开销大于收益。
- 必须与数据保持同一 CS 窗口，轮询更直观可控。

## 4. D-Cache 一致性（H7 关键）
STM32H7 带 D-Cache，DMA 与 CPU 看到的数据可能不一致。

| 方向 | 操作 | 函数 |
|---|---|---|
| TX（写 Flash） | DMA 启动前把 CPU 写的脏数据刷回 RAM | `SCB_CleanDCache_by_Addr()` |
| RX（读 Flash） | DMA 启动前丢弃 CPU 缓存中的旧副本 | `SCB_InvalidateDCache_by_Addr()` |

缓冲要求：
- `__ALIGNED(32)`：Cache 行对齐（H7 L1 cache line = 32 字节）。
- 放在 AXI SRAM / D2 SRAM：DTCM 无法被 DMA 访问。
- 本项目 TX/RX 缓冲：`g_flashTxBuf` / `g_flashRxBuf`，大小 256 字节。

## 5. RTOS 同步
- 使用信号量 `g_semFlashDmaDoneHandle`（二值信号量，初始 0）。
- DMA 传输完成中断链：`DMA TC → SPI EOT → HAL_SPI_Tx/RxCpltCallback → osSemaphoreRelease()`。
- 任务里 `osSemaphoreAcquire(g_semFlashDmaDoneHandle, osWaitForever)` 阻塞等待。
- **重要**：所有 Flash DMA 操作必须在 FreeRTOS 调度器启动后的任务上下文中执行，否则信号量永远不会释放。

## 6. API

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

## 7. 崩溃黑匣子
- 地址：`0x7FF000`（W25Q64 末扇区，8MB 最后 4KB）。
- 布局：
  - 字节 0–3：magic `0x43424144`（即 `"CRSH"`）。
  - 字节 4–7：日志长度 `len`（小端）。
  - 字节 256 起：日志体，按页（256B）轮询写入。
- 实现函数：`w25q_crashlog_save()` 完全轮询，不调用 RTOS/DMA。
- 调用点：`Core/Src/stm32h7xx_it.c` 的 `HardFault_Handler` 中调用 `logger_flush_to_flash()`。

## 8. 关键约束
- **页大小**：256 字节。`W25QXX_PageWrite` 单次最大 256B，跨页需调用方拆分。
- **扇区大小**：4096 字节。写之前必须擦除。
- **擦除寿命**：典型 10 万次。崩溃日志每次擦写末扇区，非高频场景可接受；若频繁崩溃需做磨损均衡。
- **CS 时序**：DMA 完成后、拉高 CS 之前必须确认 EOT（End Of Transfer），否则 CS 提前拉高导致数据截断。HAL 的 EOT 回调已保证这一点。

## 9. 后续可优化方向（受硬件约束）
| 方案 | 收益 | 前提 |
|---|---|---|
| 继续提 prescaler 到 `_2`（80 MHz）+ Fast Read `0x0B` | ~2× 读速 | 信号完整性验证通过 |
| 改用 QUADSPI + dual 模式 + XIP | 读速 2× + 内存映射 | 硬件改线到 QUADSPI 引脚 |
| 文件系统（LittleFS/SPIFFS） | 磨损均衡、寿命更长 | 引入额外代码量 |
