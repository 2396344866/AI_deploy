# Logger 模块说明

## 1. 定位
本项目统一日志模块，替代裸 `printf/fputc` 轮询输出。

## 2. 设计要点
| 特性 | 说明 |
|---|---|
| 分级 | ERROR / WARN / INFO / DEBUG / TRACE |
| 日志格式 | `[ms][级别][标签] (file:line) 内容\r\n` |
| 时间戳 | `HAL_GetTick()`，单位 ms |
| 标签 | 模块/任务名，如 `INFER`、`MOTOR`、`NET`、`FLASH` |
| 编译开关 | 不定义 `LOG_ENABLED` 时所有 `LOG_*` 宏展开为 `((void)0)`，零开销 |
| 两级过滤 | 编译上限 `LOG_MAX_LEVEL` + 运行时 `logger_set_level()` |
| 异步 | 生产任务只把格式化后的日志入 RAM 环形缓冲；`StartLoggerTask` 低优先级循环 `logger_drain()` 刷串口 |
| 线程/中断安全 | 入缓冲用 `PRIMASK` 关中断保护，不依赖 RTOS |
| 可插拔后端 | `log_backend_putc(char c)` 为 `__weak` 空实现，由 `BSP_USART.c` 提供 USART1 真实输出 |
| 黑匣子 | `logger_flush_to_flash()` 把缓冲刷入 W25Q64 末扇区，可在 `HardFault` 中调用 |

## 3. 文件位置
```
Components/Logger/Inc/logger.h
Components/Logger/Src/logger.c
```
> 放在 `Components/` 而非 `Core/`，避免 CubeMX 重新生成时清理/漏编。

## 4. 用法示例
```c
#include "logger.h"

LOG_I("INFER", "AI Task Started. samples=%d", n);
LOG_E("FLASH", "ID mismatch: 0x%06X", id);
LOG_D("MOTOR", "pwm=%d angle=%.2f", pwm, angle);
```

输出：
```
[806][I][INFER] (freertos.c:289) Summary: samples=1605 acc=0.9520
[39][E][FLASH] (freertos.c:584) ID mismatch: 0x000000
```

## 5. 配置项
在 `logger.h` 或编译命令中覆盖：

| 宏 | 默认值 | 含义 |
|---|---|---|
| `LOG_ENABLED` | 定义 | 总开关；注释掉则零开销关闭 |
| `LOG_MAX_LEVEL` | `LOG_LVL_DEBUG` | 编译进二进制的最高级别 |
| `LOG_LEVEL` | `LOG_LVL_INFO` | 运行时默认显示级别 |
| `LOG_SHOW_FILE_LINE` | `1` | 是否打印 `文件:行号` |

单文件覆盖（例如某驱动只想开到 WARN）：
```c
#define LOG_LOCAL_LEVEL LOG_LVL_WARN
#include "logger.h"
```

## 6. 与 BSP/HAL 的关系
- **不要**把日志实现塞进 BSP/HAL 驱动里。
- 驱动只调用 `LOG_*` 宏；后端通过弱符号接入。这样日志模块与具体串口、Flash 解耦，便于复用和测试。

## 7. 黑匣子（Flash 崩溃日志）
- 调用：`logger_flush_to_flash()`（已在 `HardFault_Handler` 中调用）。
- 写路径：logger.c 把环形缓冲线性化 → `w25q_crashlog_save()` 轮询写入 W25Q64。
- 写路径**不依赖 RTOS/DMA**，所以 HardFault 中也能用。
- 存储布局：末扇区 `0x7FF000`，前 8 字节 = magic `CRSH` + 日志长度，之后为日志体。

## 8. 注意事项
- 单条日志最大长度 `LOG_LINE_MAX = 160`；超长会被截断。
- 环形缓冲 `LOG_RB_SIZE = 1024`；满时覆盖最旧字节，保证最近日志不丢。
- `StartLoggerTask` 优先级要低，避免阻塞实时任务。
