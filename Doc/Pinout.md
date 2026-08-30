# AI_deploy 硬件配置与外设规划总文档

> 用途：CubeMX 引脚、时钟、外设、FreeRTOS 配置的**唯一可读配置源**，以及新增外设的规划与命名规范。
> 以后凡涉及引脚改写、新增外设，先改本文件，再回到 CubeMX 同步 `.ioc`，
> 然后所有驱动代码只引用 CubeMX 生成的 `main.h` 宏（`xxx_Pin` / `xxx_GPIO_Port`）与
> HAL 句柄（`hi2c1` / `hspi1` / `huart4` / `husart1` / `htim1`/`htim3`/`htim4`/`htim7`），
> **不硬编码任何引脚号**。
>
> 说明：本文件由 `Pinout.md` 与 `peripheral_plan.md` 合并而成（2026-08-24）。原 `peripheral_plan.md` 已并入此处，仅保留重定向说明，避免两份文档重复维护。

---

## 0. 规划结论速览（先看这）

| 问题 | 结论 |
|---|---|
| 串口够不够？ | H743 有 4×USART + 4×UART + LPUART1，规划 5 个（USART1=VOFA、UART4=屏、USART2=ESP-01S/OTA、USART3=MAX485、USART6=ESP32-S3），**数量完全够，只是要在 .ioc 里新增** |
| 需要改现有引脚吗？ | **不动现有功能脚**。CAN 原想用 PB12 但**那已是 TB6612_STBY**，已改到 PA11/12；其余 USART2/3/6、MAX485_DE、USBtoCAN/USBto485 全部走空闲脚，SPI1/I2C1/TIM1/3/4 一个不动 |
| 最大瓶颈 | **不是串口，是 H743 单核算力分配**。图像检测改由 **ESP32-S3 CAM 接管**（零 DCMI 脚、零图像算力）；FDCAN 还需开宏。MCU 只需管 TJA1050(CAN) + MAX485 + 地磁 + 电机PID + 故障诊断 + 收 ESP32-S3 图像结论 |
| 最省事方案 | USBtoCAN、USBto485 是**电脑端 USB 转**，MCU 不占脚；**图像检测已定由 ESP32-S3 CAM 承担**（已购 N16R8+GC2145+Type-C底座），STM32 只跑 PID+故障诊断+通信，实时性无忧 |

---

## 1. 芯片与主时钟

| 项 | 配置 |
|---|---|
| MCU | STM32H743VITx，LQFP100，Cortex-M7，revV |
| 主频 | CPU = SYSCLK = AXI =  ย480 MHz |
| Flash latency | VOS0，SCALE0 |
| HSE | 25 MHz 外部晶振，PLL 源 |
| LSE | 32.768 kHz 外部晶振 |
| HCLK | 240 MHz（HPRE = DIV2） |
| APB1 / APB2 | 120 MHz（D2PPRE1/D2PPRE2 = DIV2） |
| 定时器时钟 | TIMxCLK = 240 MHz（APB 预分频 ≠ 1 时自动 ×2） |
| I2C/SPI/UART 时钟 | PCLK1/PCLK2 = 120 MHz |

---

## 2. 引脚总览（已用 + 规划新增）

### 2.1 已占用引脚（实查自 .ioc）

| 引脚 | User Label | 模式 | 功能 | 备注 |
|---|---|---|---|---|
| PA0 | — | UART4_TX | TJC HMI / 串口屏 | DMA1_Stream0 RX / DMA1_Stream1 TX |
| PA1 | — | UART4_RX | TJC HMI / 串口屏 | 同上 |
| PA4 | `W25Q64_CS` | GPIO_Output | W25Q64 片选 | 高有效，初始化置高 |
| PA5 | `W25Q64_SCK` | SPI1_SCK | W25Q64 时钟 | 40 MHz |
| PA6 | `W25Q64_DO` | SPI1_MISO | W25Q64 数据出 | — |
| PA7 | `W25Q64_DI` | SPI1_MOSI | W25Q64 数据入 | — |
| PA8 | — | TIM1_CH1 | 电机 A PWM | 10 kHz，1000 级 |
| PA9 | `LOG_TX` | USART1_TX | 日志 / VOFA | DMA1_Stream4 RX |
| PA10 | `LOG_RX` | USART1_RX | 日志 / VOFA | 同上 |
| PA13 | — | SWDIO | 调试 | Serial Wire |
| PA14 | — | SWCLK | 调试 | Serial Wire |
| PB0 | `TB6612_AIN1` | GPIO_Output | 电机 A 方向 | Low |
| PB1 | `TB6612_AIN2` | GPIO_Output | 电机 A 方向 | Low |
| PB2 | `TB6612_BIN1` | GPIO_Output | 电机 B 方向 | Low |
| PB3 | `TB6612_BIN2` | GPIO_Output | 电机 B 方向 | Low；SYS 选 Serial Wire 释放 |
| PB4 | — | TIM3_CH1 | 电机 B 编码器 A 相 | Encoder Mode TI12 |
| PB5 | — | TIM3_CH2 | 电机 B 编码器 B 相 | Encoder Mode TI12 |
| PB6 | — | TIM4_CH1 | 电机 A 编码器 A 相 | Encoder Mode TI12 |
| PB7 | — | TIM 4_CH2 | 电机 A 编码器 B 相 | Encoder Mode TI12 |
| PB8 | `MPU6050_SCL` | I2C1_SCL | IMU + 磁力计 | 400 kHz；上拉 |
| PB9 | `MPU6050_SDA` | I2C1_SDA | IMU + 磁力计 | 同上 |
| PB12 | `TB6612_STBY` | GPIO_Output | TB6612 使能 | 初始化置高 |
| PC0 | `RED` | GPIO_Output | RGB LED | 高电平灭，上拉 |
| PC1 | `GREEN` | GPIO_Output | RGB LED | 高电平灭，上拉 |
| PC2_C | `BLUE` | GPIO_Output | RGB LED | 高电平灭，上拉 |
| PC3_C | `MPU6050_INT` | EXTI3 | MPU6050 数据就绪中断 | 上拉，上升沿 |
| PC13 | `KEY` | GPIO_Input | 用户按键 | 上拉，低电平触发 |
| PC14 | — | OSC32_IN | LSE | — |
| PC15 | — | OSC32_OUT | LSE | — |
| PE11 | — | TIM1_CH2 | 电机 B PWM | 10 kHz，1000 级 |
| PH0 | — | OSC_IN | HSE | — |
| PH1 | — | OSC_OUT | HSE | — |

> SYS 选 **Serial Wire**，关闭 JTAG，因此 PB3/PB4 释放为普通 GPIO / TIM3 编码器使用。

### 2.2 规划新增引脚（待 .ioc 配置）

| 引脚 | User Label | 模式 | 功能 | 备注 |
|---|---|---|---|---|
| PA2 | `ESP01S_TX` | USART2_TX | ESP-01S（OTA/阿里云） | **中断(无DMA)**，115200 |
| PA3 | `ESP01S_RX` | USART2_RX | ESP-01S（OTA/阿里云） | **中断(无DMA)**，115200 |
| PB10 | `MAX485_TX` | USART3_TX | MAX485 / ModbusRTU | — |
| PB11 | `MAX485_RX` | USART3_RX | MAX485 / ModbusRTU | — |
| PB14 | `MAX485_DE` | GPIO_Output | MAX485 方向控制（高=发/低=收） | 推挽，默认低 |
| PA11 | `TJA1050_RX` | FDCAN1_RX | TJA1050 CAN | AF9，避开 PB12(STBY) |
| PA12 | `TJA1050_TX` | FDCAN1_TX | TJA1050 CAN | AF9，避开 PB12(STBY) |
| PC6 | `ESP32S3_TX` | USART6_TX | ESP32-S3 CAM（命令） | 921600，DMA2 |
| PC7 | `ESP32S3_RX` | USART6_RX | ESP32-S3 CAM（图像结论） | 921600，DMA2 |

> ⚠️ **CAN 必须用 PA11/12**：PB12 已被 TB6612_STBY 占用，PB8/9 是 I2C1，均不可用于 CAN。
> ⚠️ **ESP-01S 与 ESP32-S3 不可共用 USART2**：ESP-01S 固定 USART2(PA2/3) 做 OTA；ESP32-S3 走独立 USART6(PC6/7)，避免 AT/MQTT 与图像帧互相打断。

---

## 3. 外设详细配置

### 3.1 SYS
| 项 | 配置 |
|---|---|
| Debug | Serial Wire（SWDIO/SWCLK） |
| TimeBase Source | TIM6（`TIM6_DAC_IRQn`，优先级 15） |
| DWT 周期计数器 | 由 `logger_tick_init()` 使能（`CoreDebug->DEMCR.TRCENA` + `DWT->CTRL.CYCCNTENA`），供日志时间戳；**与 SysTick 解耦，ISR 上下文读时间戳也准确**（详见 §3.11） |

### 3.2 I2C1（MPU6050 + GY273/QMC5883L）
| 项 | 配置 |
|---|---|
| 引脚 | SCL = PB8，SDA = PB9 |
| 模式 | I2C Master |
| 速度 | Fast Mode Plus，Timing = `0x307075B1`（约 400 kHz） |
| 外部上拉 | 必须，3.3 V |
| 设备地址 | MPU605 6050 = `0x68`（AD0 接地）；QMC5883L = `0x0D`；GY-273(HMC5883L) = `0x1E` |

MPU6050 INT 接 PC3，EXTI3 中断触发姿态解算。三者同总线靠地址区分，零新增引脚。

### 3.3 SPI1（W25Q64）
| 项 | 配置 |
|---|---|
| 引脚 | CS = PA4，SCK = PA5，MISO = PA6，MOSI = PA7 |
| 模式 | Full-Duplex Master |
| 时钟 | 40 MHz（PCLK2 = 160 MHz，预分频 DIV4） |
| 极性/相位 | CPOL = Low，CPHA = 1 Edge |
| 数据位 | 8 bit |
| DMA | SPI1_TX = DMA1_Stream2，SPI1_RX = DMA1_Stream3 |

### 3.4 USART1（调试 / VOFA）
| 项 | 配置 |
|---|---|
| 引脚 | TX = PA9（`LOG_TX`），RX = PA10（`LOG_RX`） |
| 模式 | Asynchronous |
| 波特率 | 921600（代码中 `DBG_UART_BAUD` 可改；VOFA 须同步 921600；此前"乱码"是 VOFA 误设 115200 的波特率不匹配，本机 921600 稳定，见 Components/Debug/Error/logger_error.md E18） |
| DMA | USART1_RX = DMA1_Stream4（Circular） |
| 中断 | USART1 global interrupt，优先级 8 |

### 3.5 UART4（TJC HMI / 串口屏）
| 项 | 配置 |
|---|---|
| 引脚 | TX = PA0，RX = PA1 |
| 模式 | Asynchronous |
| 波特率 | 115200（TJC 默认；代码中可调） |
| DMA | UART4_RX = DMA1_Stream0（Circular），UART4_TX = DMA1_Stream1（Normal） |
| 中断 | UART4 global interrupt，优先级 6 |

### 3.6 USART2（ESP-01S / OTA / 阿里云 MQTT）★新增
| 项 | 配置 |
|---|---|
| 引脚 | TX = PA2（`ESP01S_TX`），RX = PA3（`ESP01S_RX`） |
| 模式 | Asynchronous |
| 波特率 | 115200（ESP-01S AT 固件默认） |
| DMA | **无**（实际未配；驱动改用 `HAL_UARTEx_ReceiveToIdle_IT` 中断接收，同 ESP32-S3 决策） |
| 中断 | USART2 global interrupt |
| 注意 | ESP-01S 固定做 OTA/阿里云，**不与 ESP32-S3 共用** |

### 3.7 USART3（MAX485 / ModbusRTU）★新增
| 项 | 配置 |
|---|---|
| 引脚 | TX = PB10（`MAX485_TX`），RX = PB11（`MAX485_RX`） |
| 模式 | Asynchronous |
| 波特率 | 按从机设备定（常用 9600/19200/115200） |
| 方向控制 | PB14（`MAX485_DE`）推挽输出，高=发、低=收，半双工切换靠它 |
| DMA | 可选 DMA |

### 3.8 USART6（ESP32-S3 CAM 图像结论）★新增
| 项 | 配置 |
|---|---|
| 引脚 | TX = PC6（`ESP32S3_TX`），RX = PC7（`ESP32S3_RX`） |
| 模式 | Asynchronous |
| 波特率 | 921600（该串口无屏占用，可用高速） |
| DMA | DMA2_Streamx（接收，配合 IDLE 中断切帧） |
| 协议 | 帧头 `0xAA 0x55` + 长度 + CMD + 载荷 + CRC16，详见 §12.6 |

### 3.9 FDCAN1（TJA1050 CAN）★新增
| 项 | 配置 |
|---|---|
| 引脚 | TX = PA12（`TJA1050_TX`），RX = PA11（`TJA1050_RX`），AF9 |
| 注意 | **必须先在 `stm32h7xx_hal_conf.h` 打开 `HAL_FDCAN_MODULE_ENABLED`**（原第 38 行被注释） |
| 备选 | PD0/PD1（FDCAN1 另一组 AF9）也空闲 |

### 3.10 IWDG1 独立看门狗 ★已实现（2026-08-24）
| 项 | 配置 |
|---|---|
| 实例 | IWDG1 |
| 时钟源 | **LSI 32 kHz**（内部低速时钟，独立于主时钟/PLL；看门狗本质要求独立时钟，主时钟失效仍能复位） |
| 预分频 | `IWDG_PRESCALER_32` → 分频后 1 kHz（1 ms/计数） |
| 重载值 | `Reload = 4095` → **超时 ≈ 4095 ms ≈ 4.1 s** |
| 窗口值 | `Window = 4095` → 不约束提前喂狗（任意时刻可喂，无 early-window 限制） |
| 中断 | **无 NVIC 中断**（IWDG 超时直接硬复位，不可屏蔽，这是设计意图） |
| HAL 宏 | `HAL_IWDG_MODULE_ENABLED` 已在 `stm32h7xx_hal_conf.h` 启用 |
| 初始化 | `MX_IWDG1_Init()` 在 `main.c:128` 调用（所有外设初始化之后、`osKernelStart()` 之前） |
| 常态喂狗 | **`TIM7_IRQHandler`（NVIC prio3）**：每 1ms tick、每 500ms 经 `watchdog_should_feed()` 判定（`armed` 且各被监视任务心跳新鲜）才 `HAL_IWDG_Refresh(&hiwdg1)`；远低于 4.1 s 超时。`StartLoggerTask` **不再喂狗**（见 §7.1） |
| 黑匣子喂狗 | `logger_flush_to_flash()` 崩溃落盘期间调 `log_wdt_feed()`（与 boot 预喂 / POST 协作喂同一钩子），防 SPI/Flash 卡死时看门狗饿死 |
| 启动窗口风险 | `MX_IWDG1_Init()` 到首次喂狗之间：boot 段 `main.c` 调度器前先 `log_wdt_feed()` 预喂，POST 期由各 `Xxx_Test` 协作喂，POST 收尾 `watchdog_arm()` 后 TIM7 接管。含 OS 启动/W25Q 自检须 < 4.1 s；正常远小于，新增阻塞式上电自检需留意 |

> ⚠️ **铁律**：IWDG 一旦启动即无法关闭（除复位）。启用后**必须有常态喂狗**，否则系统约 4.1 s 后反复复位。本工程由 **TIM7_IRQHandler** 承担常态喂狗（见 §7.1）；POST 期由各 `Xxx_Test` 协作喂，`StartLoggerTask` 仅日志输出+控制台、不再喂狗。
> `log_wdt_feed()` 实现位于 `Core/Src/iwdg.c` 的 `USER CODE BEGIN 1` 区（CubeMX 安全区，重生成不丢）；声明在 `Components/Logger/Inc/logger.h`。

### 3.11 DWT 周期计数器（日志时间戳）★已实现（2026-08-24）
| 项 | 配置 |
|---|---|
| 模块 | Cortex-M7 内核 **DWT**（Data Watchpoint and Trace）`CYCCNT` 32 位自由运行计数器 |
| 时钟 | 与 CPU 同频（`SystemCoreClock` = 480 MHz），**非低频**；与 §3.10 的 LSI 看门狗时钟是两回事，勿混 |
| 使能 | `logger_tick_init()`：`CoreDebug->DEMCR |= TRCENA`、`DWT->CYCCNT = 0`、`DWT->CTRL |= CYCCNTENA`；在 `logger_init()` 内调用 |
| 读数 | `logger_get_tick()` 返回 `DWT->CYCCNT / (SystemCoreClock/1000)`，单位 ms |
| 优势 | **任意上下文（含更高优先级中断）读取都安全、零开销、精度高**；消除「ISR 中读 `HAL_GetTick()`(SysTick) 因 SysTick 优先级低于当前中断而读到旧值」的边界问题 |
| 回绕 | CYCCNT 约 9 s 回绕一次，日志时间戳仅作相对差，可接受；弱符号可覆盖为其他时钟源 |

---

## 4. 电机、编码器与控制节拍

### 4.1 TB6612FNG 方向 / 使能 GPIO
| 引脚 | User Label | 方向 | 初始电平 |
|---|---|---|---|
| PB0 | `TB6612_AIN1` | Output Push-Pull | Low |
| PB1 | `TB6612_AIN2` | Output Push-Pull | Low |
| PB2 | `TB6612_BIN1` | Output Push-Pull | Low |
| PB3 | `TB6612_BIN2` | Output Push-Pull | Low |
| PB12 | `TB6612_STBY` | Output Push-Pull | **High** |

TB6612 真值表（STBY = 1）：
- AIN1=1, AIN2=0 → 正转
- AIN1=0, AIN2=1 → 反转
- 同电平 → 刹车 / 停止

### 4.2 PWM 输出（TIM1，TIMxCLK = 240 MHz）
| 通道 | 引脚 | 功能 | 参数 |
|---|---|---|---|
| TIM1_CH1 | PA8 | 电机 A PWM | PSC=24-1，ARR=1000-1，10 kHz，1000 级分辨率 |
| TIM1_CH2 | PE11 | 电机 B PWM | 同上 |

占空比 = `CCR / 1000`（0~1000 对应 0~100%）。

### 4.3 编码器接口（TIM3 / TIM4，4 倍频）
| 电机 | 定时器 | CH1 引脚 | CH2 引脚 | 滤波 |
|---|---|---|---|---|
| 电机 A | TIM4 | PB6 | PB7 | IC1Filter=10 |
| 电机 B | TIM3 | PB4 | PB5 | IC1Filter=IC2Filter=10 |

模式：`Encoder Mode TI1 and TI2`（4 倍频）。
中断未使能，控制任务轮询读 `TIMx->CNT`。

### 4.4 控制环节拍（TIM7）
| 项 | 配置 |
|---|---|
| 时钟 | TIMxCLK = 240 MHz |
| 预分频 | 240-1 → 1 MHz |
| 计数周期 | 1000-1 → 1 kHz |
| 模式 | 内部时钟，ARR preload Enable |
| 中断 | TIM7 global interrupt，优先级 3 |

---

## 5. DMA 配置

| 请求 | 流 | 方向 | 模式 | 数据对齐 | 优先级 |
|---|---|---|---|---|---|
| UART4_RX | DMA1_Stream0 | 外设 → 内存 | Circular | Byte | Low |
| UART4_TX | DMA1_Stream1 | 内存 → 外设 | Normal | Byte | Low |
| SPI1_TX | DMA1_Stream2 | 内存 → 外设 | Normal | Byte | Medium |
| SPI1_RX | DMA1_Stream3 | 外设 → 内存 | Normal | Byte | Medium |
| USART1_RX | DMA1_Stream4 | 外设 → 内存 | Circular | Byte | Low |
| USART2_RX | —（未配 DMA，走 `ReceiveToIdle_IT` 中断） | — | — | — | — |
| USART2_TX | —（未配 DMA，走中断发送） | — | — | — | — |
| USART6_RX | DMA2_Streamx | 外设 → 内存 | Circular | Byte | — |

DMA 全局中断优先级统一为 6。

---

## 6. NVIC 优先级表

| 中断 | 抢占优先级 | 子优先级 | 说明 |
|---|---|---|---|
| NonMaskableInt / HardFault / MemoryManagement / BusFault / UsageFault / DebugMonitor / SVCall |  ย0 | 0 | 系统异常，不可调用 FreeRTOS API |
| TIM7_IRQn | 3 | 0 | 电机控制节拍，最高应用优先级 |
| TIM6_DAC_IRQn | 15 | 0 | HAL 时基 |
| SysTick_IRQn | 15 | 0 | FreeRTOS tick |
| PendSV_IRQn | 15 | 0 | 上下文切换 |
| DMA1_Stream0~4_IRQn | 6 | 0 | UART4/SPI1/USART1 DMA |
| SPI1_IRQn | 7 | 0 | SPI 完成中断 |
| UART4_IRQn | 6 | 0 | UART4 IDLE / 错误 |
| USART1_IRQn | 8 | 0 | USART1 IDLE / 错误 |
| EXTI3_IRQn | 6 | 0 | MPU6050 数据就绪 |

FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`：
- 优先级数值 **≤ 5** 的中断（抢占优先级更高）禁止调用 FreeRTOS API；
- TIM7（优先级 3）控制代码中不调用 FreeRTOS API，仅设置标志位。

---

## 7. FreeRTOS 配置

| 项 | 配置 |
|---|---|
| API | CMSIS-RTOS V2 |
| 调度方式 | Preemptive |
| Tick 频率 | 1 kHz |
| 最大优先级 | 56 |
| 堆实现 | Heap_4 |
| 堆大小 | 64 KB（`configTOTAL_HEAP_SIZE = 65536`） |
| FPU | 使能（`configENABLE_FPU = 1`） |
| MPU | 未使能 |
| 静态分配 | 支持 |
| 动态分配 | 支持 |
| 软件定时器 | 使能 |

### 7.1 任务
| 任务名 | 优先级 | 栈深 | 入口函数 | 说明 |
|---|---|---|---|---|
| Task_Inference | 24 | 1024 | `StartInferenceTask` | AI 推理 |
| Task_Motor | 40 | 512 | `StartMotorTask` | 电机控制 |
| Task_Network | 8 | 512 | `StartNetworkTask` | 网络 / 上云 |
| Task_Sensor | 32 | 512 | `StartSensorTask` | 传感器采集 |
| Task_Screen | 8 | 512 | `StartScreenTask` | TJC 屏显 |
| Task_Flash | 9 | 512 | `StartFlashTask` | Flash 持久化 |
| Task_logger | 10 | 512 | `StartLoggerTask` | 日志输出 + **调试控制台**（`DbgConsole_Process`，常驻不随模块开关失效）；喂狗由 TIM7_IRQHandler 承担，本任务不再喂狗 |

### 7.2 同步对象
| 类型 | 名称 | 初始状态 | 说明 |
|---|---|---|---|
| 互斥量 | `InferenceDataMutex` | Available | AI 输入数据保护 |
| 二值信号量 | `g_semScreenUpdate` | Depleted | 屏幕刷新触发 |
| 二值信号量 | `g_semInferenceLock` | Depleted | 推理完成同步 |
| 二值信号量 | `g_semFlashDmaDone` | Depleted | Flash DMA 完成 |
| 二值信号量 | `g_semAttitudeDataReady` | Available | 姿态数据就绪 |
| 队列 | `g_cmd_q` | 4 项 × 32 byte | 串口命令队列 |
| 队列 | `Queue_ImgResult` | — | ESP32-S3 图像结论（新增，待实现） |

---

## 8. 内存映射

| 区域 | 起始地址 | 大小 | 用途 |
|---|---|---|---|
| ITCMRAM | 0x0000_0000 | 64 KB | 指令 TCM |
| DTCMRAM | 0x2000_0000 | 128 KB | 数据 TCM |
| RAM（D1 AXI SRAM） | 0x2400_0000 | 512 KB | 主 RAM，默认数据区 |
| RAM_D2（D2 AHB SRAM） | 0x3000_0000 | 288 KB | DMA 缓冲区 / 外设数据 |
| RAM_D3（D3 AHB SRAM） | 0x3800_0000 | 64 KB | 备用 |
| FLASH | 0x0800_0000 | 2 MB | 程序 Flash |

---

## 9. 控制环参数（软件层）

> 算法对齐参考例程 `Doc/大鱼电子-电机PID速度位置闭环控制实验例程`（增量式 PI）。
> 编码器 PPR 按 TT 带编码电机常见值 **11**（4 倍频后单圈 44 计数）估算；需按实物铭牌或实测校准 `ENCODER_PPR`。

| 参数 | 符号 | 初值 | 说明 |
|---|---|---|---|
| 控制节拍 | `MOTOR_CTL_DT_MS` | 1 ms | 由 TIM7 提供 |
| 速度环 Kp | `VELOCITY_KP` | 14 | 增量式 PI，待整定 |
| 速度环 Ki | `VELOCITY_KI` | 8 | 增量式 PI，待整定 |
| 位置环 Kp | `POSITION_KP` | 0.04 | 位置→速度目标级联，待整定 |
| PWM 限幅 | `PWM_MAX` |  ย1000 | = ARR，满占空比 |
| 编码器 PPR | `ENCODER_PPR` | 11 | 单圈计数 = 44（4×），需校准 |
| 最大目标速度 | `TARGET_SPEED_MAX` | 200 计数/节拍 | 由位置环输出限幅 |

---

## 10. 引脚命名常量（与 main.h 风格统一）

> 风格说明：沿用 CubeMX 在 `Core/Inc/main.h` 的既有约定——**每个功能引脚拆两个宏**：`XXX_Pin`（= `GPIO_PIN_x`）+ `XXX_GPIO_Port`（= `GPIOx`）。
> 命名用「**功能语义 + 引脚角色**」的蛇形大写；新增外设**全部照此风格**在 `main.h` 生成/补充，不要出现裸 `GPIO_PIN_x` 或 `TIMX_pin` 这类无语义写法。

### 10.1 引脚命名常量（建议写入 main.h）
```c
/* ============ ESP-01S (OTA / 阿里云 MQTT) — USART2 ============ */
#define ESP01S_TX_Pin            GPIO_PIN_2      /* PA2 */
#define ESP01S_TX_GPIO_Port      GPIOA
#define ESP01S_RX_Pin            GPIO_PIN_3      /* PA3 */
#define ESP01S_RX_GPIO_Port      GPIOA

/* ============ MAX485 (ModbusRTU) — USART3 ============ */
#define MAX485_TX_Pin            GPIO_PIN_10     /* PB10 */
#define MAX485_TX_GPIO_Port      GPIOB
#define MAX485_RX_Pin            GPIO_PIN_11     /* PB11 */
#define MAX485_RX_GPIO_Port      GPIOB
#define MAX485_DE_Pin            GPIO_PIN_14     /* PB14 方向控制：高=发 低=收 */
#define MAX485_DE_GPIO_Port      GPIOB

/* ============ TJA1050 (CAN PHY) — FDCAN1 ============ */
/* ⚠️ CAN 必须用 PA11/12，PB12 已被 TB6612_STBY(MOTOR_TT_NSTBY) 占用，PB8/9 是 I2C1 */
#define TJA1050_TX_Pin           GPIO_PIN_12     /* PA12 (FDCAN1_TX, AF9) */
#define TJA1050_TX_GPIO_Port     GPIOA
#define TJA1050_RX_Pin           GPIO_PIN_11     /* PA11 (FDCAN1_RX, AF9) */
#define TJA1050_RX_GPIO_Port     GPIOA

/* ============ ESP32-S3 CAM (图像协处理器) — USART6 ============ */
/* ⚠️ 独立串口，不走 DCMI；与 ESP-01S(USART2) 分开以免互相打断数据流 */
#define ESP32S3_TX_Pin           GPIO_PIN_6      /* PC6 (USART6_TX, STM32→ESP32 命令) */
#define ESP32S3_TX_GPIO_Port     GPIOC
#define ESP32S3_RX_Pin           GPIO_PIN_7      /* PC7 (USART6_RX, ESP32→STM32 图像结论) */
#define ESP32S3_RX_GPIO_Port     GPIOC

/* ============ 地磁 QMC5883L / GY273 (I2C1 复用) ============ */
#define QMC5883L_I2C_ADDR        0x0D            /* QMC5883L */
#define GY273_I2C_ADDR           0x1E            /* HMC5883L (GY-273) */
/* SCL/SDA 复用 MPU6050_SCL_Pin / MPU6050_SDA_Pin (PB8/PB9) */
/*
STM32 ↔ MPU6050	STM32的SCL/SDA → MPU6050的SCL/SDA
STM32的GPIO → MPU6050的INT	主控通过主总线配置MPU，并接收中断信号；DMP融合后的数据也走这条总线传回STM32。
MPU6050 ↔ GY-273	MPU6050的XCL → GY-273的SCL
MPU6050的XDA → GY-273的SDA
（VCC和GND并联）
*/
```

### 10.2 GPIO 配置指南（HAL 实操步骤）
1. 所有新增引脚在 CubeMX(.ioc) 里配置，让工具自动把 §10.1 的宏写进 `main.h`，**不要手写裸 `GPIO_PIN_x`**。
2. 方向脚（MAX485_DE、各使能脚）配为 GPIO_Output（推挽、无上下拉、低速即可）；MAX485_DE 默认拉低（接收态）。
3. 复用功能脚（USART/FDCAN）由对应外设 Init 自动配为 AF，无需手写 `GPIO_InitStruct.Alternate`。
4. ESP32-S3 CAM 接 USART6：在 `.ioc` 新增 USART6，TX=PC6 / RX=PC7，波特率 921600；DMA 可选 DMA2_Streamx。STM32 侧只做接收解析，不在 H743 配任何 DCMI/XCLK。
5. TJA1050 需开 `HAL_FDCAN_MODULE_ENABLED`（`stm32h7xx_hal_conf.h`）后，PA11/12 的 AF9 才能在 `.ioc` 选 FDCAN1_TX/RX。
6. 命名落地后，BSP 层照 `led.h`（板载 LED 驱动）的宏风格再封一层函数式宏，例如：
   ```c
   #define MAX485_DIR_TX()  HAL_GPIO_WritePin(MAX485_DE_GPIO_Port, MAX485_DE_Pin, GPIO_PIN_SET)
   #define MAX485_DIR_RX()  HAL_GPIO_WritePin(MAX485_DE_GPIO_Port, MAX485_DE_Pin, GPIO_PIN_RESET)
   ```

---

## 11. 定时器 / 电机命名风格规范

> 现状：电机驱动是 **TB6612**，TIM1 出 PWM（PA8=CH1 电机A、PE11=CH2 电机B），TIM3/TIM4 作编码器接口。
> 接入的电机是 **「TT 马达」减速步进电机（带编码器）**。据此统一命名，替换原 `TB6612_*` / 裸 `htim1` 语义。

### 11.1 命名总原则
- 电机对象用「功能角色」命名，不绑定具体驱动芯片型号（换驱动芯片不改业务名）。
- 定时器句柄保留 `htimX`（CubeMX 生成不可改），但**业务层用语义宏/变量指代**，如 `MOTOR_TT_PWM_TIMER = &htim1`。
- 引脚命名沿用 §10 的 `XXX_Pin`/`XXX_GPIO_Port` 风格，按电机通道拆 A/B（对应双路步进/双电机）。

### 11.2 推荐命名映射（替换现有 TB6612_*，逐引脚展开 + 对应物理脚）

> 实查 `.ioc` + `motor.c`：PWM_A=PA8(TIM1_CH1)、PWM_B=PE11(TIM1_CH2)、AIN1=PB0、AIN2=PB1、BIN1=PB2、BIN2=PB3、STBY=PB12。

| 原命名 | 新命名（TT 马达语义） | 物理脚 | 角色说明 |
|---|---|---|---|
| `TB661 *AIN1_Pin` / `_GPIO_Port` | `MOTOR_TT_PWMA_IN1_Pin` / `_GPIO_Port` | **PB0** | TT 电机 A 方向输入1（正/反转） |
| `TB6612_AIN2_Pin` / `_GPIO_Port` | `MOTOR_TT_PWMA_IN2_Pin` / `_GPIO_Port` | **PB1** | TT 电机 A 方向输入2 |
| `TB6612_BIN1_Pin` / `_GPIO_Port` | `MOTOR_TT_PWMB_IN1_Pin` / `_GPIO_Port` | **PB2** | TT 电机 B 方向输入1 |
| `TB6612_BIN2_Pin` / `_GPIO_Port` | `MOTOR_TT_PWMB_IN2_Pin` / `_GPIO_Port` | **PB3** | TT 电机 B 方向输入2 |
| `TB6612_STBY_Pin` / `_GPIO_Port` | `MOTOR_TT_NSTBY_Pin` / `_GPIO_Port` | **PB12** | TT 驱动使能（高有效，低=休眠）。**此脚被 TT 驱动永久占用，CAN 不得再用 PB12** |
| `htim1`（PWM） | `MOTOR_TT_PWM_TIMER`（=`&htim1`）| CH1=PA8 / CH2=PE11 | TT 马达 PWM 源，双路输出 |
| `htim3`（编码器） | `MOTOR_TT_ENC_TIMER_B`（=`&htim3`）| PB4/PB5 | TT 马达 B 编码器接口（按 motor.c：B=TIM3） |
| `htim4`（编码器） | `MOTOR_TT_ENC_TIMER_A`（=`&htim4`）| PB6/PB7 | TT 马达 A 编码器接口（按 motor.c：A=TIM4） |

> ⚠️ **命名与物理脚必须一起改**：`TB6612_*` → `MOTOR_TT_*_` 只是语义层；PB12(STBY)、PA8/PE11(PWM)、PB0-3(方向)、PB4-7(编码器) 这些**物理脚不变**。改 .ioc 的 GPIO_Label 即可，不必动接线。
> ⚠️ motor.c 注释（A=TIM4、B=TIM3）与 .ioc 编码器脚位需实现前统一校准，避免方向/计数接反。

> 若 TT 马达实际是**单路带编码器**步进（不是双路直流），则只保留 `MOTOR_TT_AIN1/AIN2` + `MOTOR_TT_ENC_TIMER_A`，B 通道删去；编码器接口保留 TI12 双沿计数。

### 11.3 业务层句柄示例（仿照现有 `BSP_*` 风格）
```c
/* Components/Motor/Inc/motor.h */
extern TIM_HandleTypeDef htim1;   /* PWM */
extern TIM_HandleTypeDef htim3;   /* 编码器 A */
extern TIM_HandleTypeDef htim4;   /* 编码器 B */

#define MOTOR_TT_PWM_TIMER      (&htim1)
#define MOTOR_TT_ENC_TIMER_A    (&htim3)
#define MOTOR_TT_ENC_TIMER_B    (&htim4)

typedef struct {
    TIM_HandleTypeDef *pwm_timer;     /* PWM 定时器 */
    uint32_t          pwm_ch_a;       /* 通道1 (PA8) */
    uint32_t          pwm_ch_b;       /* 通道2 (PE11) */
    TIM_HandleTypeDef *enc_timer;     /* 编码器定时器 */
    int32_t           enc_count;      /* 实时脉冲计数 */
    float             angle_deg;      /* 换算角度 */
} MotorTT_HandleTypeDef;

void MotorTT_Init(MotorTT_HandleTypeDef *m);
void MotorTT_SetSpeed(MotorTT_HandleTypeDef *m, int32_t pps);  /* 步/秒 */
int32_t MotorTT_GetEncoder(MotorTT_HandleTypeDef *m);          /* 读编码器 */
```

### 11.4 风格自检清单（提交前核对）
- [ ] 所有新引脚都在 `main.h` 有 `XXX_Pin` + `XXX_GPIO_Port` 双宏，无裸 `GPIO_PIN_x` 散落业务代码。
- [ ] 引脚名含**功能语义**（如 `MAX485_DE_`、`OV2640_PCLK_`），不出现 `TIMX_pin` 这种无语义写法。
- [ ] 电机命名统一前缀 `MOTOR_TT_`，不再出现 `TB6612_*`（除非硬件确为 TB6612 且需并存）。
- [ ] 定时器句柄业务层用语义宏指代（`M  MOTOR_TT_PWM_TIMER`），不直接裸用 `htim1` 做语义判断。
- [ ] 复用总线（I2C1 挂 MPU6050+QMC5883L）靠**从机地址**区分，不新增引脚/不新增 I2C 外设。
- [ ] ESP32-S3 CAM 走独立 **USART6(PC6/PC7)**，不与 ESP-01S 的 USART2 共用；图像协议按 §12.6 帧格式（帧头+长度+CRC16）。

---

## 12. ESP32-S3 CAM 接管图像检测（已定稿，2026-08-24 更新）

> **已购型号**：ESP32-S3 CAM 开发板 N16R8 【GC2145 镜头】 + Type-C 烧录底座（¥40 套装）。
> **决策**：图像采集 + 目标检测**全部交给 ESP32-S3**，STM32 H743 只通过串口收"图像结论"，**零 DCMI 脚、零图像算力**（原 OV2640 直连 H743 方案已废弃）。

### 12.1 为什么不用 OV2640 直连 H743（已弃用原因回顾）
OV2640 是并行 DCMI 摄像头，**最少 12~13 脚**（D0–D7 + PCLK/VSYNC/HSYNC + XCLK + SCCB），且在 H743 单核上跑图像 CNN 推理会和故障诊断/串级 PID 抢时间片。ESP32-S3 CAM 板载已走通 GC2145 + DCMI + PSRAM，原生扛采集和轻量 CNN，**把图像算力外移是最优解**。

### 12.2 三种图像接入方式对比（结论已定 B）
| 方案 | 做法 | MCU 侧新增引脚 | 图像算力谁扛 | 结论 |
|---|---|---|---|---|
| A. STM32 直连 OV2640 | DCMI 收帧 + EdgeImpulse 在 H743 跑检测 | 12 脚（PC0-7/PE4-6/PC9） | STM32 H743 | ❌ 已弃用 |
| **B. ESP32-S3 CAM 接管** | S3 采集+推理，H743 只收结论（USART6） | **0 脚**（走 USART6/PC6-7） | ESP32-S3 | ✅ **已定稿** |
| C. 触发式抓拍 | 定时抓 1 帧 JPEG 经 ESP/W25Q 暂存 | 同 B | ESP32-S3 | 可选增强（非实时） |

> **已落地为方案 B**：STM32 专注「电机串级 PID + 故障诊断推理 + 通信(CAN/485)」，ESP32-S3 专注「摄像头采集 + 目标检测」，两者串口解耦。

### 12.3 现有 EdgeImpulse 模型确认 + ESP32-S3 迁移注意点（2026-08-23 实查）
> 已读 `AI_deploy/Components/EdgeImpulse/` 导出文件，确认当前球磨机检测模型的技术栈，**迁移到 ESP32-S3 不是重训，而是改导出平台 + 改摄像头引脚**。

| 项 | 实际值 | 证据文件 |
|---|---|---|
| 部署类型 | **EON Compiler**（编译期定死张量内存，非 TFLM） | `tflite-model/tflite_learn_638348_4_compiled.cpp` + `ei_compute_*` 编译内核 |
| 量化 | **INT8** | `model_metadata.h:109` `EI_CLASSIFIER_TFLITE_INPUT_DATATYPE=INT8`；`model_variables.h:97` `quantized=1` |
| 任务 | **FOMO 目标检测**（输出质心网格，非 bounding box） | `EI_CLASSIFIER_LAST_LAYER_FOMO`；`OBJECT_DETECTION=1`；`OBJECT_DETECTION_COUNT=10`；阈值 0.5 |
| Arena 大小 | **155008 B（≈151KB）** | `tflite_learn_*_compiled.cpp:97` `kTensorArenaSize=155008` |
| Arena 存放 | **内部 RAM（`.tensor_arena` 段）** | `#pragma Bss(".tensor_arena")` |

> **结论：arena 不用手动"调大"** —— EON 编译期已算准峰值 151KB，扩了只浪费；仅换更大模型/升分辨率时按新导出值填。

**迁移三件真要操心的事**：
1. **Arena 放内部 RAM 还是 PSRAM**：ESP32-S3 内部 RAM 有限，若同时开 WiFi/蓝牙或模型变大，用 `EI_TENSOR_ARENA_LOCATION` 宏把 arena 指定到 **PSRAM**（板载 8MB 绰绰有余）。
2. **FOMO 输出是网格质心，不是框**：ESP32 侧需按输入分辨率换算成**归一化中心**再发 H743（cx_norm=(cell_x+0.5)/GRID_W，cy_norm=(cell_y+0.5)/GRID_H），×1000 取整按 §12.6 `CMD=0x01` 帧上报。
3. **GC2145 在 EI 官方 firmware 无现成配置**：在 Arduino 工程的 `camera_pins.h` / `camera_config_t` 填入 GC2145 对应 GPIO 与 SCCB 地址（典型 0x3C/0x78，需查模组 datasheet）。

**ESP32 实测性能**（EI 导出报告，INT8 总延迟 **1026 ms** / Flash **153.7 KB**；float32 总延迟 **4708 ms** / Flash **544.7 KB** / 精度 **76.92%**）→ ESP32 侧务必用 INT8。详细见 `Deployment_Guide.md`。

### 12.4 Keil 工程分组（NN 分离，支持双端测试）
- **`FaultDiag_NN`**：仅含故障诊断实际调用的 2 个 CMSIS-NN 源文件 + `ai_infer.c`：`arm_fully_connected_s8.c`、`arm_nn_vec_mat_mult_t_s8.c`（`arm_nn_requantize`/`arm_nn_read_q7x4_ia` 为头文件内联，已在 IncludePath）。
- **`EdgeImpluse_NN`**：ESP32-S3 那套 EdgeImpulse 推理库（其余 CMSIS-NN + porting/tensorflow/tflite-model，共 234 文件）。
- **效果**：关闭 `EdgeImpluse_NN` → 仍有 `FaultDiag_NN` 提供 INT8 推理符号，STM32 故障诊断照常编译；开启两者 → 可做 ESP32 双端对照测试。两分组无文件重叠（总文件数 326 不变）。

### 12.6 ESP32-S3 ↔ STM32 串口帧格式（图像结论，已落地 2026-08-26）

> 接收端实现见 `Components/BSP/ESP32S3`（USART6 921600 + `HAL_UARTEx_ReceiveToIdle_IT` + 状态机）。
> 本格式是 §3.8 引用的"帧头+长度+CMD+载荷+CRC16"的**权威定义**，ESP32 固件须严格按此组帧。

**字节布局**（小端；CRC16 低字节在前）：
```
| 帧头 H1 (0xAA) | 帧头 H2 (0x55) | 长度 L (1B) | CMD (1B) | 载荷 (L-1 B) | CRC16 低 (1B) | CRC16 高 (1B) |
```
- `L` = **CMD + 载荷** 的总字节数（即 `L = 1 + 载荷长度`）；整帧长度 = `L + 5`。
- **CRC16 计算区间** = 从 `CMD` 字节起、到载荷末尾止（共 `L` 字节）。
- **CRC 算法** = CRC-16/MODBUS：多项式 `0x8005`（反射 `0xA001`），初值 `0xFFFF`，输入/输出均反射，`xorout = 0x0000`。STM32 侧 `esp32s3_crc16()` 即此实现。

**CMD 列表**
| CMD | 含义 | 载荷格式 |
|---|---|---|
| `0x01` | 目标检测结果帧（FOMO 网格质心） | `N(1B)` + `N × (cls(1B) + cx(2B,0~1000) + cy(2B,0~1000) + conf(1B,0~100))` |

- `N`：本帧目标数（建议 ≤ 8）。
- `cls`：类别 ID（0=球磨机/矿料等，由 ESP32 侧定义）。
- `cx`/`cy`：归一化中心 ×1000，`cx = (cell_x+0.5)/GRID_W × 1000`、`cy = (cell_y+0.5)/GRID_H × 1000`；H743 收到 ÷1000 还原。
- `conf`：置信度 0~100。

**组帧示例**（1 个目标，cls=0, cx=512, cy=480, conf=87）：
```
AA 55 | L=0x09 | 01 | 01 00 02 00 E0 01 57 | CRC16低 CRC16高
```
（L=9 = 1(CMD)+8(载荷：N(1)+cls(1)+cx(2)+cy(2)+conf(1)=7 → 1+7=8 → L=1+8=9)；载荷 = `01 00 02 00 E0 01 57`）

**接收端状态机**：找帧头 `AA 55` → 读 L → 读 CMD → 收满 `L-1` 载荷 → 收 2B CRC16 → 校验 → 校验通过则 `CMD=0x01` 帧入 `Queue_ImgResultHandle`，失败记 `s_crc_err` 并丢弃。粘包/半包由 IDLE 中断自然切分（每帧为一串连续字节 + 线路空闲）。

---

## 13. 接线速查

```text
        STM32H743            TB6612FNG           电机 A / B
        ------------------------------------------------
        PB0=AIN1  ─────────► AIN1
        PB1=AIN2  ─────────► AIN2
        PA8=TIM1_CH1 ──────► PWMA  ──┐
        PB12=STBY ─────────► STBY    │
        PB2=BIN1  ─────────► BIN1    │
        PB3=BIN2  ─────────► BIN2    │
        PE11=TIM1_CH2 ─────► PWMB  ──┘
        PB6=TIM4_CH1 ──────► 电机 A 编码器 A 相
        PB7=TIM4_CH2 ──────► 电机 A 编码器 B 相
        PB4=TIM3_CH1 ──────► 电机 B 编码器 A 相
        PB5=TIM3_CH2 ──────► 电机 B 编码器 B 相
        (VM 接电机电源 6~12V | VCC 接 3.3V | GND 必须共地)

        STM32H743            MPU6050 / GY273
        ------------------------------------------------
        PB8=I2C1_SCL ──────► SCL
        PB9=I2C1_SDA ──────► SDA
        PC3=MPU6050_INT ───► INT

        STM32H743            W25Q64
        ------------------------------------------------
        PA4=W25Q64_CS  ────► CS
        PA5=W25Q64_SCK ────► CLK
        PA6=W25Q64_DO  ────► DO
        PA7=W25Q64_DI  ────► DI

        STM32H743            TJC 串口屏
        ------------------------------------------------
        PA0=UART4_TX ──────► RX
        PA1=UART4_RX ──────► TX

        STM32H743            USB-TTL（日志 / VOFA）
        ------------------------------------------------
        PA9=USART1_TX ─────► RX
        PA10=USART1_RX ────► TX

        STM32H743            ESP-01S（OTA/阿里云）
        ------------------------------------------------
        PA2=USART2_TX ─────► RX
        PA3=USART2_RX ─────► TX

        STM32H743            MAX485（ModbusRTU）
        ------------------------------------------------
        PB10=USART3_TX ────► DI
        PB11=USART3_RX ────► RO
        PB14=MAX485_DE ────► DE/RE

        STM32H743            TJA1050（CAN）
        ------------------------------------------------
        PA12=FDCAN1_TX ────► TXD
        PA11=FDCAN1_RX ────► RXD

        STM32H743            ESP32-S3 CAM（图像结论）
        ------------------------------------------------
        PC6=USART6_TX ─────► RX
        PC7=USART6_RX ─────► TX
```

> **VCC 电平铁律**：TB6612 逻辑供电 `VCC` 必须接 **3.3 V**（取自 STM32 板），不能接 5 V。
> FNG 输入高门槛 `Vih = 0.7 × VCC`，VCC=5 V 时 Vih=3.5 V，H7 GPIO 高电平约 3.3 V，会识别失败；VCC=3.3 V 时 Vih=2.31 V，余量充足。电机动力电 `VM` 与逻辑电 `VCC` 独立。
> **STM32 / TB6612 / 电机电源三地必须物理共地**。

---

## 14. 修改流程

1. 在 CubeMX 中调整引脚 / 外设 / FreeRTOS / 时钟。
2. 重新生成代码。
3. 更新本文件对应章节，使其与 `.ioc` 一致。
4. 驱动代码只使用 `main.h` 中的 `xxx_Pin` / `xxx_GPIO_Port` 宏和 HAL 句柄；禁止写死 GPIO 端口号。
5. Keil 中 Rebuild，确认 0 错误。

---

## 15. 易错点汇总

1. **FDCAN 默认未使能**：`hal_conf.h:38` 是注释状态，不开宏编译不过 CAN 代码。
2. **CAN 引脚别踩 I2C1 也别踩 STBY**：PB8/9=I2C1、PB12=TB6612_STBY，CAN 必须用 PA11/12（FDCAN1 默认 AF9，且当前 USB 未启用，PA11/12 空闲）。
3. **ESP-01S 与 ESP32-S3 必须分串口**：ESP-01S 固定 USART2(PA2/3) 做 OTA；ESP32-S3 走独立 USART6(PC6/7) 只传图像结论。**共用 USART2 会互相打断 AT+MJSON/OBS 数据流**，别省这个串口。
4. **ESP32-S3 CAM 供电要独立**：峰值 200~500mA，别从 H743 板载 LDO 取，单独 3.3V/5V 模块供电；GC2145 板载已处理 1.8V。
5. **MAX485 方向脚必须软件控制**：DE/RE 高=发、低=收，半双工 ModbusRTU 收发切换靠它，忘了会收不到回包。
6. **ESP32-S3 串口协议用帧头+长度+CRC16**：见 §12.6，不要用裸文本行当正式协议，易截断/粘包；波特率建议 921600（USART6 无屏占用，可用高速）。
7. **USBtoCAN/USBto485 不占 MCU 资源**：别在 .ioc 里给它们找串口，它们是 PC 端工具。
8. **OV2640 直连 H743 方案已废弃**：不要再在 .ioc 配 DCMI/XCLK/MCO，图像全交给 ESP32-S3。
9. **关闭 EdgeImpluse_NN 后需实际 Build 一次**：确认无 `L6218E` 未定义符号（FaultDiag_NN 最小集基于静态追踪，若缺符号再补对应 .c）。
