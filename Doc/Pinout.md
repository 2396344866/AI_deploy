# AI_deploy 引脚与定时器配置源（单一来源 / Single Source of Truth）

> **用途**：本文件是 CubeMX 引脚/定时器分配的**唯一可读配置源**。
> 以后凡涉及引脚改写、新增外设，先改本文件，再回到 CubeMX 同步 `.ioc`，
> 然后所有驱动代码只引用 CubeMX 生成的 `main.h` 宏（`xxx_Pin` / `xxx_GPIO_Port`）与
> `tim.h` 句柄（`htim1`/`htim3`/`htim4`/`htim7`），**不硬编码任何引脚号**。
>
> **时钟依据**：APB1 = APB2 = 120 MHz，但 H7 在 APB 预分频 ≠ 1 时定时器时钟 ×2，
> 故 **TIMxCLK = 240 MHz**（已核实 `RCC.APB1Freq_Value/APB2Freq_Value=120MHz`，D2PPRE1/D2PPRE2=DIV2）。
> **SYS**：已选 `Serial Wire`，关闭 JTAG，释放 PB3/PB4 为普通功能（BIN2 / TIM3_CH1）。

---

## 1. 系统 / 调试
| 项 | 配置 |
|---|---|
| SYS → Debug | Serial Wire（PA13=SWDIO, PA14=SWCLK） |
| 释放引脚 | PB3、PB4 不再被 JTAG/TRACE 占用 |

---

## 2. 电机驱动（单颗 TB6612FNG，双路 H 桥驱动双电机）
### 2.1 方向 / 使能 GPIO（System Core → GPIO，Output Push-Pull, No pull, Speed=Low）
| 引脚 | User Label | 方向 | 初始电平 | 功能 |
|---|---|---|---|---|
| PB0 | `AIN1` | GPIO_Output | Low | 电机 A 输入1 |
| PB1 | `AIN2` | GPIO_Output | Low | 电机 A 输入2 |
| PB2 | `BIN1` | GPIO_Output | Low | 电机 B 输入1 |
| PB3 | `BIN2` | GPIO_Output | Low | 电机 B 输入2 |
| PB12 | `TB6612_STBY` | GPIO_Output | **High** | 使能（高有效，已在 CubeMX 置高） |

> TB6612 真值表（STBY=1）：
> - AIN1=1,AIN2=0 → 正转；AIN1=0,AIN2=1 → 反转；同电平 → 刹车/停止。

### 2.2 PWM 输出（TIM1，APB2，TIMxCLK=240 MHz）
| 通道 | 引脚 | User Label | 参数 |
|---|---|---|---|
| TIM1_CH1 | PA8 | 电机 A PWM | Prescaler=24-1(10MHz), ARR=1000-1, **10 kHz, 1000 级分辨率**, Pulse=0, Polarity=High |
| TIM1_CH2 | PE11 | 电机 B PWM | 同上 |

> PWM 占空比 = `CCR / 1000`（0~1000 对应 0~100%）。

---

## 3. 编码器接口（正交 ×4 倍频，16-bit 满量程 65535）
| 电机 | 定时器 | CH1 引脚 | CH2 引脚 | 滤波 | 中断 |
|---|---|---|---|---|---|
| 电机 A | TIM4 | PB6 (TIM4_CH1) | PB7 (TIM4_CH2) | ICxFilter=10 | **不使能**（轮询读 CNT） |
| 电机 B | TIM3 | PB4 (TIM3_CH1, NJTRST) | PB5 (TIM3_CH2) | IC1Filter=10 | **不使能**（轮询读 CNT） |

> 编码器模式：`Encoder Mode TI1 and TI2`（4 倍频）。

---

## 4. PID 控制环定时器（TIM7，APB1，TIMxCLK=240 MHz）
| 项 | 配置 |
|---|---|
| 模式 | Activated（内部时钟） |
| Prescaler | 240-1（分频后 1 MHz） |
| Counter Period (ARR) | 1000-1（1 MHz / 1000 = **1 kHz = 1 ms 节拍**） |
| Auto-reload preload | Enable |
| NVIC | TIM7 global interrupt = Enabled，优先级设为较高（当前 `HAL_NVIC_SetPriority(TIM7_IRQn,5,0)`） |

---

## 5. 既有板载资源（非本次新增，保留）
| 引脚 | User Label | 方向 | 备注 |
|---|---|---|---|
| PC13 | `KEY` | GPIO_Input, Pull-up | 用户按键（低电平触发） |
| PC0 | `RED` | GPIO_Output, High | RGB LED（高=灭） |
| PC1 | `GREEN` | GPIO_Output, High | RGB LED |
| PC2 | `BLUE` | GPIO_Output, High | RGB LED |
| PA4 | `SPI1_CSS` | GPIO_Output | W25Q64 片选 |
| PA8/PE11 | — | AF | TIM1 PWM（见 2.2） |
| PB6/7、PB4/5 | — | AF | 编码器（见 3） |
| USART1 / UART4 | — | AF | 串口 DMA+IDLE（见 BSP_UART.md） |
| SPI1 | — | AF | W25Q64（见 BSP_W25Q64.md，SCK=40 MHz） |

---

## 6. 控制环参数（速度/位置闭环，阶段1）
> 算法对齐参考例程 `Doc/大鱼电子-电机PID速度位置闭环控制实验例程`（增量式 PI）。
> 编码器 PPR 按 TT 带编码电机常见值 **11**（4 倍频后单圈 44 计数）估算；**需按实物铭牌/实测校准 `ENCODER_PPR`**。

| 参数 | 符号 | 初值 | 说明 |
|---|---|---|---|
| 控制节拍 | `MOTOR_CTL_DT_MS` | 1 ms | 由 TIM7 提供 |
| 速度环 Kp | `VELOCITY_KP` | 14 | 增量式 PI，待整定 |
| 速度环 Ki | `VELOCITY_KI` | 8 | 增量式 PI，待整定 |
| 位置环 Kp | `POSITION_KP` | 0.04 | 位置→速度目标 级联，待整定 |
| PWM 限幅 | `PWM_MAX` | 1000 | = ARR，满占空比 |
| 编码器 PPR | `ENCODER_PPR` | 11 | 单圈计数=44（4×），需校准 |
| 最大目标速度 | `TARGET_SPEED_MAX` | 200 计数/节拍 | 由位置环输出限幅 |

---

## 7. 接线速查（TB6612 ↔ 电机 ↔ 编码器）
```
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
        (VM 接电机电源 6~12V | VCC 接 3.3V【与H7 IO同电平】 | GND 必须共地)
```

> ⚠️ **VCC 电平铁律**：TB6612 逻辑供电 `VCC` 必须接 **3.3V**（取自 STM32 板），
> **绝不能接 5V**。原因：FNG 的输入高门槛 `Vih = 0.7 × VCC`，VCC=5V 时 Vih=3.5V，
> 而 H7 的 GPIO 输出高只有 ~3.3V，**达不到门槛 → 方向/PWM 信号识别不可靠**。
> VCC=3.3V 时 Vih=2.31V，H7 输出 3.3V 余量充足。电机动力电 `VM` 与逻辑电 `VCC` 是两路，互不影响。
> **三大 GND（STM32 / TB6612 / 电机电源）必须 physically 连到一起**，否则信号无回流路径。
