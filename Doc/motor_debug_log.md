# 双电机闭环 + CubeMX 联调手册
> STM32H743VIT6 + FreeRTOS(CMSIS-RTOS2) + Keil MDK。TB6612FNG 双 H 桥。
> 更新 2026-08-17：ai_task.c 删除、命令通道迁 CubeMX 队列、Logger 归 CubeMX 原生。

## 状态
- A/B 闭环 ✅ 可控（B 用 cmd_dir=-1 校正中心对称）
- 中心对称 ✅ A50=左轮前进 / B50=右轮后退
- RTOS 对象 ✅ 全 CubeMX 原生（4 互斥量/信号量 + g_cmd_q 队列 + 7 线程）
- 堆/栈 ✅ CubeMX 永久化（堆 64KB；Inference 1024w + 6×512w）

## 接线（已核对，非硬件问题）
| 信号 | 引脚 | 备注 |
|------|------|------|
| PWMA / PWMB | PA8 / PE11 | TIM1_CH1/2, AF1, 10kHz PWM |
| AIN1/2, BIN1/2 | PB0/1 / PB2/3 | 方向 |
| STBY | PB12 | 高有效 |
| ENC_A / ENC_B | PB6/7(TIM4) / PB4/5(TIM3) | ×4 正交 |
| 中心对称 | 两电机绕中心 180° 重合 | 红线右上/左下相反 |

## Bug 链（已修复）
- 无日志：堆 15KB→64KB；6 任务栈收回 2048B
- 命令通道：旧"volatile 标志+轮询"→ 改 CubeMX 队列 g_cmd_qHandle（ISR Put / 任务 Get）
- TIM1 误判冻结：1kHz 采样 × 10kHz PWM 整数倍混叠；看 TIM1->CCR1/2 或 TIM1span
- B 刹不住：中心对称+编码器反向 → 增量 PI 正反馈；B cmd_dir=-1 + target=0 指数泄放(×0.85)
- StartLoggerTask 链接失败(L6218E)：旧函数体在生成主体外被冲 → 现已归 CubeMX 原生 Task_logger

## 关键设计
- 符号：A cmd_dir=+1；B cmd_dir=-1；enc_sign 均 +1
- PI：bias=target-enc；pid_pwm += Kp·Δbias + Ki·bias；限幅 ±PWM_MAX(999)
- target=0：pid_pwm×=0.85 泄放刹车
- PWM_MAX=999（ARR=999，防 CCR=1000 恒高）
- 控制环：TIM7 1ms 中断 → Motor_1ms_Handler → Motor_ControlStep（速度环 + 位置环级联）

## 命令
`A<v>`/`B<v>` 设速（自动 Resume）| `S` 急停 | `R` 恢复

## CubeMX 对象（均原生，重生安全）
| 对象 | 类型 |
|------|------|
| InferenceDataMutex | Mutex |
| ScreenDataSem / InferenceLockedSemHandle / g_FlashDmaDone | Binary Sem |
| g_cmd_q | Queue(4×32B) |
| Task_Inference/Motor/Network/Sensor/Screen/Flash/logger | 7 Thread |

## 栈 / 堆 / 中断
- 堆 64KB（configTOTAL_HEAP_SIZE）
- 栈：Inference 1024w(4096B)；其余 6×512w(2048B)
- NVIC 组=4；MAX_SYSCALL_PRIO=5；TIM7=5, USART1=8, UART4=6, SPI1=7
- ISR 调 *FromISR API 须 PRIO≥5（USART1=8 ✅，g_cmd_q Put 合法）

## 覆盖规则
- Components/ 永不覆盖；Core/ 生成主体重生覆盖；USER CODE 块保留
- RTOS 对象/栈/堆：改 .ioc GUI，勿手写

## 续调 TODO
1. PID 未整定（target 远超满速≈7 counts/ms）；限 0–8 或调 Kp/Ki
2. RPM 不准：ENCODER_PPR=11 未乘减速比；查铭牌修正
3. 机械速度差：发 A5/B5 验证，必要时加 speed_scale
4. 推理：Task_Inference 直接调 AI_Inference()；精度/Flash 见 Deployment_Guide
