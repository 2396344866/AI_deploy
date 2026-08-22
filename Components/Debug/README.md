# Components/Debug — 调试遥测（UART1 firewater 固定帧）

## 目的

淘晶驰串口屏占用 **UART4**，原 VOFA firewater 调试流搬到这里，**统一经 UART1 输出**。
UART1 本来就是双向控制台（`printf`/日志 TX + T/P/K 命令 RX），命令链路不受影响；
硬件锁定只能从 UART1 输出 → 本组件即最优解：**单 UART1 + firewater 固定帧 + 分组宏开关**，零硬件改动。

依赖：`BSP_USART`(UART 发送)、`imu_filter`/`attitude`/`motor`，**不依赖 Logger**（Logger 保持独立）。

> ⚠️ **运行期错误 / AI 误判统一记在 `error.md`**，本文件只写通道映射与使用，不重复记录故障。

## 通道映射表（固定 37 通道，索引 0..36）

关掉的组其通道填 `0`，VOFA 波形布局不变（改宏只改数据、不改映射）。

| idx | 名称 | 组 | 来源 | 单位 |
|----:|------|----|------|------|
| 0 | raw_ax | IMU | imu->raw_accel[0] | LSB |
| 1 | raw_ay | IMU | imu->raw_accel[1] | LSB |
| 2 | raw_az | IMU | imu->raw_accel[2] | LSB |
| 3 | raw_gx | IMU | imu->raw_gyro[0] | LSB |
| 4 | raw_gy | IMU | imu->raw_gyro[1] | LSB |
| 5 | raw_gz | IMU | imu->raw_gyro[2] | LSB |
| 6 | filt_ax | IMU | imu->ax | g |
| 7 | filt_ay | IMU | imu->ay | g |
| 8 | filt_az | IMU | imu->az | g |
| 9 | filt_gx | IMU | imu->gx | °/s |
| 10 | filt_gy | IMU | imu->gy | °/s |
| 11 | filt_gz | IMU | imu->gz | °/s |
| 12 | roll | 姿态 | att->roll | deg |
| 13 | pitch | 姿态 | att->pitch | deg |
| 14 | yaw | 姿态 | att->yaw | deg |
| 15 | roll_ref | 姿态外环 | Attitude_GetRollRef（三轴参考角之一，二次开发可用） | deg |
| 16 | roll_err | 姿态外环 | Attitude_GetRollErr (= roll - roll_ref) | deg |
| 17 | pitch_ref | 姿态外环 | Attitude_GetPitchRef（pitch 主平衡轴） | deg |
| 18 | pitch_err | 姿态外环 | Attitude_GetPitchErr (= pitch - pitch_ref，前倾为正) | deg |
| 19 | yaw_ref | 姿态外环 | Attitude_GetYawRef（三轴参考角之一，二次开发可用） | deg |
| 20 | yaw_err | 姿态外环 | Attitude_GetYawErr (= yaw - yaw_ref) | deg |
| 21 | kp | 姿态外环 | Attitude_GetGains | — |
| 22 | ki | 姿态外环 | Attitude_GetGains | — |
| 23 | kd | 姿态外环 | Attitude_GetGains | — |
| 24 | motA_speed | 电机/速度环 | Motor_GetSpeed(A) | count/tick |
| 25 | motA_pwm | 电机/速度环 | Motor_GetPWM(A) | 带符号 |
| 26 | motA_tgt | 电机/速度环 | Motor_GetTargetSpeed(A) | count/tick |
| 27 | motB_speed | 电机/速度环 | Motor_GetSpeed(B) | count/tick |
| 28 | motB_pwm | 电机/速度环 | Motor_GetPWM(B) | 带符号 |
| 29 | motB_tgt | 电机/速度环 | Motor_GetTargetSpeed(B) | count/tick |
| 30 | loop_ms | 系统 | HAL_GetTick 间隔 | ms |
| 31 | sys_mode | 系统 | g_motor_sys.mode | 0=速度/1=位置 |
| 32 | sys_running | 系统 | g_motor_sys.running | 0/1 |
| 33 | steer | 系统 | Attitude_GetSteer | count/tick |
| 34 | raw_roll | 姿态 | Attitude_GetRawRoll（滤波前欧拉角，与 CH12 对比） | deg |
| 35 | raw_pitch | 姿态 | Attitude_GetRawPitch（滤波前欧拉角，与 CH13 对比） | deg |
| 36 | raw_yaw | 姿态 | Attitude_GetRawYaw（滤波前欧拉角，与 CH14 对比） | deg |

## VOFA+ 导入步骤

1. 串口线接板子 **UART1**（PA9=TX, PA10=RX），波特率与 `DBG_UART_BAUD` 一致（当前 921600，可在 `dbg_config.h` 改到 2000000U）。
2. VOFA+ → 数据接收 → 协议选 **firewater**。
3. 按上表顺序（idx 0→36）拖 **37 个波形控件**，逐个改名（如 `raw_ax`、`pitch`、`motA_pwm`…）。
4. 板子运行后，波形即实时刷新；关闭某组对应控件显示平线（值为 0），无需重排。
5. 在线下发命令（VOFA 的发送框，或任意串口终端）：`T1` 开外环 / `T0` 关 / `P<roll>,<pitch>,<yaw>` 三轴参考角（如 `P0,-3.6,0`；也可用 `P<deg>` 仅设 pitch，如 `P-3.6`）/ `P` 无参打印三轴 ref 与 err / `K<kp>,<ki>,<kd>` 增益（须三数写全）/ `A<速度>` `B<速度>` 电机 / `S` 急停 / `R` 恢复 / `C` 静置重标定陀螺零偏 / `F` 或 `F print` 打印滤波配置、`F lag/trim/elag 1|0 [参数]` 运行期开关调参（详见《滤波测试完整手册.md》）/ `D` 诊断 MPU6050 寄存器与一次 raw 采样。命令与波形共用 UART1，互不干扰。

## 宏速查（`dbg_config.h`）

| 宏 | 当前值 | 作用 |
|----|------|------|
| `DBG_UART_BAUD` | 921600U | UART1 波特率（CH340 可到 2000000U） |
| `DBG_TELEMETRY_DECIMATE` | 1 | 每 N 拍发 1 帧（1=≈200Hz；115200 时设 4≈50Hz） |
| `DBG_TELEMETRY_ENABLE` | 1 | 总开关：0=完全不发遥测 |
| `DBG_LOG_ENABLE` | 0 | 文本日志总闸：0=关掉每秒 MOTOR/PROBE 日志 |
| `DBG_TELEMETRY_IMU` | 1 | 组A：MPU6050 + 姿态 |
| `DBG_TELEMETRY_MOTOR` | 0 | 组B：电机/速度环 |
| `DBG_TELEMETRY_SYSTEM` | 0 | 组C：系统（默认关） |

> 以上为当前构建配置（仅供 MPU6050 单独观测时的快照）。烧录前请直接看 `dbg_config.h` 实际值，本表以它为准。

## 分组开关示例

- 只测姿态（关掉电机/系统，VOFA 只显示 0-23）：`DBG_TELEMETRY_MOTOR 0`、`DBG_TELEMETRY_SYSTEM 0`。
- 只调速度环：`DBG_TELEMETRY_IMU 0`、`DBG_TELEMETRY_SYSTEM 0`。
- 纯文本调试（关波形、保留命令）：`DBG_TELEMETRY_ENABLE 0`。
- 关掉文本日志以免干扰波形：`DBG_LOG_ENABLE 0`。

## Keil 收尾（需手动）

1. **C/C++ → Include Paths** 增加 `..\Components\Debug\Inc`。
2. 把 `Components/Debug/Src/dbg_telemetry.c` 加入工程文件树。
3. 重新编译确认 0 错误（重点关注新增 getter 是否与已有声明冲突）。
4. 烧录用 VOFA+ 接 UART1（波特率=`DBG_UART_BAUD`）看波形。

## 实现说明

- `Dbg_Telemetry_Init()` 在 `main.c` 启动早期（`BSP_UART1_RxStart` 之前）用 `DBG_UART_BAUD`
  覆盖式重设 UART1 波特率，免改 `usart.c` / 免重跑 CubeMX。
- `Dbg_Telemetry_Send()` 在 `StartSensorTask`（200Hz，MPU6050 data-ready 信号量驱动）每拍调用，
  构建固定帧后经 `BSP_UART1_SendPoll` 发送（阻塞发送；H7 余量足，若 200Hz 周期被挤压可调 `DECIMATE`）。
- `Vofa_Send`（Components/BSP/IMU）保留为兼容转发，内部转调本组件，使 UART4 完全释放给淘晶驰屏。
