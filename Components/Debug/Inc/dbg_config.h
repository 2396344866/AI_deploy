/*
 * dbg_config.h
 * ---------------------------------------------------------------------------
 * 调试遥测总开关（编译期宏）—— 统一把开发期波形/变量观测从 UART4 搬到 UART1。
 *
 * 背景：
 *   - VOFA firewater 
 *   - UART1 本来就是双向控制台（printf/log TX + T/P/K 命令 RX），命令链路不动。
 *   - 硬件锁定只能从 UART1 输出 → 最优解：单 UART1 + firewater 固定帧 + 分组宏开关，
 *     零硬件改动。CH340/CH340G/CH340C 实测可达 2 Mbps，故波特率做成宏。
 *
 * 用法：烧录前改下面宏 → 重新编译（并在 Keil 把 Components/Debug/Inc 加入 Include Paths、
 *      把 Components/Debug/Src/dbg_telemetry.c 加入工程）→ 重新烧录。
 *      VOFA+ 接 UART1，波特率与 DBG_UART_BAUD 一致，选 firewater 协议，按 44 通道顺序拖控件。
 *
 * 固定帧宽 DBG_FRAME_N=44 与 dbg_telemetry.c、README.md 的通道映射表三者必须一致，勿随意改。
 * ---------------------------------------------------------------------------
 */
#ifndef DBG_CONFIG_H
#define DBG_CONFIG_H

/* ===== 串口与速率 ===== */
#define DBG_UART_BAUD          921600U   /* CH340 支持到 2M；115200 扛不住 37ch@200Hz(每帧≈330B)，会 TX 阻塞+丢帧 */
#define DBG_TELEMETRY_DECIMATE 1        /* 每 N 拍发 1 帧：1=每拍(≈200Hz)；115200 时设 4(≈50Hz) */

/* ===== 总开关 ===== */
#define DBG_TELEMETRY_ENABLE   1   /* 0 = 完全不发遥测（VOFA 关，UART1 仅文本/命令） */
#define DBG_LOG_ENABLE         0   /* 0 = 关掉高频文本日志（含每秒 MOTOR/PROBE 日志），避免与 firewater 抢视觉 */

/* ===== 分组子开关（仅当 DBG_TELEMETRY_ENABLE=1 生效）=====
 *   关闭的组其通道填 0，VOFA 波形布局不变（改宏只改数据、不改映射）。
 *   例：只测姿态 → DBG_TELEMETRY_MOTOR 0、DBG_TELEMETRY_SYSTEM 0。 
 *   摩托任务 + 姿态要一起看时，直接保持默认即可：这样同一帧里同时有姿态角(12–14)、三轴参考角/误差(15–20)、电机速度/PWM(24–29)，可以直观看"倾角变化→外环给出转向/速度指令→电机响应"的整条链路。
 */
 
 
#define DBG_TELEMETRY_IMU      1   /* 0-23  组A：MPU6050 原始/滤波 + 姿态角 + 三轴参考角/误差 + 增益 */
/* 0-2  原始 accel(LSB)；3-5 原始 gyro(LSB) */
/* 6-8  滤波 accel(g)；9-11 滤波 gyro(°/s) */
/* 12-14 姿态角 roll/pitch/yaw(deg) */
/* 15-16 roll_ref/roll_err；17-18 pitch_ref/pitch_err；19-20 yaw_ref/yaw_err (deg) */
/* 21-23 外环增益 kp/ki/kd */
/* 34-36 原始欧拉角 roll/pitch/yaw(deg)：后滤波前（融合直出），与 12-14 同屏对比滤波效果 */




#define DBG_TELEMETRY_MOTOR    1   /* 组B：电机/速度环（A/B 速度 + PWM + 目标速度） */
    /* 24-26 电机A：速度 / PWM / 目标速度（计数/节拍）；27-29 电机B */

#define DBG_TELEMETRY_SYSTEM   0   /* 组C：系统（循环周期/模式/运行/转向），默认关 */

/* ===== 固定帧宽（勿改：与 dbg_telemetry.c / README 映射表一致） ===== */
#define DBG_FRAME_N            44   /* 0-36 原 37 通道 + 37-43 磁力计(raw/calib/heading) */

#endif /* DBG_CONFIG_H */
