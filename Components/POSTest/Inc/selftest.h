#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>

/*
 * selftest.h —— 上电自检（POST）框架
 * ---------------------------------------------------------------------------
 * 设计（见 logger_error.md E24 / 开发规范）：
 *   - Init / Test 常驻编译（在模块 .c，不在 #if APP_ENABLE_X 内）；
 *     仅各任务的运行循环体由 #if APP_ENABLE_X 包。
 *   - 独立 Task_Test（CubeMX 生成，osPriorityHigh）上电一次性跑 Selftest_RunAll，
 *     跑完 osThreadTerminate 自删；不干扰控制任务实时性。
 *   - 关键失败清单 {Inference, Sensor, Esp32S3, Flash} → logger_flush_to_flash()+复位；
 *     非关键 {Motor, Network, Screen} → 打印+继续。
 *   - 自检任务自身在长段（ML 推理）内自喂狗，避免最低优先级 Logger 被饿死误复位。
 * ---------------------------------------------------------------------------
 */

/* 各模块自检：返回 0=通过，<0=失败 */
int Flash_Test(void);        /* 外部 Flash：读黑匣子扇区打印 → 擦 → W25QXX_Test */
int FaultDiag_ML_Test(void); /* 故障诊断 ML：1605 样本批量推理算指标，写 g_Test_results */
int Sensor_Test(void);       /* IMU/姿态（关键） */
int Esp32S3_Test(void);      /* 图像协处理器（关键） */
int Motor_Test(void);        /* 电机（非关键） */
int Network_Test(void);      /* 网络（非关键） */
int Screen_Test(void);       /* 屏幕（非关键） */

/* 一站式自检：顺序 Flash→Inference→Sensor→Esp32S3→Motor→Network→Screen。
 * 关键失败 → logger_flush_to_flash() + NVIC_SystemReset()；非关键失败 → 打印+继续。 */
int Selftest_RunAll(void);

#endif /* SELFTEST_H */
