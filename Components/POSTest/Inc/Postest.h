/*
 * Postest.h —— 上电自检（POST）编排层头文件
 * ---------------------------------------------------------------------------
 * 框架只保留编排（Postest_RunAll + 自检表）与无驱动的 Screen_Test 桩。
 * 各模块的自检实现已下沉到各自组件的 .c/.h：
 *   Flash_Test        → BSP_W25Q64.c/.h
 *   FaultDiag_ML_Test → ai_infer.c/.h
 *   Sensor_Test       → attitude.c/.h (BSP/IMU)
 *   Esp32S3_Test      → esp32s3.c/.h (BSP/ESP)
 *   Motor_Test        → motor.c/.h
 *   Network_Test      → esp01s.c/.h (BSP/ESP)
 *   Logger_Test       → logger.c/.h
 * 这样自检与组件同生命周期、同编译门控(APP_ENABLE_X)，避免 Postest.c 越权包含全部模块内部细节。
 *
 * 本头仅暴露编排入口 Postest_RunAll（供 StartTestTask 调用）与框架内置的 Screen_Test 桩；
 * 其余 Xxx_Test 声明分散在各组件头，由 Postest.c 在本 TU 内 include 后直接引用。
 * ---------------------------------------------------------------------------
 */
#ifndef POSTEST_H
#define POSTEST_H

/* 一站式调度入口：独立 Task_Test（CubeMX 生成，osPriorityHigh）在调度器启动后一次性调用。 */
int Postest_RunAll(void);

/* 屏幕自检桩（工程当前无 Components 级屏幕驱动，保留 SKIP；后续接 OLED/LCD 再补 Init+Clean+图案自检）。 */
int Screen_Test(void);

#endif /* POSTEST_H */
