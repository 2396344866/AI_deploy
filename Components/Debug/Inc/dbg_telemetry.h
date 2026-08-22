#ifndef DBG_TELEMETRY_H
#define DBG_TELEMETRY_H

#include "dbg_config.h"
#include "imu_filter.h"   /* ImuData_t */
#include "attitude.h"     /* Attitude_t */

/* =============================================================================
 * 调试遥测（开发期波形观测）
 *   把 MPU6050 / 姿态外环 / 电机速度环 / 系统状态 统一经 UART1 以 VOFA+ firewater
 *   协议输出。固定 30 通道宽帧，按宏开关分组（见 dbg_config.h）。
 *
 *   调用：传感器任务（StartSensorTask）每拍调用 Dbg_Telemetry_Send(...)；
 *   签名兼容原 Vofa_Send，故 freertos.c 仅改 include 与函数名即可。
 *
 *   UART1 波特率由 Dbg_Telemetry_Init() 在 main.c 启动早期（DMA RX 启动前）用
 *   DBG_UART_BAUD 覆盖式重设，详见 Components/Debug/README.md。
 * ============================================================================= */

/* 启动时调用一次：用 DBG_UART_BAUD 重设 UART1 波特率（须在 BSP_UART1_RxStart 之前）。 */
void Dbg_Telemetry_Init(void);

/* 每拍调用：构建并发送一帧固定宽度 firewater。参数与 Vofa_Send 一致。 */
void Dbg_Telemetry_Send(const ImuData_t *imu, const Attitude_t *att,
                         int32_t tgtA, int32_t tgtB);

#endif /* DBG_TELEMETRY_H */
