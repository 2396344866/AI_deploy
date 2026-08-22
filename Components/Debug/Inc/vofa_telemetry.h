#ifndef _VOFA_TELEMETRY_H
#define _VOFA_TELEMETRY_H

#include "imu_filter.h"
#include "attitude.h"

/* =============================================================================
 * VOFA+ 遥测（兼容层，已弃用）
 *   firewater 调试流已迁至 UART1：见 Components/Debug/dbg_telemetry.c。
 *   此处 Vofa_Send 仅作转发，不再经 UART4 输出（UART4 留给淘晶驰串口屏）。
 *   通道布局见 Components/Debug/README.md（30 通道固定帧）。
 * ============================================================================= */
void Vofa_Send(const ImuData_t *imu, const Attitude_t *att, int32_t tgtA, int32_t tgtB);

#endif /* _VOFA_TELEMETRY_H */
