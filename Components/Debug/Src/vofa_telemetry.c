/* =============================================================================
 * VOFA+ 遥测（兼容层，已弃用）
 *   firewater 调试流已迁至 UART1，见 Components/Debug/dbg_telemetry.c。
 *   本文件仅保留 Vofa_Send 作为兼容转发，不再向 UART4 输出，
 *   使 UART4 完全释放给淘晶驰串口屏。新代码请直接调用 Dbg_Telemetry_Send。
 * ============================================================================= */
#include "vofa_telemetry.h"
#include "dbg_telemetry.h"

void Vofa_Send(const ImuData_t *imu, const Attitude_t *att, int32_t tgtA, int32_t tgtB)
{
    Dbg_Telemetry_Send(imu, att, tgtA, tgtB);
}
