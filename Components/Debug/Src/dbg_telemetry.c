/* =============================================================================
 * 调试遥测（UART1 firewater 固定帧）
 *   设计：单 UART1 输出，固定 DBG_FRAME_N 通道宽帧；宏开关分组（DBG_TELEMETRY_*）。
 *   关掉的组其通道填 0，VOFA 波形布局稳定（改宏只改数据、不改映射）。
 *   通道映射/宏说明见 dbg_config.h 与 Components/Debug/README.md。
 * ============================================================================= */
#include "dbg_config.h"
#include "dbg_telemetry.h"
#include "motor.h"        /* Motor_GetSpeed / Motor_GetPWM / Motor_GetTargetSpeed / g_motor_sys */
#include "attitude.h"      /* Attitude_Get* */
#include "BSP_LOG.h"    /* BSP_LOG_UART1_SendPoll */
#include "usart.h"        /* huart1 (extern) */
#include "logger.h"      /* logger_get_level() : 运行期遥测闸门（>=DEBUG 才发帧） */
#include <string.h>       /* memset */
#include <stdio.h>        /* snprintf */

/* 一行最大长度：44 通道 × (最多 "%.3f," ≈ 8 字符) + 换行 + 余量 */
#define DBG_LINE_MAX   (DBG_FRAME_N * 10 + 8)

/* 严格分层 AND（与文本日志同构，共用 LOG_ENABLED 总闸；任一层不满足 -> 空实现/不发包）：
 *   1) LOG_ENABLED 定义            —— 日志框架总闸（生产模式注释掉即零开销关全部日志）
 *   2) 任一遥测分组开              —— DBG_TELEMETRY_IMU/MOTOR/SYSTEM 各自独立 OR（不再依附 DBG_LOG_MOTOR）
 *                                      （只看姿态 -> 仅开 IMU；只看电机环 -> 仅开 MOTOR；可任意组合）
 *   3) DBG_TELEMETRY_ENABLE == 1   —— 遥测特征开关（代码编进固件）
 *   4) logger_get_level() >= TRACE —— 运行期级别达 TRACE（最高冗长度；高吞吐波形属 TRACE 级）
 *   故"彻底开启遥测 = LOG_ENABLED 定义 + 任一分组开 + DBG_TELEMETRY_ENABLE=1
 *        + LOG_COMPILE_MAX_LEVEL>=TRACE + logger_set_level(TRACE)"，层层严格满足。
 *   正常业务只取数值解算、不参与打印；遥测是深挖传感器/控制环数据(VOFA 波形)，默认全关、仅调试短时开启。 */
	#if DBG_TELEMETRY_ENABLE && defined(LOG_ENABLED) && (DBG_TELEMETRY_IMU || DBG_TELEMETRY_MOTOR || DBG_TELEMETRY_SYSTEM)

extern UART_HandleTypeDef huart1;

static float    s_frame[DBG_FRAME_N];
static uint32_t s_tick      = 0;
static uint32_t s_last_ms   = 0;   /* 测传感器循环周期(ms) */

/* ---------------------------------------------------------------------------
 * Dbg_Telemetry_Init：在 main.c 启动早期（BSP_LOG_UART1_RxStart 之前）调用，
 * 用 DBG_UART_BAUD 覆盖 CubeMX 默认 115200，免改 usart.c / 免重跑 CubeMX。
 * ------------------------------------------------------------------------- */
void Dbg_Telemetry_Init(void)
{
    huart1.Init.BaudRate = (uint32_t)DBG_UART_BAUD;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        /* 波特率失败不致命：UART1 仍按 CubeMX 默认 115200 工作，日志照常。 */
    }
}

/* ---------------------------------------------------------------------------
 * Dbg_Telemetry_Send：构建固定宽度帧并经 UART1 以 firewater 发出。
 *   原始 float 格式化用 %.3f（与旧 Vofa_Send 一致），逗号分隔 + 换行。
 * ------------------------------------------------------------------------- */
void Dbg_Telemetry_Send(const ImuData_t *imu, const Attitude_t *att,
                        int32_t tgtA, int32_t tgtB)
{
    /* 运行期闸门：遥测帧是 TRACE 级诊断数据（最高 200Hz 二进制流，会占满 UART1）。
     * 仅当运行期级别 >= TRACE 才真正发帧；生产态（运行默认<=DEBUG）即便代码编进二进制也
     * 不发包，UART1 留给文本日志/命令。现场看波形 -> logger_set_level(LOG_LVL_TRACE) 即开，
     * 调回 DEBUG 即停，无需重烧。与文本日志共用同一个运行期旋钮。
     * 注意：TRACE 须先编进二进制（LOG_COMPILE_MAX_LEVEL>=TRACE），否则永远不发。 */
    if (logger_get_level() < LOG_LVL_TRACE) {
        return;
    }

    /* 降采样 */
    s_tick++;
    if ((DBG_TELEMETRY_DECIMATE > 1) &&
        ((s_tick % (uint32_t)DBG_TELEMETRY_DECIMATE) != 0U)) {
        return;
    }

    memset(s_frame, 0, sizeof(s_frame));

#if DBG_TELEMETRY_IMU
    /* 0-2  原始 accel(LSB)；3-5 原始 gyro(LSB) */
    s_frame[0] = (float)imu->raw_accel[0];
    s_frame[1] = (float)imu->raw_accel[1];
    s_frame[2] = (float)imu->raw_accel[2];
    s_frame[3] = (float)imu->raw_gyro[0];
    s_frame[4] = (float)imu->raw_gyro[1];
    s_frame[5] = (float)imu->raw_gyro[2];
    /* 6-8  滤波 accel(g)；9-11 滤波 gyro(°/s) */
    s_frame[6]  = imu->ax; s_frame[7]  = imu->ay; s_frame[8]  = imu->az;
    s_frame[9]  = imu->gx; s_frame[10] = imu->gy; s_frame[11] = imu->gz;
    /* 12-14 姿态角(deg) */
    s_frame[12] = att->roll; s_frame[13] = att->pitch; s_frame[14] = att->yaw;
    /* 15-20 三轴参考角 / 误差(deg)：roll_ref/roll_err；pitch_ref/pitch_err；yaw_ref/yaw_err */
    s_frame[15] = Attitude_GetRollRef();  s_frame[16] = Attitude_GetRollErr();
    s_frame[17] = Attitude_GetPitchRef(); s_frame[18] = Attitude_GetPitchErr();
    s_frame[19] = Attitude_GetYawRef();   s_frame[20] = Attitude_GetYawErr();
    /* 21-23 外环增益 */
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    Attitude_GetGains(&kp, &ki, &kd);
    s_frame[21] = kp; s_frame[22] = ki; s_frame[23] = kd;
    /* 34-36 原始欧拉角（滤波前，融合直出）：与 12-14 后滤波对比，验证欧拉角后滤波 */
    s_frame[34] = Attitude_GetRawRoll();
    s_frame[35] = Attitude_GetRawPitch();
    s_frame[36] = Attitude_GetRawYaw();
    /* 37-39 原始磁力计(counts)；40-42 标定磁力计(阶段2 前=raw)；43 磁航向(deg) */
    int16_t rm[3]; Attitude_GetRawMag(rm);
    s_frame[37] = (float)rm[0]; s_frame[38] = (float)rm[1]; s_frame[39] = (float)rm[2];
    float cm[3]; Attitude_GetCalibMag(cm);
    s_frame[40] = cm[0]; s_frame[41] = cm[1]; s_frame[42] = cm[2];
    s_frame[43] = Attitude_GetMagHeading();
#endif

#if DBG_TELEMETRY_MOTOR
    /* 24-26 电机A：速度 / PWM / 目标速度（计数/节拍）；27-29 电机B */
    s_frame[24] = (float)Motor_GetSpeed(MOTOR_A);
    s_frame[25] = (float)Motor_GetPWM(MOTOR_A);
    s_frame[26] = (float)Motor_GetTargetSpeed(MOTOR_A);
    s_frame[27] = (float)Motor_GetSpeed(MOTOR_B);
    s_frame[28] = (float)Motor_GetPWM(MOTOR_B);
    s_frame[29] = (float)Motor_GetTargetSpeed(MOTOR_B);
#endif

#if DBG_TELEMETRY_SYSTEM
    /* 30 循环周期(ms)；31 模式；32 运行；33 转向 */
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_last_ms;
    s_last_ms    = now;
    s_frame[30] = (float)dt;
    s_frame[31] = (float)g_motor_sys.mode;
    s_frame[32] = (float)g_motor_sys.running;
    s_frame[33] = (float)Attitude_GetSteer();
#endif

    /* 拼成 firewater 一行：逗号分隔 + 换行 */
    char line[DBG_LINE_MAX];
    int  off = 0;
    for (int i = 0; i < DBG_FRAME_N; i++) {
        off += snprintf(line + off, (size_t)(DBG_LINE_MAX - off),
                         (i == 0) ? "%.3f" : ",%.3f", s_frame[i]);
    }
    line[off++] = '\n';
    BSP_LOG_UART1_SendPoll((const uint8_t *)line, (uint16_t)off);
}

 #else /* 任一闸门不满足（LOG_ENABLED 未定义 / 三组分组全关 / DBG_TELEMETRY_ENABLE=0）：空实现，零运行时开销 */

void Dbg_Telemetry_Init(void) { /* 空 */ }
void Dbg_Telemetry_Send(const ImuData_t *imu, const Attitude_t *att,
                        int32_t tgtA, int32_t tgtB)
{
    (void)imu; (void)att; (void)tgtA; (void)tgtB;
}

#endif
