/* =============================================================================
 * 调试遥测（UART1 firewater 固定帧）
 *   设计：单 UART1 输出，固定 DBG_FRAME_N 通道宽帧；宏开关分组（DBG_TELEMETRY_*）。
 *   关掉的组其通道填 0，VOFA 波形布局稳定（改宏只改数据、不改映射）。
 *   通道映射/宏说明见 dbg_config.h 与 Components/Debug/Ref/motor_vofa_telemetry.md。
 * ============================================================================= */
#include "dbg_config.h"
#include "dbg_telemetry.h"
#include "motor.h"        /* Motor_GetSpeed / Motor_GetPWM / Motor_GetTargetSpeed / g_motor_sys */
#include "attitude.h"      /* Attitude_Get* */
#include "imu_mpu6050.h"   /* MPU_ACCEL_LSB_PER_G / MPU_GYRO_LSB_PER_DPS：raw LSB->物理量换算系数 */
#include "BSP_LOG.h"    /* BSP_LOG_UART1_SendPoll */
#include "usart.h"        /* huart1 (extern) */
#include "logger.h"      /* logger_get_level() : 运行期遥测闸门（>=TRACE 才发帧） */
#include <string.h>       /* memset */
#include <stdio.h>        /* snprintf */

/* 一行最大长度：59 通道 × (最多 "%.3f," ≈ 8 字符) + 换行 + 余量 */
#define DBG_LINE_MAX   (DBG_FRAME_N * 10 + 8)

/* 分层 AND（共用 LOG_ENABLED 总闸；任一层不满足→空实现/不发包）：
 *   1) LOG_ENABLED 定义            —— 日志框架总闸（生产注释掉即零开销）
 *   2) 任一遥测分组开              —— IMU/MOTOR/SYSTEM 独立 OR（只看姿态→仅 IMU；只看电机→仅 MOTOR；可任意组合）
 *   3) DBG_TELEMETRY_ENABLE == 1   —— 遥测特征开关（编进固件）
 *   4) logger_get_level() >= TRACE —— 运行级达 TRACE（波形属 TRACE 级）
 * 开启= LOG_ENABLED + 任一分组 + ENABLE=1 + LOG_COMPILE_MAX_LEVEL>=TRACE + logger_set_level(TRACE)，层层满足。
 * 遥测仅深挖传感器/控制环(VOFA 波形)，默认全关、调试短时开。 */
	#if defined(DBG_TELEMETRY_ENABLE)
	#if DBG_TELEMETRY_ENABLE && defined(LOG_ENABLED) && (DBG_TELEMETRY_IMU || DBG_TELEMETRY_MOTOR || DBG_TELEMETRY_SYSTEM)

extern UART_HandleTypeDef huart1;

/* VOFA 通道索引（与 dbg_config.h DBG_FRAME_N、Ref/motor_vofa_telemetry.md 三处一致）。
   命名常量替代魔法数字；CH_COUNT 须等于 DBG_FRAME_N（见下方 _Static_assert）。

   ══ 命名公约（四类语义，后缀互斥；完整版见 Ref/motor_vofa_telemetry.md §命名公约）══
   _tgt   目标/设定值(reference)，人为下发            | 例 roll_tgt / motA_tgt / motB_tgt
   _err   控制误差 = measured − tgt                  | 例 roll_err = roll − roll_tgt
   _res   滤波残差 = raw − flt                       | 例 res_ax = raw_ax − flt_ax
   _innov 观测新息 = observation − estimate          | 例 yaw_innov = mag_hdg − fused_yaw
   ⚠ REF(目标)≠RES(残差)≠INNOV(新息)：一字之差但语义迥异，故 CTRL 目标角用 _tgt 避开 REF/RES 撞名。
   前缀：raw_=SENSOR 组原始采样；flt_=滤波后；pre_=ATT 组后滤波前欧拉角(融合直出,非原始采样)。 */
typedef enum {
    /* ===== ACC 组(9)：原始 / 滤波 / 残差，单位 g ===== */
    CH_ACC_RAW_X, CH_ACC_RAW_Y, CH_ACC_RAW_Z,         /*  0-2   原始 accel(g, LSB÷16384 已换算) */
    CH_ACC_FLT_X, CH_ACC_FLT_Y, CH_ACC_FLT_Z,         /*  3-5   滤波 accel(g) */
    CH_ACC_RES_X, CH_ACC_RES_Y, CH_ACC_RES_Z,         /*  6-8   残差 accel(g) = raw - flt（滤波抑制量） */
    /* ===== GYRO 组(9)：原始 / 滤波 / 残差，单位 °/s ===== */
    CH_GYRO_RAW_X, CH_GYRO_RAW_Y, CH_GYRO_RAW_Z,       /*  9-11  原始 gyro(°/s, LSB÷131 已换算) */
    CH_GYRO_FLT_X, CH_GYRO_FLT_Y, CH_GYRO_FLT_Z,       /* 12-14  滤波 gyro(°/s) */
    CH_GYRO_RES_X, CH_GYRO_RES_Y, CH_GYRO_RES_Z,       /* 15-17  残差 gyro(°/s) = raw - flt */
    /* ===== MAG 组(9)：原始 / 滤波 / 残差，单位 counts ===== */
    CH_MAG_RAW_X, CH_MAG_RAW_Y, CH_MAG_RAW_Z,          /* 18-20  原始磁力计(counts, 芯片原生) */
    CH_MAG_FLT_X, CH_MAG_FLT_Y, CH_MAG_FLT_Z,          /* 21-23  滤波标定磁力计(counts) */
    CH_MAG_RES_X, CH_MAG_RES_Y, CH_MAG_RES_Z,          /* 24-26  残差磁力计(counts) = raw - flt */
    /* ===== ATT 组(9)：欧拉角 + yaw 融合诊断，单位 deg ===== */
    CH_ROLL, CH_PITCH, CH_YAW,                         /* 27-29  融合欧拉角(deg, 后滤波) */
    CH_ROLL_PRE, CH_PITCH_PRE, CH_YAW_PRE,             /* 30-32  后滤波前欧拉角(deg, 融合直出未平滑)——注意:非原始采样,pre_避免与 SENSOR 组 raw_ 混 */
    CH_YAW_GYRO,                                      /* 33     纯陀螺积分 yaw(°)：不碰磁，直观看漂移 */
    CH_MAG_HDG,                                        /* 34     倾角补偿磁航向(deg) */
    CH_YAW_INNOV,                                      /* 35     yaw 新息(°)：磁校正前 yaw_wrap_diff(mag_hdg, fused) */
    /* ===== GBIAS 组(3)：在线陀螺零偏估计，单位 °/s ===== */
    CH_GBIAS_X, CH_GBIAS_Y, CH_GBIAS_Z,                /* 36-38  在线零偏估计(°/s) */
    /* ===== CTRL 组：参考角 / 误差 / 增益 / 电机 / 系统 ===== */
    CH_ROLL_TGT, CH_ROLL_ERR,                          /* 39-40  roll tgt/err */
    CH_PITCH_TGT, CH_PITCH_ERR,                        /* 41-42  pitch tgt/err */
    CH_YAW_TGT, CH_YAW_ERR,                            /* 43-44  yaw tgt/err(6轴,wrap) */
    CH_GAIN_KP, CH_GAIN_KI, CH_GAIN_KD,                /* 45-47  外环增益 */
    CH_MOTA_SPD, CH_MOTA_PWM, CH_MOTA_TGT,             /* 48-50  电机A 速度/PWM/目标 */
    CH_MOTB_SPD, CH_MOTB_PWM, CH_MOTB_TGT,             /* 51-53  电机B */
    CH_LOOP_MS, CH_MODE, CH_RUNNING, CH_STEER,         /* 54-57  周期/模式/运行/转向 */
    CH_HDG_ERR,                                        /* 58     航向误差(wrap, 磁航向→yaw_ref) */
    CH_COUNT                                           /* =59 */
} dbg_chan_t;
_Static_assert(CH_COUNT == DBG_FRAME_N, "DBG_FRAME_N 须 == VOFA 通道数 CH_COUNT");

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
    /* 运行期闸门：遥测帧属 TRACE 级(高吞吐二进制)，仅运行级>=TRACE 才发包；
     * 生产态(默认<=DEBUG)即便编进也不发，UART1 留给文本/命令。TRACE 须先编进二进制
     * (LOG_COMPILE_MAX_LEVEL>=TRACE)，否则永不发；与文本共用 logger_set_level 旋钮。 */
    /* 发包即静音 UART1 文本(<WARN)，留 W/E/F，避免冲波形；
     * 与发包同步：≥TRACE 才静音，回 DEBUG 即恢复；POST 自检在 boot 已打完不受影响。 */
    logger_set_uart1_text_mute_level((logger_get_level() >= LOG_LVL_TRACE)
                                     ? (uint8_t)LOG_LVL_WARN : 0xFFU);

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
    /* ACC 组：raw(LSB÷16384→g) / flt(g) / res(g) */
    const float accel_sf = 1.0f / MPU_ACCEL_LSB_PER_G;   /* 16384 LSB/g */
    const float gyro_sf  = 1.0f / MPU_GYRO_LSB_PER_DPS;  /* 131 LSB/(°/s) */
    float araw[3] = { (float)imu->raw_accel[0] * accel_sf,
                      (float)imu->raw_accel[1] * accel_sf,
                      (float)imu->raw_accel[2] * accel_sf };
    s_frame[CH_ACC_RAW_X] = araw[0]; s_frame[CH_ACC_RAW_Y] = araw[1]; s_frame[CH_ACC_RAW_Z] = araw[2];
    s_frame[CH_ACC_FLT_X] = imu->ax;  s_frame[CH_ACC_FLT_Y] = imu->ay;  s_frame[CH_ACC_FLT_Z] = imu->az;
    s_frame[CH_ACC_RES_X] = araw[0] - imu->ax; s_frame[CH_ACC_RES_Y] = araw[1] - imu->ay; s_frame[CH_ACC_RES_Z] = araw[2] - imu->az;

    /* GYRO 组：raw(LSB÷131→°/s) / flt(°/s) / res(°/s) */
    float graw[3] = { (float)imu->raw_gyro[0] * gyro_sf,
                      (float)imu->raw_gyro[1] * gyro_sf,
                      (float)imu->raw_gyro[2] * gyro_sf };
    s_frame[CH_GYRO_RAW_X] = graw[0]; s_frame[CH_GYRO_RAW_Y] = graw[1]; s_frame[CH_GYRO_RAW_Z] = graw[2];
    s_frame[CH_GYRO_FLT_X] = imu->gx;  s_frame[CH_GYRO_FLT_Y] = imu->gy;  s_frame[CH_GYRO_FLT_Z] = imu->gz;
    s_frame[CH_GYRO_RES_X] = graw[0] - imu->gx; s_frame[CH_GYRO_RES_Y] = graw[1] - imu->gy; s_frame[CH_GYRO_RES_Z] = graw[2] - imu->gz;

    /* MAG 组：raw(counts) / flt(counts) / res(counts) */
    int16_t rm[3]; Attitude_GetRawMag(rm);
    float fm[3];   Attitude_GetFilteredMag(fm);
    s_frame[CH_MAG_RAW_X] = (float)rm[0]; s_frame[CH_MAG_RAW_Y] = (float)rm[1]; s_frame[CH_MAG_RAW_Z] = (float)rm[2];
    s_frame[CH_MAG_FLT_X] = fm[0]; s_frame[CH_MAG_FLT_Y] = fm[1]; s_frame[CH_MAG_FLT_Z] = fm[2];
    s_frame[CH_MAG_RES_X] = (float)rm[0] - fm[0]; s_frame[CH_MAG_RES_Y] = (float)rm[1] - fm[1]; s_frame[CH_MAG_RES_Z] = (float)rm[2] - fm[2];

    /* ATT 组：融合欧拉 / 原始欧拉 / yaw 融合诊断 */
    s_frame[CH_ROLL] = att->roll; s_frame[CH_PITCH] = att->pitch; s_frame[CH_YAW] = att->yaw;
    s_frame[CH_ROLL_PRE] = Attitude_GetRawRoll(); s_frame[CH_PITCH_PRE] = Attitude_GetRawPitch(); s_frame[CH_YAW_PRE] = Attitude_GetRawYaw();
    s_frame[CH_YAW_GYRO]  = Attitude_GetYawGyro();     /* 纯陀螺积分 yaw：不碰磁，直观看漂移 */
    s_frame[CH_MAG_HDG]   = Attitude_GetMagHeading();  /* 倾角补偿磁航向(deg) */
    s_frame[CH_YAW_INNOV] = Attitude_GetYawInnov();   /* yaw 新息(°)：磁校正前误差，即磁拉回量 */

    /* GBIAS 组：在线陀螺零偏估计(°/s) */
    float gb[3]; Attitude_GetGyroBiasEst(gb);
    s_frame[CH_GBIAS_X] = gb[0]; s_frame[CH_GBIAS_Y] = gb[1]; s_frame[CH_GBIAS_Z] = gb[2];

    /* CTRL 组：参考角 / 误差 / 增益 / 航向误差 */
    float kp = 0.0f, ki = 0.0f, kd = 0.0f; Attitude_GetGains(&kp, &ki, &kd);
    s_frame[CH_ROLL_TGT]  = Attitude_GetRollRef();  s_frame[CH_ROLL_ERR]  = Attitude_GetRollErr();
    s_frame[CH_PITCH_TGT] = Attitude_GetPitchRef(); s_frame[CH_PITCH_ERR] = Attitude_GetPitchErr();
    s_frame[CH_YAW_TGT]   = Attitude_GetYawRef();   s_frame[CH_YAW_ERR]   = Attitude_GetYawErr();
    s_frame[CH_GAIN_KP]   = kp; s_frame[CH_GAIN_KI] = ki; s_frame[CH_GAIN_KD] = kd;
    s_frame[CH_HDG_ERR]   = Attitude_GetHeadingErr(Attitude_GetYawRef());
#endif

#if DBG_TELEMETRY_MOTOR
    /* 电机A(48-50) / 电机B(51-53)：速度 / PWM / 目标速度 */
    s_frame[CH_MOTA_SPD] = (float)Motor_GetSpeed(MOTOR_A);
    s_frame[CH_MOTA_PWM] = (float)Motor_GetPWM(MOTOR_A);
    s_frame[CH_MOTA_TGT] = (float)Motor_GetTargetSpeed(MOTOR_A);
    s_frame[CH_MOTB_SPD] = (float)Motor_GetSpeed(MOTOR_B);
    s_frame[CH_MOTB_PWM] = (float)Motor_GetPWM(MOTOR_B);
    s_frame[CH_MOTB_TGT] = (float)Motor_GetTargetSpeed(MOTOR_B);
#endif

#if DBG_TELEMETRY_SYSTEM
    /* 系统(54-57)：循环周期(ms) / 模式 / 运行 / 转向 */
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_last_ms;
    s_last_ms    = now;
    s_frame[CH_LOOP_MS]  = (float)dt;
    s_frame[CH_MODE]     = (float)g_motor_sys.mode;
    s_frame[CH_RUNNING]  = (float)g_motor_sys.running;
    s_frame[CH_STEER]    = (float)Attitude_GetSteer();
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

#endif /* DBG_TELEMETRY_ENABLE && ... */

#else /* 运行级<TRACE：DBG_TELEMETRY_ENABLE 未声明，遥测不编进固件，空实现 */

void Dbg_Telemetry_Init(void) { /* 空（运行级<TRACE，遥测未编进） */ }
void Dbg_Telemetry_Send(const ImuData_t *imu, const Attitude_t *att,
                        int32_t tgtA, int32_t tgtB)
{
    (void)imu; (void)att; (void)tgtA; (void)tgtB;
}

#endif /* defined(DBG_TELEMETRY_ENABLE) */

/* =============================================================================
 * 通道清单（公共区，随 DBG_TELEMETRY_ENABLE==1 编进；否则整段剔除 —— chan 是 TRACE 级命令）
 *   ⚠ 三处一致：本表 = vofa_panel.json = motor_vofa_telemetry.md（name/grp/unit/sem）
 *   sem：RAW/FLT/RES/PRE/TGT/ERR/INNOV/BIAS/GAIN/MOTOR/SYS（ERR* 方向 tgt−mag，与 *_err 相反）
 * =========================================================================== */
#if defined(DBG_TELEMETRY_ENABLE) && (DBG_TELEMETRY_ENABLE == 1)
typedef struct {
    uint8_t     idx;
    const char *name;
    const char *grp;    /* ACC / GYRO / MAG / ATT / GBIAS / CTRL */
    const char *unit;
    const char *sem;    /* 语义标记（见上方） */
} dbg_ch_info_t;

static const dbg_ch_info_t DBG_CH_TBL[DBG_FRAME_N] = {
    /* 顺序须 == 枚举 dbg_chan_t（按类型分组：raw→flt→res，三轴连续）；改后跑 offline_regression.py T1 */
    {0, "raw_ax", "ACC", "g", "RAW"},
    {1, "raw_ay", "ACC", "g", "RAW"},
    {2, "raw_az", "ACC", "g", "RAW"},
    {3, "flt_ax", "ACC", "g", "FLT"},
    {4, "flt_ay", "ACC", "g", "FLT"},
    {5, "flt_az", "ACC", "g", "FLT"},
    {6, "res_ax", "ACC", "g", "RES"},
    {7, "res_ay", "ACC", "g", "RES"},
    {8, "res_az", "ACC", "g", "RES"},
    {9, "raw_gx", "GYRO", "dps", "RAW"},
    {10, "raw_gy", "GYRO", "dps", "RAW"},
    {11, "raw_gz", "GYRO", "dps", "RAW"},
    {12, "flt_gx", "GYRO", "dps", "FLT"},
    {13, "flt_gy", "GYRO", "dps", "FLT"},
    {14, "flt_gz", "GYRO", "dps", "FLT"},
    {15, "res_gx", "GYRO", "dps", "RES"},
    {16, "res_gy", "GYRO", "dps", "RES"},
    {17, "res_gz", "GYRO", "dps", "RES"},
    {18, "raw_mag_x", "MAG", "counts", "RAW"},
    {19, "raw_mag_y", "MAG", "counts", "RAW"},
    {20, "raw_mag_z", "MAG", "counts", "RAW"},
    {21, "flt_mag_x", "MAG", "counts", "FLT"},
    {22, "flt_mag_y", "MAG", "counts", "FLT"},
    {23, "flt_mag_z", "MAG", "counts", "FLT"},
    {24, "res_mag_x", "MAG", "counts", "RES"},
    {25, "res_mag_y", "MAG", "counts", "RES"},
    {26, "res_mag_z", "MAG", "counts", "RES"},
    {27, "roll", "ATT", "deg", "ATT"},
    {28, "pitch", "ATT", "deg", "ATT"},
    {29, "yaw", "ATT", "deg", "ATT"},
    {30, "pre_roll", "ATT", "deg", "PRE"},
    {31, "pre_pitch", "ATT", "deg", "PRE"},
    {32, "pre_yaw", "ATT", "deg", "PRE"},
    {33, "yaw_gyro", "ATT", "deg", "ATT"},
    {34, "mag_hdg", "ATT", "deg", "ATT"},
    {35, "yaw_innov", "ATT", "deg", "INNOV"},
    {36, "gbias_x", "GBIAS", "dps", "BIAS"},
    {37, "gbias_y", "GBIAS", "dps", "BIAS"},
    {38, "gbias_z", "GBIAS", "dps", "BIAS"},
    {39, "roll_tgt", "CTRL", "deg", "TGT"},
    {40, "roll_err", "CTRL", "deg", "ERR"},
    {41, "pitch_tgt", "CTRL", "deg", "TGT"},
    {42, "pitch_err", "CTRL", "deg", "ERR"},
    {43, "yaw_tgt", "CTRL", "deg", "TGT"},
    {44, "yaw_err", "CTRL", "deg", "ERR"},
    {45, "kp", "CTRL", "-", "GAIN"},
    {46, "ki", "CTRL", "-", "GAIN"},
    {47, "kd", "CTRL", "-", "GAIN"},
    {48, "motA_spd", "CTRL", "cnt/tick", "MOTOR"},
    {49, "motA_pwm", "CTRL", "signed", "MOTOR"},
    {50, "motA_tgt", "CTRL", "cnt/tick", "MOTOR"},
    {51, "motB_spd", "CTRL", "cnt/tick", "MOTOR"},
    {52, "motB_pwm", "CTRL", "signed", "MOTOR"},
    {53, "motB_tgt", "CTRL", "cnt/tick", "MOTOR"},
    {54, "loop_ms", "CTRL", "ms", "SYS"},
    {55, "sys_mode", "CTRL", "0/1", "SYS"},
    {56, "sys_running", "CTRL", "0/1", "SYS"},
    {57, "steer", "CTRL", "cnt/tick", "SYS"},
    {58, "hdg_err", "CTRL", "deg", "ERR*"},   /* *=方向 tgt-mag(目标-磁航向)，与 *_err(meas-tgt) 相反 */
};

/* 大小写不敏感比较（组名过滤用） */
static int dbg_strcase_eq(const char *a, const char *b)
{
    if (a == NULL || b == NULL) return 0;
    while (*a != '\0' && *b != '\0') {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= (char)32;
        if (cb >= 'a' && cb <= 'z') cb -= (char)32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return (*a == '\0' && *b == '\0');
}

/* `chan [<组>]` 打印通道清单；grp=NULL/空 打印全部，否则按组名(不区分大小写)过滤 */
void Dbg_Telemetry_PrintChannels(const char *grp)
{
    const char *hdr = "> 通道清单(与 vofa_panel.json 同步): chan <组> 过滤 ACC/GYRO/MAG/ATT/GBIAS/CTRL\r\n";
    BSP_LOG_UART1_SendPoll((const uint8_t *)hdr, (uint16_t)strlen(hdr));
    char line[64];
    for (uint16_t i = 0U; i < (uint16_t)DBG_FRAME_N; i++) {
        const dbg_ch_info_t *c = &DBG_CH_TBL[i];
        if (grp != NULL && grp[0] != '\0' && !dbg_strcase_eq(c->grp, grp))
            continue;
        int n = snprintf(line, sizeof(line),
                         "> CH%u  %-12s [%s,%s] %s\r\n",
                         (unsigned)i, c->name, c->grp, c->unit, c->sem);
        if (n > 0) BSP_LOG_UART1_SendPoll((const uint8_t *)line, (uint16_t)n);
    }
    const char *hint = "> sem: RAW=原始采样 FLT=滤波后 RES=残差(raw-flt) PRE=后滤波前 TGT=目标 ERR=误差(meas-tgt) INNOV=新息(obs-est) BIAS/GAIN/MOTOR/SYS\r\n";
    BSP_LOG_UART1_SendPoll((const uint8_t *)hint, (uint16_t)strlen(hint));
}

#endif /* DBG_TELEMETRY_ENABLE && telemetry on: 通道表查看器(chan) 随遥测特征编进，TRACE 关则剔除 */
