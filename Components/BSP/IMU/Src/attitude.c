/* =============================================================================
 * 姿态解算（Madgwick / 互补） + 两轮自平衡外环控制器
 *   外环 PID：pitch 误差 -> 目标轮速（计数/节拍），交给现有 1kHz 速度PI内环。
 *   依赖 motor.h 的公共 API（Motor_SetSpeed），不直接碰 g_motor[]（static）。
 * =============================================================================
 */
#include "attitude.h"
#include "imu_mpu6050.h"
#include "mag_qmc5883l.h"   /* GY-273 / QMC5883L 磁力计（阶段1 9轴） */
#include "motor.h"
#include "logger.h"
#include <math.h>
#include <string.h>   /* memset: 磁力计滑窗缓存清空 */
#include <stdlib.h>   /* atoi / atof（Attitude_ProcessCommand 解析 T/P/K） */
#include <string.h>    /* strcmp / strncmp / strtof（F 命令滤波配置解析） */
#include <stdint.h>       /* int64_t：Sensor_Test 重力模平方防溢出 */
#include "cmsis_os2.h"    /* osDelay */
#include "iwdg.h"         /* log_wdt_feed：POST 协作喂狗 */
#include "dbg_config.h"   /* DBG_LOG_POSTEST */
#include "app_config.h"   /* APP_ENABLE_SENSOR：Sensor_Test 门控 */

#ifndef RAD2DEG
#define RAD2DEG   57.2957795f
#endif
#ifndef DEG2RAD
#define DEG2RAD   0.0174532925f   /* ARM CLANG math.h 不默认定义 M_PI，用字面量规避 */
#endif

/* ---- 外环控制状态（默认保守增益，靠 VOFA / 命令 K 整定）---- */
static uint8_t  g_enabled   = 0;
static float    g_att_ref[3] = {0.0f, 0.0f, 0.0f};  /* 三轴参考角(deg, = reference setpoint 目标设定值)：[0]roll [1]pitch [2]yaw；pitch 为主平衡轴。
                                                      ⚠ 不是残差！是"目标"，遥测通道层对应 *_tgt（与 motA_tgt 对齐） */
static float    g_kp = 6.0f, g_ki = 0.0f, g_kd = 0.0f, g_k_yaw = 0.0f;  /* g_k_yaw: 航向保持增益，默认0=关 */
static uint8_t  g_yaw_ref_snapped = 0;  /* yaw_ref 是否已在"航向保持 engage 瞬间"对齐到当前航向（防满舵阶跃） */
static int32_t  g_steer     = 0;          /* 转向差，左右轮对称加减 */
static float    g_i_term    = 0.0f;
static float    g_prev_err  = 0.0f;

/* ---- 最新数据（getter / 控制器 / VOFA 共用）---- */
static ImuData_t   s_imu;
static Attitude_t  s_att;
/* 欧拉角后处理低通（默认开，运行时 F elag 命令开关/调参） */
static Attitude_t s_att_raw;                 /* 后滤波前欧拉角（融合直出），供 VOFA CH30-32 对比 */
static float      s_euler_lpf[3] = {0.0f, 0.0f, 0.0f};  /* 欧拉角后滤波状态 */
static uint8_t    s_euler_lag_en    = (uint8_t)EULER_FILTER_LAG_DEFAULT;
static float      s_euler_lag_alpha = EULER_LAG_ALPHA_DEFAULT;
/* yaw 分离式磁融合状态：pitch/roll 来自 6 轴融合，本段只对 yaw 做 陀螺积分+磁互补 */
static float      s_yaw_fused    = 0.0f;   /* 融合 yaw(°)：陀螺积分 + 磁航向低频校正 */
static uint8_t    s_yaw_valid    = 0;      /* 首拍初始化标志（避免从 0 爬升/跳变） */
static float      s_yaw_mag_gain = YAW_MAG_GAIN_DEFAULT;  /* 磁互补增益(0~1) */
/* yaw 融合诊断量（仅供 VOFA 对比，不参与控制）：
   s_yaw_gyro_raw = 纯陀螺积分 yaw(°)，从不碰磁 -> 直接暴露"陀螺在漂"；
   s_yaw_innov    = 磁校正前新息(°) = yaw_wrap_diff(mag_hdg, yaw_fused) -> 即"磁拉回多少"。
   工业 EKF 的新息(residual/innovation) 监控是判断滤波好坏的标准手段，这里把同一概念落到 yaw 上。 */
static float      s_yaw_gyro_raw = 0.0f;
static float      s_yaw_innov    = 0.0f;
static int32_t     g_last_tgtA = 0, g_last_tgtB = 0;

/* 本地自治状态（v4b 框架；FSM 永久接管，无临时门控） */
static uint8_t g_key_stand   = 0;   /* KEY 意图：1=站起 0=坐下 */
static uint8_t g_cloud_stand = 0;   /* 云端 balance_enable=1 意图 */
static uint8_t g_cloud_sit   = 0;   /* 云端 balance_enable=0 意图 */
static uint8_t g_net_online  = 1;   /* 网络在线（默认在线，由 StartNetworkTask 边沿同步：CONNACK OK→1 / 掉线→0） */
static uint8_t g_autonomous  = 0;   /* FSM 当前是否自治使能平衡 */

/* ---- 磁力计状态（9 轴）：阶段1 原始采集 + 占位硬铁偏移；阶段2 做标定持久化 ---- */
static int16_t s_mag_raw[3]   = {0, 0, 0};   /* 原始计数（芯片原生 X/Y/Z） */
static float   s_mag_calib[3] = {0.0f, 0.0f, 0.0f}; /* 轴系对齐(§1.1)后减硬铁偏移（阶段1 offset=0 透传） */
static float   s_mag_heading  = 0.0f;        /* 磁航向(deg)：倾角补偿罗盘(§1.2) */
static float   s_mag_raw_heading = 0.0f;     /* 水平 atan2 磁航向(deg)：未做倾角补偿，供 VOFA/调试对比 */
/* 硬铁偏移占位（默认 0；§1.3 标定后填入，s_mag_calib = align(raw) - offset） */
static float   s_mag_offset[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_mag_ready = 0;
/* 磁力计一阶滞后（默认开，F mlag 运行期开关/调参）：压 QMC 噪声；
   只作用于航向计算向量，s_mag_calib 保持原始供 VOFA CH40-42 对比。 */
static float   s_mag_lpf[3]    = {0.0f, 0.0f, 0.0f};
static uint8_t s_mag_lag_en    = 1;
static float   s_mag_lag_alpha = MAG_LAG_ALPHA_DEFAULT;
static uint8_t s_mag_lpf_valid = 0;   /* 首拍直接捕获，避免从 0 爬行 */
/* 磁力计去极值均值滑窗（F mtrim，默认开，与 accel/gyro 的 TRIM 对称）：
   对原始计数去单点野值/降噪，零相位；s_mag_raw 仍保留原始供 VOFA CH37-39 对比。 */
static int16_t s_mag_win[3][MAG_TRIM_WIN_MAX];  /* 每轴独立环形窗口 */
static uint8_t s_mag_widx[3]   = {0};
static uint8_t s_mag_wcount[3] = {0};
static uint8_t s_mag_trim_en   = 1;
static uint8_t s_mag_trim_win  = MAG_TRIM_WIN_DEFAULT;

/* §1.1 轴系对齐：GY273 与 MPU6050 是两块独立小板，朝向大概率不一致。
   用安装欧拉角(deg)把磁力计芯片原生系旋到 IMU body 系（标准安装：Z 向上、X 向前）。
   默认 0 = 假设两板平行同向粘贴；若绕某轴偏角，调这三个值（M align 命令运行期生效）。
   矩阵由安装欧拉角按 ZYX(Rz*Ry*Rx) 顺序生成，用户只需调 3 个数，不用碰 9 个矩阵元。 */
#ifndef MAG_INSTALL_YAW_DEG
#define MAG_INSTALL_YAW_DEG    0.0f
#endif
#ifndef MAG_INSTALL_PITCH_DEG
#define MAG_INSTALL_PITCH_DEG  0.0f
#endif
#ifndef MAG_INSTALL_ROLL_DEG
#define MAG_INSTALL_ROLL_DEG   0.0f
#endif
static float s_mag_align[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
static float s_mag_install_euler[3] = {MAG_INSTALL_YAW_DEG, MAG_INSTALL_PITCH_DEG, MAG_INSTALL_ROLL_DEG};
static void Attitude_BuildMagAlign(void);   /* 由安装欧拉角重建 s_mag_align（前向声明） */
static int16_t mag_trim_mean(const int16_t *w, uint8_t n);  /* 磁力计滑窗去极值均值（前向声明，供 Attitude_Update 调用） */
static float       s_gyro_offset[3] = {0.0f, 0.0f, 0.0f};  /* 零偏校准(°/s)，Init 阶段计算 */
static float       s_gyro_bias_est[3] = {0.0f, 0.0f, 0.0f}; /* 在线零偏估计(°/s)：静止时代偿陀螺缓慢漂移/坏值 */

/* 互补滤波状态（仅互补后端） */
#if (ATTITUDE_BACKEND == ATTITUDE_BACKEND_COMPLEMENTARY)
static float s_pitch_comp = 0.0f;
#endif

/* 静态陀螺零偏标定：采集 N 拍，要求陀螺“足够安静”（单轴极差 < 阈值）才接受，
   避免上电搬动/放置过程中把运动当成零偏，导致后续静止时仍被积分成大幅漂移/锯齿。
   标定成功后把零偏(°/s)写入 s_gyro_offset，供 Attitude_Update 扣除。 */
#define GYRO_CAL_SAMPLES     100
#define GYRO_STATIONARY_DPS  5.0f    /* 标定期间单轴极差阈值(°/s) */
#define GYRO_BIAS_STILL_DPS  4.0f    /* 在线零偏估计“静止”门限(°/s)：三轴合速率低于此才学零偏，
                                        避免转动时把真实角速度当成零偏吃掉的 E11 现象 */

static int Attitude_CalibrateGyroStatic(void)
{
    int16_t ra[3], rg[3];
    float   gg[3]   = {0.0f, 0.0f, 0.0f};
    float   gmin[3] = { 1e9f,  1e9f,  1e9f};
    float   gmax[3] = {-1e9f, -1e9f, -1e9f};
    int     good = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        if (MPU6050_ReadRaw(ra, rg, NULL, 1) == 0) {   /* 标定路径：阻塞等锁，必须读满 */
            for (int j = 0; j < 3; j++) {
                float v = (float)rg[j];
                gg[j]   += v;
                if (v < gmin[j]) gmin[j] = v;
                if (v > gmax[j]) gmax[j] = v;
            }
            good++;
        }
        HAL_Delay(5);
    }
    if (good < (GYRO_CAL_SAMPLES / 2)) {
        LOG_W("ATT", "CAL FAIL: I2C read only %d/%d ok -> check wiring / keep board still", good, GYRO_CAL_SAMPLES);
        return -1;
    }
    const float inv = 1.0f / (float)good;
    int ok = 1;
    for (int j = 0; j < 3; j++) {
        float range_dps = (gmax[j] - gmin[j]) / MPU_GYRO_LSB_PER_DPS;
        if (range_dps > GYRO_STATIONARY_DPS) {
            LOG_W("ATT", "CAL FAIL: axis%d moved (range %.1f dps > 5.0) -> hold board still",
                  j, (double)range_dps);
            ok = 0;
        }
    }
    if (ok) {
        for (int j = 0; j < 3; j++) {
            s_gyro_offset[j] = (gg[j] * inv) / MPU_GYRO_LSB_PER_DPS;
        }
        LOG_I("ATT", "CAL OK: zero-bias(dps)=%.2f,%.2f,%.2f (subtracted in fusion)",
                  (double)s_gyro_offset[0], (double)s_gyro_offset[1], (double)s_gyro_offset[2]);
    } else {
        LOG_W("ATT", "CAL SKIPPED: board moved during sampling -> keep still, retry C");
    }
    return ok ? 0 : -1;
}

int Attitude_Init(void)
{
    if (MPU6050_Init() != 0) {
        LOG_W("ATT", "MPU6050 init FAIL (check I2C addr / pullup / CubeMX I2C1)");
        return -1;
    }
    Madgwick_Init((float)ATTITUDE_RATE_HZ);
    ImuFilter_Init();
    Attitude_BuildMagAlign();   /* §1.1 由安装欧拉角构建磁力计轴系对齐矩阵 */

    /* 磁力计（GY-273/QMC5883L）：Init 失败不致命。注意：欧拉角融合恒为 6 轴
       (gyro+accel)，磁力计不参与 roll/pitch，仅用于 yaw 分离磁融合(航向锁定)与航向保持。 */
    if (QMC5822_Init() == 0) {
        s_mag_ready = 1;
    } else {
        s_mag_ready = 0;
        LOG_W("ATT", "QMC5883L init FAIL -> mag unavailable: yaw uses gyro integration only (drifts), heading-hold disabled");
    }

    /* 陀螺零偏标定（要求静置，否则拒绝；可用串口命令 C 在静置后重新标定） */
    Attitude_CalibrateGyroStatic();

    /* 静态加速度均值：确认安装方向（重力应落在某一轴 ≈ ±16384 LSB） */
    int16_t ra[3], rg[3];
    float   ga[3] = {0.0f, 0.0f, 0.0f};
    int     good  = 0;
    for (int i = 0; i < 50; i++) {
        if (MPU6050_ReadRaw(ra, rg, NULL, 1) == 0) {   /* Init 路径：阻塞等锁 */
            ga[0] += (float)ra[0]; ga[1] += (float)ra[1]; ga[2] += (float)ra[2];
            good++;
        }
        HAL_Delay(5);
    }
    if (good > 0) {
        const float inv = 1.0f / (float)good;
        for (int i = 0; i < 3; i++) ga[i] *= inv;
    }
    LOG_I("ATT", "static accel(raw)=%.0f,%.0f,%.0f  (gravity axis should be ~ +-%d LSB)",
          (double)ga[0], (double)ga[1], (double)ga[2], (int)MPU_ACCEL_LSB_PER_G);
    LOG_I("ATT", "attitude outer-loop ready (backend=%d rate=%uHz)",
          (int)ATTITUDE_BACKEND, (unsigned)ATTITUDE_RATE_HZ);
    return 0;
}

/* wrap-safe 航向差：返回 (-180,180] 内 target-cur，防 0/360 跳变 */
static float yaw_wrap_diff(float target, float cur)
{
    float e = target - cur;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

void Attitude_Update(const ImuData_t *imu)
{
    s_imu = *imu;

    /* 1) 扣除 Init 阶段标定的零偏（°/s） */
    s_imu.gx -= s_gyro_offset[0];
    s_imu.gy -= s_gyro_offset[1];
    s_imu.gz -= s_gyro_offset[2];

    /* 2) 在线零偏估计：仅当“真正静止”成立时，用慢 leaky 积分把陀螺均值当零偏扣除。
       治“上电搬动导致 Init 标定被拒 / 陀螺带恒定偏置”引起的静止缓慢漂移。
       注意（E11）：静止代理不能只看加速度模——纯旋转不改变 |g|，会让 bias 估计在
       转动时仍更新，把真实角速度当成零偏慢慢扣掉，表现为正常/慢速运动时 gx/gy/gz
       恒为 0。故加角速度门限：三轴合速率 < GYRO_BIAS_STILL_DPS 才算静止。 */
    float amag = sqrtf(s_imu.ax * s_imu.ax + s_imu.ay * s_imu.ay + s_imu.az * s_imu.az);
    int   gyro_bad = (abs((int)imu->raw_gyro[0]) >= 32000)
                   || (abs((int)imu->raw_gyro[1]) >= 32000)
                   || (abs((int)imu->raw_gyro[2]) >= 32000);
    float grate = sqrtf(s_imu.gx * s_imu.gx + s_imu.gy * s_imu.gy + s_imu.gz * s_imu.gz);
    if (!gyro_bad && fabsf(amag - 1.0f) < 0.05f && grate < GYRO_BIAS_STILL_DPS) {
        const float kB = 0.002f;
        s_gyro_bias_est[0] += kB * (s_imu.gx - s_gyro_bias_est[0]);
        s_gyro_bias_est[1] += kB * (s_imu.gy - s_gyro_bias_est[1]);
        s_gyro_bias_est[2] += kB * (s_imu.gz - s_gyro_bias_est[2]);
    }
    s_imu.gx -= s_gyro_bias_est[0];
    s_imu.gy -= s_gyro_bias_est[1];
    s_imu.gz -= s_gyro_bias_est[2];

    /* 3) 陀螺异常剔除：原始计数饱和(≈±250°/s)说明陀螺输出坏值/被冲击，
       该帧不让陀螺参与融合（Madgwick 退化为仅靠加速度求姿态），避免尖峰把
       姿态角瞬间甩到 ±180° 产生周期锯齿。加速度在本工程实测稳定，可作兜底参考。 */
    if (gyro_bad) {
        s_imu.gx = 0.0f;
        s_imu.gy = 0.0f;
        s_imu.gz = 0.0f;
    }

#if (ATTITUDE_BACKEND == ATTITUDE_BACKEND_COMPLEMENTARY)
    /* 互补：加速度算姿态角（无积分漂移），陀螺积分（无高频噪声） */
    float pitch_acc = atan2f(s_imu.ax, s_imu.az) * RAD2DEG;  /* 假设 Z 向上、X 向前 */
    float dt = 1.0f / (float)ATTITUDE_RATE_HZ;
    s_pitch_comp = COMP_ALPHA * (s_pitch_comp + s_imu.gx * dt)
                 + (1.0f - COMP_ALPHA) * pitch_acc;
    s_att_raw.roll  = atan2f(-s_imu.ay, s_imu.az) * RAD2DEG * IMU_PITCH_SIGN;
    s_att_raw.pitch = s_pitch_comp * IMU_PITCH_SIGN;
    /* yaw 不在后端块赋值：统一在下方 yaw 分离式磁融合段处理 */
#else
    /* Madgwick：标准安装（Z 向上、X 向前）直接喂滤波后物理量 */
    Madgwick_UpdateIMU(s_imu.gx, s_imu.gy, s_imu.gz, s_imu.ax, s_imu.ay, s_imu.az);
    s_att_raw.roll  = Madgwick_GetRoll()  * IMU_PITCH_SIGN;
    s_att_raw.pitch = Madgwick_GetPitch() * IMU_PITCH_SIGN;
    /* yaw 不在后端块赋值：统一在下方 yaw 分离式磁融合段处理 */
#endif

    /* 欧拉角后处理低通（elag）已移至下方 mag 块之后，与 yaw 分离融合统一处理 */

    /* 4) 磁力计读取 + 航向（§1.1 轴系对齐 + §1.2 倾角补偿罗盘） */
    if (s_mag_ready) {
        int16_t mr[3];
        if (QMC5822_ReadRaw(mr, 0) == 0) {   /* Attitude_Update 走 Sensor 热路径：抢不到锁即丢一帧 */
            s_mag_raw[0] = mr[0]; s_mag_raw[1] = mr[1]; s_mag_raw[2] = mr[2];
            /* 去极值均值滑窗（F mtrim，默认开）：对原始计数去单点野值/降噪，零相位。
               s_mag_raw 保留原始（供 VOFA CH37-39 对比），trim 结果走对齐/标定。 */
            int16_t mt[3];
            if (s_mag_trim_en) {
                uint8_t w = s_mag_trim_win;
                if (w < 3) w = 3;
                if (w > MAG_TRIM_WIN_MAX) w = MAG_TRIM_WIN_MAX;
                for (int i = 0; i < 3; i++) {
                    int16_t *b = s_mag_win[i];
                    b[s_mag_widx[i]] = mr[i];
                    if (s_mag_wcount[i] < w) s_mag_wcount[i]++;
                    s_mag_widx[i] = (s_mag_widx[i] + 1U) % w;
                    uint8_t n = (s_mag_wcount[i] < w) ? s_mag_wcount[i] : w;
                    mt[i] = mag_trim_mean(b, n);
                }
            } else {
                mt[0] = mr[0]; mt[1] = mr[1]; mt[2] = mr[2];
            }
            /* §1.1 轴系对齐：芯片原生系 -> IMU body 系（先旋转，再减 body 系硬铁偏移） */
            float mb[3];
            mb[0] = s_mag_align[0][0]*(float)mt[0] + s_mag_align[0][1]*(float)mt[1] + s_mag_align[0][2]*(float)mt[2];
            mb[1] = s_mag_align[1][0]*(float)mt[0] + s_mag_align[1][1]*(float)mt[1] + s_mag_align[1][2]*(float)mt[2];
            mb[2] = s_mag_align[2][0]*(float)mt[0] + s_mag_align[2][1]*(float)mt[1] + s_mag_align[2][2]*(float)mt[2];
            /* 减硬铁偏移（阶段1 offset=0，等价透传） */
            s_mag_calib[0] = mb[0] - s_mag_offset[0];
            s_mag_calib[1] = mb[1] - s_mag_offset[1];
            s_mag_calib[2] = mb[2] - s_mag_offset[2];
            /* 一阶滞后：首拍直接捕获，之后指数平滑；关则透传。s_mag_calib 仍原始(供 VOFA CH40-42) */
            if (!s_mag_lpf_valid) {
                s_mag_lpf[0] = s_mag_calib[0];
                s_mag_lpf[1] = s_mag_calib[1];
                s_mag_lpf[2] = s_mag_calib[2];
                s_mag_lpf_valid = 1;
            } else if (s_mag_lag_en) {
                s_mag_lpf[0] += s_mag_lag_alpha * (s_mag_calib[0] - s_mag_lpf[0]);
                s_mag_lpf[1] += s_mag_lag_alpha * (s_mag_calib[1] - s_mag_lpf[1]);
                s_mag_lpf[2] += s_mag_lag_alpha * (s_mag_calib[2] - s_mag_lpf[2]);
            } else {
                s_mag_lpf[0] = s_mag_calib[0];
                s_mag_lpf[1] = s_mag_calib[1];
                s_mag_lpf[2] = s_mag_calib[2];
            }
            /* 水平 atan2（调试对比，无倾角补偿；QMC5883L 新坐标系：-my, mx） */
            s_mag_raw_heading = atan2f(-s_mag_lpf[1], s_mag_lpf[0]) * RAD2DEG;
            if (s_mag_raw_heading < 0.0f) s_mag_raw_heading += 360.0f;
            /* §1.2 倾角补偿罗盘：用融合后的 pitch/roll（s_att_raw 无后滤波延迟）做 tilt 补偿。 */
            float pr = s_att_raw.pitch * DEG2RAD;
            float rr = s_att_raw.roll  * DEG2RAD;
            float mx2 = s_mag_lpf[0]*cosf(pr) + s_mag_lpf[2]*sinf(pr);
            float my2 = s_mag_lpf[0]*sinf(rr)*sinf(pr) + s_mag_lpf[1]*cosf(rr)
                      - s_mag_lpf[2]*sinf(rr)*cosf(pr);
            s_mag_heading = atan2f(-my2, mx2) * RAD2DEG;
            if (s_mag_heading < 0.0f) s_mag_heading += 360.0f;
        }
    }

    /* ---- yaw 分离式磁融合（pitch/roll 已由上 6 轴融合给出，本段只处理 yaw）----
       基础：陀螺 z 积分（短期响应快、无滞后）；
       校正：mag 就绪时用倾角补偿磁航向做低频互补（wrap-safe），长期锁定绝对航向；
       退化：mag 不可用 → 纯陀螺积分（会漂），由 s_mag_ready 自然门控，无需额外分支。 */
    {
        float ydt = 1.0f / (float)ATTITUDE_RATE_HZ;
        if (!s_yaw_valid) {
            s_yaw_fused    = s_mag_ready ? s_mag_heading : 0.0f;  /* 首拍：有磁用磁，无磁从 0 起漂 */
            s_yaw_gyro_raw = s_yaw_fused;                          /* 同起点，便于看 gyro 漂离 */
            s_yaw_valid    = 1;
        }
        /* 纯陀螺积分 yaw（不碰磁）：独立维护一条只积分的链，直接暴露陀螺漂移 */
        s_yaw_gyro_raw += s_imu.gz * ydt;
        while (s_yaw_gyro_raw >  180.0f) s_yaw_gyro_raw -= 360.0f;
        while (s_yaw_gyro_raw < -180.0f) s_yaw_gyro_raw += 360.0f;

        s_yaw_fused += s_imu.gz * ydt;                         /* 陀螺积分预测 */
        float ye = 0.0f;
        if (s_mag_ready) {
            ye = yaw_wrap_diff(s_mag_heading, s_yaw_fused);     /* 磁校正前新息 */
            s_yaw_fused += s_yaw_mag_gain * ye;                /* 磁低频校正 */
        }
        s_yaw_innov = s_mag_ready ? ye : 0.0f;                 /* 新息：磁拉回多少（mag 失能则归零） */
        while (s_yaw_fused >  180.0f) s_yaw_fused -= 360.0f;  /* 归一化 (-180,180] */
        while (s_yaw_fused < -180.0f) s_yaw_fused += 360.0f;
        s_att_raw.yaw = s_yaw_fused;

        /* 欧拉角后处理低通（默认开，F elag 命令运行期开关/调参）：
           压 yaw 磁融合输出毛刺，降低 PID 微分项噪声。s_att_raw 保留滤波前，供 VOFA 对比。 */
        if (s_euler_lag_en) {
            s_euler_lpf[0] += s_euler_lag_alpha * (s_att_raw.roll  - s_euler_lpf[0]);
            s_euler_lpf[1] += s_euler_lag_alpha * (s_att_raw.pitch - s_euler_lpf[1]);
            s_euler_lpf[2] += s_euler_lag_alpha * (s_att_raw.yaw   - s_euler_lpf[2]);
            s_att.roll  = s_euler_lpf[0];
            s_att.pitch = s_euler_lpf[1];
            s_att.yaw   = s_euler_lpf[2];
        } else {
            s_att.roll  = s_att_raw.roll;
            s_att.pitch = s_att_raw.pitch;
            s_att.yaw   = s_att_raw.yaw;
        }
    }
}

float Attitude_GetRoll(void)  { return s_att.roll; }
float Attitude_GetPitch(void) { return s_att.pitch; }
float Attitude_GetYaw(void)   { return s_att.yaw; }
const Attitude_t* Attitude_Get(void) { return &s_att; }

float Attitude_GetRawRoll(void)  { return s_att_raw.roll; }
float Attitude_GetRawPitch(void) { return s_att_raw.pitch; }
float Attitude_GetRawYaw(void)   { return s_att_raw.yaw; }

void Attitude_SetEulerLag(uint8_t en, float alpha)
{
    s_euler_lag_en = en ? 1 : 0;
    if (alpha >= 0.0f && alpha <= 1.0f) s_euler_lag_alpha = alpha;
}
void Attitude_GetEulerLag(uint8_t *en, float *alpha)
{
    if (en) *en = s_euler_lag_en;
    if (alpha) *alpha = s_euler_lag_alpha;
}
/* 去极值均值（与 ImuFilter 的 trim_mean 同构；mag 独立维护，避免跨文件耦合）。
   窗口内去掉一个最大、一个最小后平均；窗口<=2 直接平均。sum 用 int32_t 防御溢出。 */
static int16_t mag_trim_mean(const int16_t *w, uint8_t n)
{
    if (n == 0) return 0;
    int16_t min = w[0], max = w[0];
    int32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (w[i] < min) min = w[i];
        if (w[i] > max) max = w[i];
        sum += w[i];
    }
    if (n <= 2) return (int16_t)(sum / n);
    return (int16_t)((sum - min - max) / (int32_t)(n - 2));
}

void Attitude_SetMagLag(uint8_t en, float alpha)
{
    s_mag_lag_en = en ? 1 : 0;
    if (alpha >= 0.0f && alpha <= 1.0f) s_mag_lag_alpha = alpha;
}
void Attitude_GetMagLag(uint8_t *en, float *alpha)
{
    if (en) *en = s_mag_lag_en;
    if (alpha) *alpha = s_mag_lag_alpha;
}

void Attitude_SetMagTrim(uint8_t en, uint8_t win)
{
    s_mag_trim_en = en ? 1 : 0;
    if (win >= 3 && win <= MAG_TRIM_WIN_MAX) {
        s_mag_trim_win = win;
        /* 窗口变化：清空缓存，避免新旧数据混合 */
        memset(s_mag_win, 0, sizeof(s_mag_win));
        for (int i = 0; i < 3; i++) { s_mag_widx[i] = 0; s_mag_wcount[i] = 0; }
    }
}

void Attitude_GetMagTrim(uint8_t *en, uint8_t *win)
{
    if (en)  *en  = s_mag_trim_en;
    if (win) *win = s_mag_trim_win;
}

void Attitude_SetYawMagGain(float gain)
{
    if (gain >= 0.0f && gain <= 1.0f) s_yaw_mag_gain = gain;
}
float Attitude_GetYawMagGain(void) { return s_yaw_mag_gain; }

/* yaw 融合诊断 getter（仅供 VOFA 对比，不参与控制） */
float Attitude_GetYawGyro(void)  { return s_yaw_gyro_raw; }   /* 纯陀螺积分 yaw(°)：不碰磁，直观看漂移 */
float Attitude_GetYawInnov(void) { return s_yaw_innov; }       /* yaw 新息(°)：磁校正前 yaw_wrap_diff(mag_hdg, fused) */
void  Attitude_GetGyroBiasEst(float b[3])                     /* 在线零偏估计(°/s) */
{ b[0]=s_gyro_bias_est[0]; b[1]=s_gyro_bias_est[1]; b[2]=s_gyro_bias_est[2]; }
void  Attitude_GetGyroOffset(float b[3])                      /* 静态零偏校准(°/s) */
{ b[0]=s_gyro_offset[0];  b[1]=s_gyro_offset[1];  b[2]=s_gyro_offset[2];  }

void Attitude_SetEnable(uint8_t on)
{
    g_enabled = on;
    if (!on) { g_i_term = 0.0f; g_prev_err = 0.0f; }   /* 关闭时清积分/历史，防突跳 */
}
uint8_t Attitude_GetEnable(void) { return g_enabled; }
void Attitude_SetPitchRef(float deg) { g_att_ref[ATT_PITCH] = deg; }
void Attitude_SetRollRef(float deg)  { g_att_ref[ATT_ROLL]  = deg; }
void Attitude_SetYawRef(float deg)   { g_att_ref[ATT_YAW]   = deg; }
void Attitude_SetAttRef(uint8_t axis, float deg) { if (axis <= ATT_YAW) g_att_ref[axis] = deg; }
void Attitude_SetGains(float kp, float ki, float kd) { g_kp = kp; g_ki = ki; g_kd = kd; }
void Attitude_SetSteer(int32_t steer) { g_steer = steer; }
/* 航向保持 engage 防阶跃（E39）：
   yaw_ref 上电默认 0，而 Attitude_GetHeadingErr() 在 mag 就绪时取 s_mag_heading，
   故 hdg_err = 0 - mag_hdg ≈ -66°（实测）。k_yaw 一旦置正，该整段绝对误差会直接
   乘增益变成满舵转向指令（≈ -66×k_yaw，限幅 ±200 直接打满）——这是"开航向保持就抽一下"
   的根因。故在增益由 0 变正的上升沿，把 yaw_ref 对齐到当前航向，使 hdg_err≈0、engage 无冲击。
   基准量与 Attitude_GetHeadingErr 保持一致（mag 就绪用磁航向，否则用融合 yaw），保证 snap 后误差恰为 0。 */
void Attitude_SnapYawRefToCurrent(void)
{
    float cur = s_mag_ready ? s_mag_heading : s_att.yaw;
    g_att_ref[ATT_YAW] = cur;
    g_yaw_ref_snapped  = 1;
    LOG_I("ATT", "yaw_ref snapped to current heading %.2f deg (hdg_err->0, engage without step)",
          (double)cur);
}
uint8_t Attitude_IsYawRefSnapped(void) { return g_yaw_ref_snapped; }

void Attitude_SetHeadingK(float k)
{
    /* 0 -> 正 的上升沿且磁有效：自动对齐 yaw_ref。
       若下发 H 时 mag 尚未就绪，此处不 snap，由 RunController 在 mag 就绪后的首拍补做。 */
    if (k > 0.0f && g_k_yaw <= 0.0f && s_mag_ready) {
        Attitude_SnapYawRefToCurrent();
    }
    if (k <= 0.0f) g_yaw_ref_snapped = 0;   /* 关断后复位：下次再开重新对齐一次 */
    g_k_yaw = k;
}
float Attitude_GetHeadingK(void) { return g_k_yaw; }

void Attitude_GetTargets(int32_t *tgtA, int32_t *tgtB)
{
    if (tgtA) *tgtA = g_last_tgtA;
    if (tgtB) *tgtB = g_last_tgtB;
}

float Attitude_GetPitchRef(void) { return g_att_ref[ATT_PITCH]; }
float Attitude_GetRollRef(void)  { return g_att_ref[ATT_ROLL]; }
float Attitude_GetYawRef(void)   { return g_att_ref[ATT_YAW]; }
float Attitude_GetPitchErr(void) { return s_att.pitch - g_att_ref[ATT_PITCH]; }  /* 与控制器 err 同号：前倾为正 */
float Attitude_GetRollErr(void)  { return s_att.roll  - g_att_ref[ATT_ROLL]; }
float Attitude_GetYawErr(void)
{
    float err = s_att.yaw - g_att_ref[ATT_YAW];
    while (err >  180.0f) err -= 360.0f;   /* wrap-safe：防 0/360 跳变 */
    while (err < -180.0f) err += 360.0f;
    return err;
}
void  Attitude_GetGains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = g_kp;
    if (ki) *ki = g_ki;
    if (kd) *kd = g_kd;
}
int32_t Attitude_GetSteer(void) { return g_steer; }

/* 磁力计 getter（供 VOFA MAG 组 18-26 / CH34 与 M 命令） */
void Attitude_GetRawMag(int16_t mag[3])
{
    mag[0] = s_mag_raw[0]; mag[1] = s_mag_raw[1]; mag[2] = s_mag_raw[2];
}
void Attitude_GetCalibMag(float mag[3])
{
    mag[0] = s_mag_calib[0]; mag[1] = s_mag_calib[1]; mag[2] = s_mag_calib[2];
}
void Attitude_GetFilteredMag(float mag[3])
{
    mag[0] = s_mag_lpf[0]; mag[1] = s_mag_lpf[1]; mag[2] = s_mag_lpf[2];
}
float Attitude_GetMagHeading(void)   { return s_mag_heading; }       /* 倾角补偿后磁航向(deg)，= CH34 */
float Attitude_GetRawMagHeading(void) { return s_mag_raw_heading; }    /* 水平 atan2 磁航向(deg)，未补偿，供对比 */

/* §1.2 倾角补偿罗盘主接口：返回经 tilt 补偿的磁航向(deg)，与 CH34 一致 */
float Attitude_TiltCompassHeading(void) { return s_mag_heading; }

/* 航向误差(wrap-safe)：供 PID 航向控制(g_steer)避免 0/360 跳变。
   优先磁航向(s_mag_heading, 绝对无漂移, =CH34)；mag 不在则退回 6 轴融合 yaw(s_att.yaw, 会漂移)。
   返回 (-180,180]，正=目标在右/逆时针。 */
float Attitude_GetHeadingErr(float target_deg)
{
    float cur = s_mag_ready ? s_mag_heading : s_att.yaw;
    float err = target_deg - cur;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* §1.1 安装欧拉角设置：重建轴系对齐矩阵（运行期 M align 命令调用，不需重烧） */
void Attitude_SetMagAlign(float yaw_deg, float pitch_deg, float roll_deg)
{
    s_mag_install_euler[0] = yaw_deg;
    s_mag_install_euler[1] = pitch_deg;
    s_mag_install_euler[2] = roll_deg;
    Attitude_BuildMagAlign();
    LOG_I("ATT", "mag align(y,p,r)=%.1f,%.1f,%.1f deg applied",
          (double)yaw_deg, (double)pitch_deg, (double)roll_deg);
}
void Attitude_GetMagAlign(float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    if (yaw_deg)   *yaw_deg   = s_mag_install_euler[0];
    if (pitch_deg) *pitch_deg = s_mag_install_euler[1];
    if (roll_deg)  *roll_deg  = s_mag_install_euler[2];
}

/* 由安装欧拉角(ZYX: Rz*Ry*Rx)重建磁力计轴系对齐矩阵 s_mag_align */
static void Attitude_BuildMagAlign(void)
{
    float y = s_mag_install_euler[0] * 0.0174532925f;  /* deg->rad */
    float p = s_mag_install_euler[1] * 0.0174532925f;
    float r = s_mag_install_euler[2] * 0.0174532925f;
    float cy = cosf(y), sy = sinf(y);
    float cp = cosf(p), sp = sinf(p);
    float cr = cosf(r), sr = sinf(r);
    s_mag_align[0][0] = cy*cp;
    s_mag_align[0][1] = cy*sp*sr - sy*cr;
    s_mag_align[0][2] = cy*sp*cr + sy*sr;
    s_mag_align[1][0] = sy*cp;
    s_mag_align[1][1] = sy*sp*sr + cy*cr;
    s_mag_align[1][2] = sy*sp*cr - cy*sr;
    s_mag_align[2][0] = -sp;
    s_mag_align[2][1] = cp*sr;
    s_mag_align[2][2] = cp*cr;
}

/* 运行时重探测磁力计（M 命令调用；换模块/接线后不需重烧） */
int Attitude_MagInit(void)
{
    s_mag_ready = 0;
    s_mag_lpf_valid = 0;   /* 重新探测后下一拍重新捕获首值 */
    memset(s_mag_win, 0, sizeof(s_mag_win));
    for (int i = 0; i < 3; i++) { s_mag_widx[i] = 0; s_mag_wcount[i] = 0; }   /* 清空滑窗，避免旧数据污染 */
    if (QMC5822_Init() == 0) {
        s_mag_ready = 1;
        return 0;
    }
    return -1;
}

/* 长名（域.动作）-> 旧单键 归一化，供 Attitude_ProcessCommand 复用既有解析链（旧键零改动）：
     att.ref 0,-3.6,0 -> P0,-3.6,0     att.pid 6,0,0   -> K6,0,0
     att.on / att.off -> T1 / T0       att.cal/att.dump-> C / D      att.yaw 0.5 -> H0.5
     mag / mag.init / mag.align a,b,c  -> M / M init / M align a,b,c
     filt / filt.<sub> [args]          -> F / F <sub> [args]
   规则：命中前缀 -> 写 repl -> 若前缀后是 '.' 补一个空格 -> 跳过空格 -> 拷剩余参数。
   repl 恒不长于其前缀，输出长度 ≤ 输入，缓冲同尺寸即可。
   边界：前缀后须为 '\0'/空格/Tab/'.'，否则不算命中（避免 "filtx"、"magfoo" 误匹配）。
   无命中则原样拷贝。纯手写拷贝，不依赖 strlen/strncmp/snprintf。 */
static void att_cmd_normalize(const char *in, char *out, size_t osz)
{
    static const struct { const char *pfx; const char *repl; } alias[] = {
        { "att.ref",  "P" },
        { "att.pid",  "K" },
        { "att.on",   "T1" },
        { "att.off",  "T0" },
        { "att.cal",  "C" },
        { "att.dump", "D" },
        { "att.yaw",  "H" },
        { "mag",      "M" },
        { "filt",     "F" },
    };
    if (osz == 0U) return;
    for (size_t i = 0U; i < sizeof(alias) / sizeof(alias[0]); i++) {
        size_t pl = 0U; while (alias[i].pfx[pl] != '\0') pl++;
        size_t k = 0U;  while (k < pl && in[k] == alias[i].pfx[k]) k++;
        if (k != pl) continue;
        char c = in[pl];
        const char *rest;
        if (c == '.')                                   { rest = in + pl + 1; }   /* 跳过 '.'，分隔空格统一由下方补 */
        else if (c == '\0' || c == ' ' || c == '\t')    { rest = in + pl; }
        /* 紧贴参数：数字 / 正负号 / 小数点。旧键 P-3.6 / H0.5 本就无分隔符，
           迁移长名后肌肉记忆会写成 att.ref-3.6 / att.yaw0.5，故一并容忍。 */
        else if (c >= '0' && c <= '9')                  { rest = in + pl; }
        else if ((c == '-' || c == '+') && (in[pl + 1] >= '0' && in[pl + 1] <= '9')) { rest = in + pl; }
        else continue;                       /* 非 token 边界：本项不命中，试下一项 */
        while (*rest == ' ' || *rest == '\t') rest++;
        size_t o = 0U;
        for (size_t j = 0U; alias[i].repl[j] != '\0' && o < osz - 1U; j++) out[o++] = alias[i].repl[j];
        /* 参数非空则补一个分隔空格：mag init -> "M init"（而非 "Minit"）、
           filt lag 1 0.3 -> "F lag 1 0.3"（而非 "Flag 1 0.3"）。
           无参时不补，输出与旧键逐字节相同（att.ref -> "P"）。
           下游各分支本就跳前导空格，不补也能跑，但补了输出才是规范命令串。 */
        if (*rest != '\0' && o < osz - 1U) out[o++] = ' ';
        for (size_t j = 0U; rest[j] != '\0' && o < osz - 1U; j++) out[o++] = rest[j];
        out[o] = '\0';
        return;
    }
    size_t o = 0U;
    for (size_t j = 0U; in[j] != '\0' && o < osz - 1U; j++) out[o++] = in[j];
    out[o] = '\0';
}

/* 串口命令 T/P/K 等解析（由 DbgConsole_Process 路由调用）。
   双轨命名：旧单键 T/P/K/C/F/D/M/H 全部保留，新增 att./mag./filt. 长名经
   att_cmd_normalize() 归一化为单键后复用下方同一条解析链。
   T1/T0 开/关外环；P<r>,<p>,<y> 三轴参考角（或 P<deg> 仅设 pitch）；K<kp>,<ki>,<kd> 在线增益。 */
void Attitude_ProcessCommand(const char *cmd, uint16_t len)
{
    if (cmd == NULL || len == 0U) return;

    char cmdbuf[64];   /* 与控制台命令缓冲同尺寸（.ioc g_cmd_q ItemSize=64） */
    att_cmd_normalize(cmd, cmdbuf, sizeof(cmdbuf));
    cmd = cmdbuf;
    char op = cmd[0];
    if (op == 'T') {
        const char *p = &cmd[1];
        while (*p == ' ' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0') {
            /* 无参 = 查询（公约①）。⚠ 不可落进 atoi("")=0 的旧路径：
               裸 T 会把外环静默关掉，与「无参=查询」的直觉相反（att.on / att.off 才是开关）。 */
            LOG_I("ATT", "outer-loop=%u (att.on / att.off to toggle)", (unsigned)Attitude_GetEnable());
        } else {
            int v = atoi(p);
            Attitude_SetEnable((uint8_t)(v != 0));
            LOG_I("ATT", "attitude outer-loop %s", (v != 0) ? "ON" : "OFF");
        }
    } else if (op == 'P') {
        /* 参考角设置（仿 K<kp>,<ki>,<kd> 风格，统一一条命令，消除 PR/PY 歧义）：
             P<roll>,<pitch>,<yaw>  → 三轴参考角一起设（主平衡轴 pitch + 二次开发 roll/yaw）
             P<deg>                 → 单参兼容：仅设 pitch（原来 P-3.6 仍可用）
             P (无参)              → 打印三轴 ref 与当前 err，确认安装偏角补偿是否生效 */
        const char *p = &cmd[1];
        while (*p == ' ' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0') {
            LOG_I("ATT", "tgt(r,p,y)=%.2f,%.2f,%.2f  err=%.2f,%.2f,%.2f",
                  (double)g_att_ref[ATT_ROLL],  (double)g_att_ref[ATT_PITCH], (double)g_att_ref[ATT_YAW],
                  (double)(s_att.roll  - g_att_ref[ATT_ROLL]),
                  (double)(s_att.pitch - g_att_ref[ATT_PITCH]),
                  (double)(s_att.yaw   - g_att_ref[ATT_YAW]));
        } else {
            float v[3]; int n = 0;
            const char *q = p;
            while (*q && n < 3) {
                char *end;
                float f = strtof(q, &end);
                if (end == q) break;          /* 解析不出数字则停 */
                v[n++] = f; q = end;
                while (*q == ',' || *q == ' ') q++;
            }
            if (n == 1) {
                Attitude_SetPitchRef(v[0]);
                LOG_I("ATT", "pitch_tgt=%.2f deg (single-arg compat)", (double)v[0]);
            } else if (n >= 3) {
                Attitude_SetRollRef(v[0]);
                Attitude_SetPitchRef(v[1]);
                Attitude_SetYawRef(v[2]);
                LOG_I("ATT", "tgt(r,p,y)=%.2f,%.2f,%.2f deg", (double)v[0], (double)v[1], (double)v[2]);
            } else {
                LOG_W("ATT", "P needs 1 or 3 numbers, got %d", n);
            }
        }
    } else if (op == 'K') {
        const char *p = &cmd[1];
        while (*p == ' ' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0') {
            /* 无参 = 查询（公约①）。⚠ 危险修复：旧路径 atof("")==0 会把 kp/ki/kd 全部写成 0，
               输入 att.pid 本意是查增益，结果外环增益清零 -> 姿态环直接失效/失衡。 */
            float g0 = 0.0f, g1 = 0.0f, g2 = 0.0f; Attitude_GetGains(&g0, &g1, &g2);
            LOG_I("ATT", "GAIN: kp=%.2f ki=%.2f kd=%.2f (att.pid kp,ki,kd to set)",
                  (double)g0, (double)g1, (double)g2);
        } else {
            float kp = 0.0f, ki = 0.0f, kd = 0.0f;
            kp = (float)atof(p);
            while (*p && *p != ',') p++;
            if (*p == ',') { ki = (float)atof(p + 1); p++; }   /* 跳过第 1 个逗号 */
            while (*p && *p != ',') p++;
            if (*p == ',') kd = (float)atof(p + 1);
            Attitude_SetGains(kp, ki, kd);
            LOG_I("ATT", "GAIN set: kp=%.2f ki=%.2f kd=%.2f (angle-loop PID)", kp, ki, kd);
        }
    } else if (op == 'C') {
        /* 静置后重新标定陀螺零偏：治“上电搬动导致静止仍漂移/锯齿”。
           标定前确保板子完全不动。 */
        Attitude_CalibrateGyroStatic();
        LOG_I("ATT", "CAL OK: gyro zero-bias recalibrated (applied to fusion)");
    } else if (op == 'D') {
        /* 诊断：打印 MPU6050 寄存器 + 一次 raw 采样，判断陀螺零读数是
           软件配置问题还是硬件损坏。路由方是 StartLoggerTask 的 DbgConsole_Process（非 Motor 任务）。 */
        MPU6050_DumpStatus();
    } else if (op == 'F') {
        /* 滤波运行期配置（无需重烧）：
             F / F print         → 打印当前 物理量LAG/去极值均值/欧拉角后滤波 配置
             F lag 1 [α]         → 物理量一阶滞后开，可选改 α
             F lag 0             → 物理量一阶滞后关
             F trim 1 [win]      → 去极值均值开，可选改窗口
             F trim 0            → 去极值均值关
             F elag 1 [α]        → 欧拉角后滤波开，可选改 α
             F elag 0            → 欧拉角后滤波关
             F mlag 1 [α]        → 磁力计一阶滞后开，可选改 α（仅影响航向计算）
             F mlag 0            → 磁力计一阶滞后关
             F mtrim 1 [win]     → 磁力计去极值均值滑窗开，可选改窗口（3~8）
             F mtrim 0           → 磁力计去极值均值滑窗关
             F yawmag [gain]     → yaw 分离式磁融合增益(0~1)，默认 0.02；越大锁磁越快、抗扰越弱
           例：F elag 1 0.3  F mlag 1 0.15  F mtrim 1 5  F yawmag 0.02  F trim 0  F print */
        const char *q = &cmd[1];
        while (*q == ' ' || *q == '\r' || *q == '\n') q++;
        if (*q == '\0' || strncmp(q, "print", 5) == 0) {
            uint8_t lag_en, trim_en, elag_en; float lag_a, elag_a; uint8_t trim_w;
            ImuFilter_GetCfg(&lag_en, &lag_a, &trim_en, &trim_w);
            Attitude_GetEulerLag(&elag_en, &elag_a);
            uint8_t mag_lag_en; float mag_lag_a; uint8_t mag_trim_en, mag_trim_w;
            Attitude_GetMagLag(&mag_lag_en, &mag_lag_a);
            Attitude_GetMagTrim(&mag_trim_en, &mag_trim_w);
            LOG_I("ATT", "FILT raw_lag=%u a=%.2f | trim=%u win=%u | euler_lag=%u a=%.2f | mag_trim=%u win=%u | mag_lag=%u a=%.2f",
                  (unsigned)lag_en, (double)lag_a, (unsigned)trim_en, (unsigned)trim_w,
                  (unsigned)elag_en, (double)elag_a,
                  (unsigned)mag_trim_en, (unsigned)mag_trim_w,
                  (unsigned)mag_lag_en, (double)mag_lag_a);
            LOG_I("ATT", "FILT yaw-fusion g=%.3f | mag=%u (0=gyro-only drift, 1=mag-locked)",
                  (double)s_yaw_mag_gain, (unsigned)s_mag_ready);
            return;
        }
        char sub[8]; int si = 0;
        while (*q && *q != ' ' && si < 7) sub[si++] = *q++;
        sub[si] = '\0';
        while (*q == ' ' || *q == '\t') q++;
        uint8_t on = 0; float num = 0.0f; int has_num = 0; int has_on = 0;
        if (*q == '1')      { on = 1; has_on = 1; q++; }
        else if (*q == '0') { on = 0; has_on = 1; q++; }
        if (has_on || *q) {
            while (*q == ' ' || *q == '\t') q++;
            char *end; float f = strtof(q, &end);
            if (end != q) { num = f; has_num = 1; }
        }
        if (!has_on && !has_num) {
            LOG_W("ATT", "FILT ERR: %s needs on/off (1|0) [optional param]", sub);
            return;
        }
        if (strcmp(sub, "lag") == 0) {
            float a = has_num ? num : (on ? LAG_ALPHA_DEFAULT : g_imu_filter.lag_alpha);
            ImuFilter_SetLag(on, a);
            LOG_I("ATT", "FILT raw-lag %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "trim") == 0) {
            uint8_t w = has_num ? (uint8_t)num : TRIM_WIN_DEFAULT;
            ImuFilter_SetTrim(on, w);
            LOG_I("ATT", "FILT raw-trim %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "elag") == 0) {
            float a = has_num ? num : (on ? EULER_LAG_ALPHA_DEFAULT : s_euler_lag_alpha);
            Attitude_SetEulerLag(on, a);
            LOG_I("ATT", "FILT euler-lag %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "mlag") == 0) {
            float a = has_num ? num : (on ? MAG_LAG_ALPHA_DEFAULT : s_mag_lag_alpha);
            Attitude_SetMagLag(on, a);
            LOG_I("ATT", "FILT mag-lag %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "mtrim") == 0) {
            uint8_t w = has_num ? (uint8_t)num : MAG_TRIM_WIN_DEFAULT;
            Attitude_SetMagTrim(on, w);
            LOG_I("ATT", "FILT mag-trim %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "yawmag") == 0) {
            float g = has_num ? num : YAW_MAG_GAIN_DEFAULT;
            Attitude_SetYawMagGain(g);
            LOG_I("ATT", "FILT yaw-mag-fusion gain=%.3f", (double)g);
        } else {
            LOG_W("ATT", "FILT ERR: unknown subcmd %s (use lag/trim/elag/mlag/mtrim/yawmag/print)", sub);
        }
    } else if (op == 'H') {
        /* 航向保持增益（磁门控）：H<k_yaw> 设增益（0=关闭，走外部 steer）。
           mag 不在时即便 H>0 也无效（RunController 内跳过，降级6轴）。 */
        const char *p = &cmd[1];
        while (*p == ' ' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0') {
            LOG_I("ATT", "HED: k_yaw=%.2f | mag=%s", (double)g_k_yaw, s_mag_ready ? "on" : "off");
        } else {
            float k = strtof(p, NULL);
            Attitude_SetHeadingK(k);
            LOG_I("ATT", "HED: k_yaw=%.2f -> %s", (double)k,
                  (k > 0.0f && s_mag_ready) ? "yaw-hold ACTIVE" : "off: need mag ready + k>0");
        }
    } else if (op == 'M') {
        /* 磁力计诊断/轴系对齐/重探测（9 轴）：
             M                  → 打印状态 + 一次采样（tilt 补偿航向 vs 水平航向 对比 + 安装角）
             M init             → 重探测 QMC5883L（换模块/接线后不需重烧）
             M align <y>,<p>,<r> → 设安装欧拉角(deg)并重建对齐矩阵（绕竖轴偏调 yaw，
                                    板贴反调 180；不需重烧） */
        const char *q = &cmd[1];
        while (*q == ' ' || *q == '\r' || *q == '\n') q++;
        if (strncmp(q, "init", 4) == 0) {
            int r = Attitude_MagInit();
            LOG_I("ATT", "MAG init: %s", (r == 0) ? "OK" : "FAIL (no I2C ACK?)");
        } else if (strncmp(q, "align", 5) == 0) {
            const char *a = q + 5;
            while (*a == ' ' || *a == ',') a++;
            float v[3]; int n = 0;
            const char *qq = a;
            while (*qq && n < 3) {
                char *end; float f = strtof(qq, &end);
                if (end == qq) break;
                v[n++] = f; qq = end;
                while (*qq == ',' || *qq == ' ') qq++;
            }
            if (n >= 3) Attitude_SetMagAlign(v[0], v[1], v[2]);
            else LOG_W("ATT", "M align needs 3 numbers: yaw,pitch,roll (deg)");
        }
        QMC5822_DumpStatus(1);   /* M 命令在 Logger 上下文：阻塞等锁跑完 */
        LOG_I("ATT", "mag ready=%u raw=%d,%d,%d calib=%.1f,%.1f,%.1f",
              (unsigned)s_mag_ready,
              (int)s_mag_raw[0], (int)s_mag_raw[1], (int)s_mag_raw[2],
              (double)s_mag_calib[0], (double)s_mag_calib[1], (double)s_mag_calib[2]);
        LOG_I("ATT", "heading(tilt-comp)=%.1f | heading(raw,no-tilt)=%.1f | align(y,p,r)=%.1f,%.1f,%.1f deg",
              (double)s_mag_heading, (double)s_mag_raw_heading,
              (double)s_mag_install_euler[0], (double)s_mag_install_euler[1], (double)s_mag_install_euler[2]);
        LOG_I("ATT", "verify: rotate board about Z 360deg -> tilt-comp heading should go 0->360 monotonic (no jump)");
    } else if (strncmp(cmd, "att", 3) == 0 && (cmd[3] == '\0' || cmd[3] == ' ')) {
        /* att（无参）= 查询（公约①）：姿态域概览。
           注：att.<sub> 各子命令已在 att_cmd_normalize() 归一化为 P/K/T/C/D/H，不会落到这里。 */
        float kp = 0.0f, ki = 0.0f, kd = 0.0f; Attitude_GetGains(&kp, &ki, &kd);
        float bias[3] = {0.0f, 0.0f, 0.0f};    Attitude_GetGyroOffset(bias);
        LOG_I("ATT", "ATT: enable=%u | tgt(r,p,y)=%.2f,%.2f,%.2f | err=%.2f,%.2f,%.2f",
              (unsigned)Attitude_GetEnable(),
              (double)g_att_ref[ATT_ROLL],  (double)g_att_ref[ATT_PITCH], (double)g_att_ref[ATT_YAW],
              (double)(s_att.roll  - g_att_ref[ATT_ROLL]),
              (double)(s_att.pitch - g_att_ref[ATT_PITCH]),
              (double)(s_att.yaw   - g_att_ref[ATT_YAW]));
        LOG_I("ATT", "ATT: pid=%.2f,%.2f,%.2f | k_yaw=%.2f | gyro_bias=%.2f,%.2f,%.2f dps | mag=%u hdg=%.1f",
              (double)kp, (double)ki, (double)kd, (double)Attitude_GetHeadingK(),
              (double)bias[0], (double)bias[1], (double)bias[2],
              (unsigned)s_mag_ready, (double)Attitude_GetMagHeading());
    } else {
        /* 未知子命令：明确报错，不再静默丢弃（E41 教训）。
           能走到这里的只可能是未识别的 att.*mag.*filt.*，旧单键已被上面各分支覆盖。 */
        LOG_W("ATT", "unknown att cmd: %s (try: att.ref/pid/on/off/cal/dump/yaw | mag | filt)", cmd);
    }
}

void Attitude_RunController(void)
{
    if (!g_enabled) return;

    /* 倾角误差：前倾(pitch↑)为正 → u 正 → 轮子前进“追”重心扶直。
       必须用 pitch - ref；写成 ref - pitch 会变正反馈（越倒越快）。 */
    float pitch = s_att.pitch;
    float err   = pitch - g_att_ref[ATT_PITCH];
    float rate  = s_imu.gx * IMU_PITCH_SIGN;     /* 前向轴角速度 deg/s，与 pitch 同向 */

    g_i_term += g_ki * err;
    if (g_i_term >  200.0f) g_i_term =  200.0f;  /* 积分限幅，防 windup */
    if (g_i_term < -200.0f) g_i_term = -200.0f;

    float u = g_kp * err + g_i_term - g_kd * rate;

    /* 输出限幅到内环合理范围（计数/节拍；TARGET_SPEED_MAX=200） */
    if (u >  200.0f) u =  200.0f;
    if (u < -200.0f) u = -200.0f;

    int32_t base = (int32_t)u;
    /* 航向保持（磁门控，默认 k_yaw=0 关闭）：用 wrap-safe 磁航向误差驱动转向差，
       抵抗 yaw 扰动、保持/转到 yaw_ref；mag 不在(降级6轴)时跳过，g_steer 退回外部命令值。
       限幅 ±200 与内环一致。 */
    if (s_mag_ready && g_k_yaw > 0.0f) {
        /* 兜底：H 命令可能在 mag 就绪前下发（彼时无有效航向可对齐），
           故在此补做一次 snap——磁一就绪就把 yaw_ref 拉到当前航向，engage 无阶跃。 */
        if (!g_yaw_ref_snapped) Attitude_SnapYawRefToCurrent();
        g_steer = (int32_t)(g_k_yaw * Attitude_GetHeadingErr(g_att_ref[ATT_YAW]));
        if (g_steer >  200) g_steer =  200;
        if (g_steer < -200) g_steer = -200;
    }
    /* 仅当电机运行（g_motor_sys.running）内环才真正输出；这里只写目标 */
    Motor_SetSpeed(MOTOR_A, base + g_steer);
    Motor_SetSpeed(MOTOR_B, base - g_steer);
    g_last_tgtA = base + g_steer;
    g_last_tgtB = base - g_steer;
}

/* =============================================================================
 * 本地自治（v4b 框架，详见 Components/Motor/balance_autonomy_plan.md）
 *   FSM 永久接管使能决策：AutonomyTick 周期执行，仲裁平衡使能与电机运行态。
 * ============================================================================= */

/* 传感器/姿态健康代理：加速度模接近 1g 且三轴陀螺原始计数未饱和（±32000） */
uint8_t Attitude_IsHealthy(void)
{
    float amag = sqrtf(s_imu.ax*s_imu.ax + s_imu.ay*s_imu.ay + s_imu.az*s_imu.az);
    int gyro_bad = (abs((int)s_imu.raw_gyro[0]) >= 32000)
                 || (abs((int)s_imu.raw_gyro[1]) >= 32000)
                 || (abs((int)s_imu.raw_gyro[2]) >= 32000);
    if (gyro_bad) return 0;
    if (fabsf(amag - 1.0f) > 0.3f) return 0;
    return 1;
}

void Attitude_SetKeyStand(uint8_t req)
{
    g_key_stand = req ? 1U : 0U;
}
void Attitude_SetCloudStand(uint8_t req)
{
    g_cloud_stand = req ? 1U : 0U;
}
void Attitude_SetCloudSit(uint8_t req)
{
    g_cloud_sit = req ? 1U : 0U;
}
void Attitude_SetNetOnline(uint8_t on)
{
    g_net_online = on ? 1U : 0U;
    if (on) {
        /* 恢复在线：锁存的云端意图(g_cloud_stand/g_cloud_sit)将自动重新生效 */
        LOG_I("ATT", "net ONLINE: cloud intent re-enabled (latched cloud_stand=%u cloud_sit=%u)",
              g_cloud_stand, g_cloud_sit);
    }
}

void Attitude_AutonomyTick(void)
{
    /* D1 传感器健康：失效则强制急停 */
    if (!Attitude_IsHealthy()) {
        if (g_autonomous || g_enabled) {
            Attitude_SetEnable(0);
            g_autonomous = 0;
            Motor_EmergencyStop();
        }
        return;
    }

    /* 当前最大倾角（俯仰/横滚绝对值） */
    float tilt = fabsf(s_att.pitch);
    if (fabsf(s_att.roll) > tilt) tilt = fabsf(s_att.roll);

    /* D2 倾覆急停 */
    if (tilt > ATT_TILT_OVERTURN_DEG) {
        if (g_autonomous || g_enabled) {
            Attitude_SetEnable(0);
            g_autonomous = 0;
            Motor_EmergencyStop();
        }
        return;
    }

    /* 意图汇总：仅门控"意图来源(KEY/云端)"——决定谁允许触发站起/坐下，不触及任何平衡计算。
       网络状态(D3)只在此层生效：离线时 cloud_can_drive=0 → 仅 KEY 可驱动、云端 stand/sit 锁存(值保留,
       恢复在线后自动重新生效)；姿态外环(平衡)由运动控制律独立处理，本函数不耦合网络状态与平衡。 */
    uint8_t cloud_can_drive = g_net_online;   /* 离线→仅 KEY 可驱动；云端意图锁存(值保留) */
    uint8_t want_up = (g_key_stand || (cloud_can_drive && g_cloud_stand)) ? 1U : 0U;
    if (g_cloud_sit && cloud_can_drive) want_up = 0U;   /* 云端坐下也仅在在线时生效 */

    if (want_up) {
        /* D4/D5 站起请求 + 直立且就绪 → ENGAGE */
        if (tilt <= ATT_TILT_STAND_READY_DEG) {
            if (!g_autonomous) {
                Motor_Resume();            /* 确保电机解除刹车 */
                Attitude_SetEnable(1);
                g_autonomous = 1;
                LOG_I("ATT", "AUTONOMY ENGAGE (key=%u cloud=%u)", g_key_stand, g_cloud_stand);
            }
        } else {
            /* 倾角过大：拒绝站起，保持安全锁（已在平衡中则维持） */
            LOG_W("ATT", "stand rejected: tilt=%.1f > %.1f", (double)tilt, (double)ATT_TILT_STAND_READY_DEG);
        }
    } else {
        /* D6/D7 坐下请求 + 接近安全姿态 → 优雅放下 */
        if (g_autonomous) {
            if (tilt <= ATT_TILT_SIT_SAFE_DEG) {
                Attitude_SetEnable(0);
                g_autonomous = 0;
                Motor_EmergencyStop();
                LOG_I("ATT", "AUTONOMY graceful sit");
            } else {
                LOG_W("ATT", "sit rejected: tilt=%.1f > %.1f", (double)tilt, (double)ATT_TILT_SIT_SAFE_DEG);
            }
        }
    }

    /* D3 断网保护已落到上方意图汇总（line 634-637）：离线时 cloud_can_drive=0 → 仅 KEY 可驱动、
       云端 stand/sit 锁存(值保留, 恢复后自动生效)。本分支只门控"意图来源(KEY/云端)"，不触碰平衡计算——
       姿态外环尚未实现，平衡控制律独立处理，网络状态与平衡解耦。
       g_net_online 由 StartNetworkTask 边沿同步（CONNACK OK→1 / 掉线/超时→0，幂等边沿触发）。 */
}

/* ===================== [迁移] 传感器模块自检：从 selftest.c 下沉到本组件（按 APP_ENABLE_SENSOR 门控） ===================== */
#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
int Sensor_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Sensor_Test enter");
#endif
    log_wdt_feed();
    /* MPU6050 已由 main.c 的 Attitude_Init() 在调度器启动前初始化；
       此处做数据合理性校验（不重新 Init，避免与融合任务抢 I2C）。 */
    int rc_mpu = 0;
    int good = 0;
    /* 复检 WHO_AM_I：Attitude_Init 已校验过，此处再确认芯片在位（I2C 总线未掉/未掉电） */
    uint8_t mpu_id = 0;
    if (MPU6050_ReadWhoAmI(&mpu_id) != 0 ||
        (mpu_id != 0x68 && mpu_id != 0x70)) {
        LOG_E("POSTEST", "Sensor MPU6050 WHO_AM_I=0x%02X unexpected/missing", (unsigned)mpu_id);
        return -4;
    }
    int16_t accel[3], gyro[3], temp;
    for (int i = 0; i < 8; i++) {
        if (MPU6050_ReadRaw(accel, gyro, &temp, 1) != 0) { rc_mpu = -1; break; }   /* Sensor_Test：阻塞等锁 */
        /* 单样本加速度模平方（int64 防溢出）：0.7g~1.3g => s 落在 [131.5e6, 453.7e6] */
        int64_t s = (int64_t)accel[0]*accel[0] + (int64_t)accel[1]*accel[1]
                  + (int64_t)accel[2]*accel[2];
        if (s >= 131533373LL && s <= 453656121LL) good++;
        /* 陀螺任一轴接近饱和（±31000）判失败 */
        if (gyro[0] < -31000 || gyro[0] > 31000 ||
            gyro[1] < -31000 || gyro[1] > 31000 ||
            gyro[2] < -31000 || gyro[2] > 31000) { rc_mpu = -3; break; }
        osDelay(2); log_wdt_feed();
    }
    if (rc_mpu == 0 && good == 0) rc_mpu = -2;   /* 8 帧无一在合理重力范围 */
    (void)temp;   /* 仅作 ReadRaw 输出参数，未参与判定 */

    if (rc_mpu != 0) {
        LOG_E("POSTEST", "Sensor MPU6050 data invalid rc=%d (chk whoami/init)", rc_mpu);
    } else {
        LOG_I("POSTEST", "Sensor MPU6050 data OK");
    }

    /* 磁力计 QMC5883L：非致命校验 ---  缺磁不影响站立，yaw 可走陀螺积分  ---  */
    {
        int bypass = MPU6050_IsBypassEnabled();
        if (bypass > 0)
            LOG_I("POSTEST", "Sensor QMC bypass: ENABLED (aux bus reachable via PB8/PB9)");
        else if (bypass == 0)
            LOG_W("POSTEST", "Sensor QMC bypass: DISABLED! aux bus unreachable -> mag cannot start");
        else
            LOG_W("POSTEST", "Sensor QMC bypass: read-back NACK (cannot verify, check I2C)");

        /* 启动期 QMC5822_Init 可能早于 aux-bus 旁路就绪而失败，且其日志在调度器
           启动前被 ring buffer 冲掉、此处看不到。旁路已确认 ENABLED 时，这里重新
           Init 自愈一次，并 DumpStatus 打印寄存器状态便于定位；旁路未开则跳过重初始化。
           无论哪种路径，最终都同步融合层门控标志 s_mag_ready，确保 yaw 磁校正可用。 */
        if (!QMC5822_IsReady()) {
            if (bypass > 0) {
                LOG_W("POSTEST", "Sensor QMC NOT ready at boot -> re-init now (bypass ENABLED)");
                QMC5822_Init();
            } else {
                LOG_W("POSTEST", "Sensor QMC NOT ready, bypass DISABLED -> skip re-init (check XCL/XDA wiring)");
            }
            QMC5822_DumpStatus(1);   /* SelfTest：阻塞等锁 */
            MPU6050_ScanBus();   /* 决定性取证：打出 aux 总线上所有 ACK 的从机地址，判定 QMC 是否真在总线上 */
        }

        if (QMC5822_IsReady()) {
            s_mag_ready = 1;   /* 融合层门控：yaw 走磁航向低频校正（航向锁定） */
            int16_t mag[3];
            if (QMC5822_ReadRaw(mag, 1) == 0) {   /* SelfTest：阻塞等锁 */
                int64_t mm = (int64_t)mag[0]*mag[0] + (int64_t)mag[1]*mag[1]
                           + (int64_t)mag[2]*mag[2];
                if (mm == 0 || mm > (int64_t)4000*4000)
                    LOG_W("POSTEST", "Sensor QMC raw suspicious X=%d Y=%d Z=%d |mag|=%.0f counts (non-fatal)",
                          (int)mag[0], (int)mag[1], (int)mag[2], (double)sqrtf((float)mm));
                else
                    LOG_I("POSTEST", "Sensor QMC mag OK  raw(X,Y,Z)=%d,%d,%d  |mag|=%.0f counts",
                          (int)mag[0], (int)mag[1], (int)mag[2], (double)sqrtf((float)mm));
            } else {
                LOG_W("POSTEST", "Sensor QMC read failed, non-fatal");
            }
        } else {
            s_mag_ready = 0;   /* 融合层门控：退化为纯陀螺积分（yaw 漂移） */
            LOG_W("POSTEST", "Sensor QMC still NOT ready after re-init (non-fatal; yaw via gyro integration)");
        }
    }
    LOG_I("POSTEST", "Sensor QMC check done (non-fatal; bypass/mag status above this line)");
    return rc_mpu;   /* 仅 MPU 失败才触发关键 halt */
}
#endif
