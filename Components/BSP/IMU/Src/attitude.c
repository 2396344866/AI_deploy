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
#include <stdlib.h>   /* atoi / atof（Attitude_ProcessCommand 解析 T/P/K） */
#include <string.h>    /* strcmp / strncmp / strtof（F 命令滤波配置解析） */

#ifndef RAD2DEG
#define RAD2DEG   57.2957795f
#endif
#ifndef DEG2RAD
#define DEG2RAD   0.0174532925f   /* ARM CLANG math.h 不默认定义 M_PI，用字面量规避 */
#endif

/* ---- 外环控制状态（默认保守增益，靠 VOFA / 命令 K 整定）---- */
static uint8_t  g_enabled   = 0;
static float    g_att_ref[3] = {0.0f, 0.0f, 0.0f};  /* 三轴参考角(deg)：[0]roll [1]pitch [2]yaw；pitch 为主平衡轴 */
static float    g_kp = 6.0f, g_ki = 0.0f, g_kd = 0.0f;
static int32_t  g_steer     = 0;          /* 转向差，左右轮对称加减 */
static float    g_i_term    = 0.0f;
static float    g_prev_err  = 0.0f;

/* ---- 最新数据（getter / 控制器 / VOFA 共用）---- */
static ImuData_t   s_imu;
static Attitude_t  s_att;
/* 欧拉角后处理低通（默认开，运行时 F elag 命令开关/调参） */
static Attitude_t s_att_raw;                 /* 后滤波前欧拉角（融合直出），供 VOFA CH34-36 对比 */
static float      s_euler_lpf[3] = {0.0f, 0.0f, 0.0f};  /* 欧拉角后滤波状态 */
static uint8_t    s_euler_lag_en    = (uint8_t)EULER_FILTER_LAG_DEFAULT;
static float      s_euler_lag_alpha = EULER_LAG_ALPHA_DEFAULT;
static int32_t     g_last_tgtA = 0, g_last_tgtB = 0;

/* ---- 磁力计状态（9 轴）：阶段1 原始采集 + 占位硬铁偏移；阶段2 做标定持久化 ---- */
static int16_t s_mag_raw[3]   = {0, 0, 0};   /* 原始计数（芯片原生 X/Y/Z） */
static float   s_mag_calib[3] = {0.0f, 0.0f, 0.0f}; /* 轴系对齐(§1.1)后减硬铁偏移（阶段1 offset=0 透传） */
static float   s_mag_heading  = 0.0f;        /* 磁航向(deg)：倾角补偿罗盘(§1.2) */
static float   s_mag_raw_heading = 0.0f;     /* 水平 atan2 磁航向(deg)：未做倾角补偿，供 VOFA/调试对比 */
/* 硬铁偏移占位（默认 0；§1.3 标定后填入，s_mag_calib = align(raw) - offset） */
static float   s_mag_offset[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_mag_ready = 0;

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
        if (MPU6050_ReadRaw(ra, rg, NULL) == 0) {
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
        LOG_W("ATT", "gyro cal skipped: only %d/%d samples ok", good, GYRO_CAL_SAMPLES);
        return -1;
    }
    const float inv = 1.0f / (float)good;
    int ok = 1;
    for (int j = 0; j < 3; j++) {
        float range_dps = (gmax[j] - gmin[j]) / MPU_GYRO_LSB_PER_DPS;
        if (range_dps > GYRO_STATIONARY_DPS) {
            LOG_W("ATT", "gyro cal rejected: axis%d not still (range=%.1f dps)",
                  j, (double)range_dps);
            ok = 0;
        }
    }
    if (ok) {
        for (int j = 0; j < 3; j++) {
            s_gyro_offset[j] = (gg[j] * inv) / MPU_GYRO_LSB_PER_DPS;
        }
        LOG_I("ATT", "gyro offset(dps)=%.2f,%.2f,%.2f",
              (double)s_gyro_offset[0], (double)s_gyro_offset[1], (double)s_gyro_offset[2]);
    } else {
        LOG_W("ATT", "gyro cal SKIPPED (board not still during cal)");
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

    /* 磁力计（GY-273/QMC5883L，阶段1 9轴）：Init 失败不致命，仅降级为 6 轴融合 */
    if (QMC5822_Init() == 0) {
        s_mag_ready = 1;
    } else {
        s_mag_ready = 0;
        LOG_W("ATT", "QMC5883L init FAIL -> degrade 9-axis to 6-axis (yaw relies on gyro integration, will drift)");
    }

    /* 陀螺零偏标定（要求静置，否则拒绝；可用串口命令 C 在静置后重新标定） */
    Attitude_CalibrateGyroStatic();

    /* 静态加速度均值：确认安装方向（重力应落在某一轴 ≈ ±16384 LSB） */
    int16_t ra[3], rg[3];
    float   ga[3] = {0.0f, 0.0f, 0.0f};
    int     good  = 0;
    for (int i = 0; i < 50; i++) {
        if (MPU6050_ReadRaw(ra, rg, NULL) == 0) {
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
    s_att_raw.yaw   = 0.0f;
#else
    /* Madgwick：标准安装（Z 向上、X 向前）直接喂滤波后物理量 */
    Madgwick_UpdateIMU(s_imu.gx, s_imu.gy, s_imu.gz, s_imu.ax, s_imu.ay, s_imu.az);
    s_att_raw.roll  = Madgwick_GetRoll()  * IMU_PITCH_SIGN;
    s_att_raw.pitch = Madgwick_GetPitch() * IMU_PITCH_SIGN;
    s_att_raw.yaw   = Madgwick_GetYaw();
#endif

    /* 欧拉角后处理低通（默认开，F elag 命令运行期开关/调参）：
       压融合输出毛刺，降低 PID 微分项噪声。s_att_raw 保留滤波前，供 VOFA 对比。 */
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

    /* 4) 磁力计读取 + 航向（§1.1 轴系对齐 + §1.2 倾角补偿罗盘） */
    if (s_mag_ready) {
        int16_t mr[3];
        if (QMC5822_ReadRaw(mr) == 0) {
            s_mag_raw[0] = mr[0]; s_mag_raw[1] = mr[1]; s_mag_raw[2] = mr[2];
            /* §1.1 轴系对齐：芯片原生系 -> IMU body 系（先旋转，再减 body 系硬铁偏移） */
            float mb[3];
            mb[0] = s_mag_align[0][0]*(float)mr[0] + s_mag_align[0][1]*(float)mr[1] + s_mag_align[0][2]*(float)mr[2];
            mb[1] = s_mag_align[1][0]*(float)mr[0] + s_mag_align[1][1]*(float)mr[1] + s_mag_align[1][2]*(float)mr[2];
            mb[2] = s_mag_align[2][0]*(float)mr[0] + s_mag_align[2][1]*(float)mr[1] + s_mag_align[2][2]*(float)mr[2];
            /* 减硬铁偏移（阶段1 offset=0，等价透传） */
            s_mag_calib[0] = mb[0] - s_mag_offset[0];
            s_mag_calib[1] = mb[1] - s_mag_offset[1];
            s_mag_calib[2] = mb[2] - s_mag_offset[2];
            /* 水平 atan2（调试对比，无倾角补偿；QMC5883L 新坐标系：-my, mx） */
            s_mag_raw_heading = atan2f(-s_mag_calib[1], s_mag_calib[0]) * RAD2DEG;
            if (s_mag_raw_heading < 0.0f) s_mag_raw_heading += 360.0f;
            /* §1.2 倾角补偿罗盘：用融合后的 pitch/roll（s_att_raw 无后滤波延迟）做 tilt 补偿，
               消除倾斜时机标偏 90° 导致的航向误差。公式见待处理.md §1.2。 */
            float pr = s_att_raw.pitch * DEG2RAD;
            float rr = s_att_raw.roll  * DEG2RAD;
            float mx2 = s_mag_calib[0]*cosf(pr) + s_mag_calib[2]*sinf(pr);
            float my2 = s_mag_calib[0]*sinf(rr)*sinf(pr) + s_mag_calib[1]*cosf(rr)
                      - s_mag_calib[2]*sinf(rr)*cosf(pr);
            s_mag_heading = atan2f(-my2, mx2) * RAD2DEG;
            if (s_mag_heading < 0.0f) s_mag_heading += 360.0f;
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
float Attitude_GetYawErr(void)   { return s_att.yaw   - g_att_ref[ATT_YAW]; }
void  Attitude_GetGains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = g_kp;
    if (ki) *ki = g_ki;
    if (kd) *kd = g_kd;
}
int32_t Attitude_GetSteer(void) { return g_steer; }

/* 磁力计 getter（供 VOFA CH37-43 与 M 命令） */
void Attitude_GetRawMag(int16_t mag[3])
{
    mag[0] = s_mag_raw[0]; mag[1] = s_mag_raw[1]; mag[2] = s_mag_raw[2];
}
void Attitude_GetCalibMag(float mag[3])
{
    mag[0] = s_mag_calib[0]; mag[1] = s_mag_calib[1]; mag[2] = s_mag_calib[2];
}
float Attitude_GetMagHeading(void)   { return s_mag_heading; }       /* 倾角补偿后磁航向(deg)，= CH43 */
float Attitude_GetRawMagHeading(void) { return s_mag_raw_heading; }    /* 水平 atan2 磁航向(deg)，未补偿，供对比 */

/* §1.2 倾角补偿罗盘主接口：返回经 tilt 补偿的磁航向(deg)，与 CH43 一致 */
float Attitude_TiltCompassHeading(void) { return s_mag_heading; }

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
    if (QMC5822_Init() == 0) {
        s_mag_ready = 1;
        return 0;
    }
    return -1;
}

/* 串口命令 T/P/K 解析（由 StartMotorTask 路由调用）。
   T1/T0 开/关外环；P<r>,<p>,<y> 三轴参考角（或 P<deg> 仅设 pitch）；K<kp>,<ki>,<kd> 在线增益。 */
void Attitude_ProcessCommand(const char *cmd, uint16_t len)
{
    if (cmd == NULL || len == 0U) return;
    char op = cmd[0];
    if (op == 'T') {
        int v = atoi(&cmd[1]);
        Attitude_SetEnable((uint8_t)(v != 0));
        LOG_I("ATT", "attitude outer-loop %s", (v != 0) ? "ON" : "OFF");
    } else if (op == 'P') {
        /* 参考角设置（仿 K<kp>,<ki>,<kd> 风格，统一一条命令，消除 PR/PY 歧义）：
             P<roll>,<pitch>,<yaw>  → 三轴参考角一起设（主平衡轴 pitch + 二次开发 roll/yaw）
             P<deg>                 → 单参兼容：仅设 pitch（原来 P-3.6 仍可用）
             P (无参)              → 打印三轴 ref 与当前 err，确认安装偏角补偿是否生效 */
        const char *p = &cmd[1];
        while (*p == ' ' || *p == '\r' || *p == '\n') p++;
        if (*p == '\0') {
            LOG_I("ATT", "ref(r,p,y)=%.2f,%.2f,%.2f  err=%.2f,%.2f,%.2f",
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
                LOG_I("ATT", "pitch_ref=%.2f deg (single-arg compat)", (double)v[0]);
            } else if (n >= 3) {
                Attitude_SetRollRef(v[0]);
                Attitude_SetPitchRef(v[1]);
                Attitude_SetYawRef(v[2]);
                LOG_I("ATT", "ref(r,p,y)=%.2f,%.2f,%.2f deg", (double)v[0], (double)v[1], (double)v[2]);
            } else {
                LOG_W("ATT", "P needs 1 or 3 numbers, got %d", n);
            }
        }
    } else if (op == 'K') {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        const char *p = &cmd[1];
        kp = (float)atof(p);
        while (*p && *p != ',') p++;
        if (*p == ',') { ki = (float)atof(p + 1); p++; }   // ← 关键：跳过第 1 个逗号
        while (*p && *p != ',') p++;
        if (*p == ',') kd = (float)atof(p + 1);
        Attitude_SetGains(kp, ki, kd);
        LOG_I("ATT", "gains kp=%.2f ki=%.2f kd=%.2f", kp, ki, kd);
    } else if (op == 'C') {
        /* 静置后重新标定陀螺零偏：治“上电搬动导致静止仍漂移/锯齿”。
           标定前确保板子完全不动。 */
        Attitude_CalibrateGyroStatic();
        LOG_I("ATT", "gyro re-calibrate done (board must be still)");
    } else if (op == 'D') {
        /* 诊断：打印 MPU6050 寄存器 + 一次 raw 采样，判断陀螺零读数是
           软件配置问题还是硬件损坏。命令由 StartMotorTask 路由。 */
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
           例：F elag 1 0.3  F trim 0  F print */
        const char *q = &cmd[1];
        while (*q == ' ' || *q == '\r' || *q == '\n') q++;
        if (*q == '\0' || strncmp(q, "print", 5) == 0) {
            uint8_t lag_en, trim_en, elag_en; float lag_a, elag_a; uint8_t trim_w;
            ImuFilter_GetCfg(&lag_en, &lag_a, &trim_en, &trim_w);
            Attitude_GetEulerLag(&elag_en, &elag_a);
            LOG_I("ATT", "FILT raw_lag=%u a=%.2f | trim=%u win=%u | euler_lag=%u a=%.2f",
                  (unsigned)lag_en, (double)lag_a, (unsigned)trim_en, (unsigned)trim_w,
                  (unsigned)elag_en, (double)elag_a);
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
            LOG_W("ATT", "F %s needs on/off (1|0) [optional param]", sub);
            return;
        }
        if (strcmp(sub, "lag") == 0) {
            float a = has_num ? num : (on ? LAG_ALPHA_DEFAULT : g_imu_filter.lag_alpha);
            ImuFilter_SetLag(on, a);
            LOG_I("ATT", "raw LAG %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "trim") == 0) {
            uint8_t w = has_num ? (uint8_t)num : TRIM_WIN_DEFAULT;
            ImuFilter_SetTrim(on, w);
            LOG_I("ATT", "raw TRIM %s", on ? "ON" : "OFF");
        } else if (strcmp(sub, "elag") == 0) {
            float a = has_num ? num : (on ? EULER_LAG_ALPHA_DEFAULT : s_euler_lag_alpha);
            Attitude_SetEulerLag(on, a);
            LOG_I("ATT", "euler LAG %s", on ? "ON" : "OFF");
        } else {
            LOG_W("ATT", "F subcmd unknown: %s (use lag/trim/elag/print)", sub);
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
            LOG_I("ATT", "mag re-init %s", (r == 0) ? "OK" : "FAIL (no ACK?)");
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
        QMC5822_DumpStatus();
        LOG_I("ATT", "mag ready=%u raw=%d,%d,%d calib=%.1f,%.1f,%.1f",
              (unsigned)s_mag_ready,
              (int)s_mag_raw[0], (int)s_mag_raw[1], (int)s_mag_raw[2],
              (double)s_mag_calib[0], (double)s_mag_calib[1], (double)s_mag_calib[2]);
        LOG_I("ATT", "heading(tilt-comp)=%.1f | heading(raw,no-tilt)=%.1f | align(y,p,r)=%.1f,%.1f,%.1f deg",
              (double)s_mag_heading, (double)s_mag_raw_heading,
              (double)s_mag_install_euler[0], (double)s_mag_install_euler[1], (double)s_mag_install_euler[2]);
        LOG_I("ATT", "verify: rotate board about Z 360deg -> tilt-comp heading should go 0->360 monotonic (no jump)");
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
    /* 仅当电机运行（g_motor_sys.running）内环才真正输出；这里只写目标 */
    Motor_SetSpeed(MOTOR_A, base + g_steer);
    Motor_SetSpeed(MOTOR_B, base - g_steer);
    g_last_tgtA = base + g_steer;
    g_last_tgtB = base - g_steer;
}
