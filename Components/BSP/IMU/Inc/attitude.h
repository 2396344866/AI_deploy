#ifndef _ATTITUDE_H
#define _ATTITUDE_H

#include "imu_filter.h"
#include "madgwick.h"
#include <stdint.h>

/* =============================================================================
 * 姿态解算 + 外环控制器（阶段4 / 阶段5）
 *   后端可切换（宏）：Madgwick（默认，库融合） / 互补滤波（无库兜底）。
 *   外环：倾角误差 -> 轮速目标，经 Motor_SetSpeed 写入内环（TIM7 1kHz 速度PI）。
 * ============================================================================= */

/* 后端选择 */
#define ATTITUDE_BACKEND_MADGWICK       0
#define ATTITUDE_BACKEND_COMPLEMENTARY  1

#ifndef ATTITUDE_BACKEND
#define ATTITUDE_BACKEND   ATTITUDE_BACKEND_MADGWICK
#endif

/* 外环运行频率（与 MPU6050 SMPLRT_DIV 对应：1000/(1+4)=200Hz） */
#ifndef ATTITUDE_RATE_HZ
#define ATTITUDE_RATE_HZ   200U
#endif

/* 安装方向 / 符号（标准安装：芯片 Z 轴向上、X 轴向前）。
   若实际安装不同或倾角方向反了，改这里翻转，无需动算法。 */
#ifndef IMU_PITCH_SIGN
#define IMU_PITCH_SIGN   1      /* 俯仰角符号：若倒地时角度反向，改为 -1 */
#endif

/* 互补滤波系数（仅 ATTITUDE_BACKEND==COMPLEMENTARY 生效） */
#ifndef COMP_ALPHA
#define COMP_ALPHA       0.98f
#endif

/* 欧拉角后滤波默认值（运行期可经 F 命令开关/调参，默认开）：
   压 Madgwick 输出毛刺，降低 PID 微分项噪声。比物理量层更跟手。 */
#ifndef EULER_FILTER_LAG_DEFAULT
#define EULER_FILTER_LAG_DEFAULT   1
#endif
#ifndef EULER_LAG_ALPHA_DEFAULT
#define EULER_LAG_ALPHA_DEFAULT    0.30f
#endif

/* 磁力计一阶滞后默认（运行期 F mlag 开关/调参，默认开）：压 QMC5883L 噪声，
   仅作用于航向计算向量；滤波结果 s_mag_lpf 供 VOFA CH40-42（航向实际用量），
   s_mag_calib 仍保留原始值。 */
#ifndef MAG_LAG_ALPHA_DEFAULT
#define MAG_LAG_ALPHA_DEFAULT      0.15f
#endif
/* 磁力计去极值均值滑窗（F mtrim）：与 accel/gyro 的 TRIM 对称，对原始计数去单点野值/降噪，零相位 */
#ifndef MAG_TRIM_WIN_DEFAULT
#define MAG_TRIM_WIN_DEFAULT       5       /* 窗口大小（3~8 为宜） */
#endif
#ifndef MAG_TRIM_WIN_MAX
#define MAG_TRIM_WIN_MAX           8       /* 运行期窗口上限（数组固定大小，改大需同步数组） */
#endif
/* yaw 分离式磁融合：陀螺积分 + 磁航向互补增益（F yawmag 调参，默认 0.02）。
   仅用于 yaw：pitch/roll 恒由 6 轴 Madgwick/互补给出，不受磁影响。
   增益越大锁磁越快、抗磁扰越弱；过小则 yaw 漂移抑制不足。 */
#ifndef YAW_MAG_GAIN_DEFAULT
#define YAW_MAG_GAIN_DEFAULT      0.02f
#endif

typedef struct {
    float roll, pitch, yaw;   /* deg */
} Attitude_t;

/* 姿态角索引（三轴参考角/误差统一索引访问，便于二次开发扩展） */
#define ATT_ROLL   0
#define ATT_PITCH  1
#define ATT_YAW    2

int  Attitude_Init(void);
void Attitude_Update(const ImuData_t *imu);
float Attitude_GetRoll(void);
float Attitude_GetPitch(void);
float Attitude_GetYaw(void);
const Attitude_t* Attitude_Get(void);

/* 外环控制接口（命令 T/P/K 调用） */
void Attitude_SetEnable(uint8_t on);
uint8_t Attitude_GetEnable(void);
/* 三轴参考角（安装偏角补偿）：pitch 为主平衡轴，roll/yaw 供二次开发 */
void Attitude_SetPitchRef(float deg);
void Attitude_SetRollRef(float deg);
void Attitude_SetYawRef(float deg);
void Attitude_SetAttRef(uint8_t axis, float deg);
void Attitude_SetGains(float kp, float ki, float kd);
void Attitude_SetSteer(int32_t steer);          /* 转向差（计数/节拍），左右轮对称加减 */
void Attitude_SetHeadingK(float k);             /* 航向保持增益(k_yaw)：>0 时磁门控航向PID接管转向；0=关走外部steer */
float Attitude_GetHeadingK(void);
/* 把 yaw_ref 对齐到当前航向（engage 防阶跃）：yaw_ref 默认 0 而 hdg_err=yaw_ref-mag_hdg≈-66°，
   直接开 k_yaw 会输出满舵转向指令。SetHeadingK 在 0->正 上升沿自动调用；也可手动调用以重新定航向。 */
void    Attitude_SnapYawRefToCurrent(void);
uint8_t Attitude_IsYawRefSnapped(void);         /* yaw_ref 是否已对齐过（1=对齐，0=待首拍对齐） */
void Attitude_ProcessCommand(const char *cmd, uint16_t len);  /* 串口命令 T/P/K 解析（由任务路由调用） */
void Attitude_RunController(void);              /* 计算并下发电机目标（任务里调用） */
void Attitude_GetTargets(int32_t *tgtA, int32_t *tgtB);  /* 供 VOFA 显示 */

/* 调试遥测 getter（供 Components/Debug 固定帧使用）
 * ⚠ 命名澄清：Ref = Reference（目标角/设定值，即"想达到的角"），非残差(residual)。
 *   本工程真实"实测−估计"残差叫 RES/INNOV（如 yaw_innov / ACC_RES_*）。遥测层用 _tgt 后缀
 *   （与 motA_tgt/motB_tgt 对齐）避开 REF/RES/INNOV 一字之差；ERR = 当前角−Ref，属控制误差非观测新息。 */
float    Attitude_GetPitchRef(void);                 /* 目标俯仰角(deg)，即 Ref 设定值 */
float    Attitude_GetRollRef(void);                  /* 目标横滚角(deg) */
float    Attitude_GetYawRef(void);                   /* 目标偏航角(deg) */
float    Attitude_GetPitchErr(void);                 /* = pitch - pitch_ref(deg)，前倾为正 */
float    Attitude_GetRollErr(void);                  /* = roll - roll_ref(deg) */
float    Attitude_GetYawErr(void);                   /* = yaw - yaw_ref(deg) */
void     Attitude_GetGains(float *kp, float *ki, float *kd);  /* 外环增益 */
int32_t  Attitude_GetSteer(void);                    /* 转向差(计数/节拍) */

/* 欧拉角后滤波（运行期可开关/调参，默认开）：压 Madgwick 输出毛刺，降 PID 微分噪声 */
void    Attitude_SetEulerLag(uint8_t en, float alpha);
void    Attitude_GetEulerLag(uint8_t *en, float *alpha);
float   Attitude_GetRawRoll(void);   /* 后滤波前欧拉角（Madgwick 直出），供 VOFA 对比 */
float   Attitude_GetRawPitch(void);
float   Attitude_GetRawYaw(void);

/* 磁力计 getter（9 轴，供 VOFA MAG 组 18-26 / CH34）：
   raw=原始计数；calib=轴系对齐(§1.1)后减硬铁偏移（阶段1 offset=0 等价透传）；
   heading=倾角补偿罗盘(§1.2)航向(deg)，稳定可观测；raw_heading=水平 atan2（调试对比）。 */
void    Attitude_GetRawMag(int16_t mag[3]);
void    Attitude_GetCalibMag(float mag[3]);
void    Attitude_GetFilteredMag(float mag[3]);   /* 经一阶滞后的标定磁向量(航向实际用量)，供 VOFA CH21-23 */
float   Attitude_GetMagHeading(void);      /* 倾角补偿后磁航向(deg)，同 CH34 */
float   Attitude_GetRawMagHeading(void);   /* 水平 atan2 磁航向(deg)，未做倾角补偿，供对比 */
/* 磁力计一阶滞后（运行期 F mlag 开关/调参，默认开）：仅影响航向计算用向量 */
void    Attitude_SetMagLag(uint8_t en, float alpha);
void    Attitude_GetMagLag(uint8_t *en, float *alpha);
void    Attitude_SetMagTrim(uint8_t en, uint8_t win);
void    Attitude_GetMagTrim(uint8_t *en, uint8_t *win);
/* yaw 分离式磁融合增益：陀螺积分 + 磁航向互补（wrap-safe）。仅作用于 yaw，pitch/roll 不变 */
void    Attitude_SetYawMagGain(float gain);
float   Attitude_GetYawMagGain(void);
/* yaw 融合诊断（仅供 VOFA 对比，不参与控制）：
   GetYawGyro = 纯陀螺积分 yaw(°)：不碰磁，直观看漂移；
   GetYawInnov = 磁校正前新息(°)：yaw_wrap_diff(mag_hdg, fused)，即磁"拉回多少"；
   GetGyroBiasEst/GetGyroOffset = 在线零偏估计 / 静态零偏校准(°/s)，看零偏健康。 */
float   Attitude_GetYawGyro(void);
float   Attitude_GetYawInnov(void);
void    Attitude_GetGyroBiasEst(float bias[3]);
void    Attitude_GetGyroOffset(float bias[3]);
/* 航向误差(wrap-safe)：供 PID 航向控制避免 0/360 跳变。
   优先磁航向(绝对,CH34)，mag 不在则退回 6 轴融合 yaw(漂移)。返回 (-180,180]。 */
float   Attitude_GetHeadingErr(float target_deg);
int     Attitude_MagInit(void);       /* 运行时重探测 QMC5883L（M init 命令调用） */

/* §1.1 轴系对齐：GY273 与 MPU6050 是两块独立小板，朝向大概率不一致。
   用安装欧拉角(deg)把磁力计芯片原生系旋到 IMU body 系再融合。
   默认 0 = 假设两板平行同向；若绕某轴偏角，调这三个值（M align 命令运行期生效）。 */
void    Attitude_SetMagAlign(float yaw_deg, float pitch_deg, float roll_deg);
void    Attitude_GetMagAlign(float *yaw_deg, float *pitch_deg, float *roll_deg);

/* =============================================================================
 * 本地自治（v4b 框架，详见 Components/Motor/balance_autonomy_plan.md）
 *   - 本地自治为唯一真相源；云端 / KEY 仅发“意图”（站起 / 坐下）。
 *   - FSM 永久接管使能决策（无临时门控、无旧逻辑兜底）：
 *     传感器健康 → 倾角安全 → 网络状态(Phase-2) → 站起/坐下校验。
 * ============================================================================= */

/* 安全阈值（俯仰/横滚绝对值，deg） */
#ifndef ATT_TILT_OVERTURN_DEG
#define ATT_TILT_OVERTURN_DEG    50.0f   /* 超过即倾覆，强制急停关平衡 */
#endif
#ifndef ATT_TILT_STAND_READY_DEG
#define ATT_TILT_STAND_READY_DEG  10.0f  /* 在此范围内才允许 ENGAGE 站起 */
#endif
#ifndef ATT_TILT_SIT_SAFE_DEG
#define ATT_TILT_SIT_SAFE_DEG      8.0f  /* 接近此姿态才允许优雅放下 */
#endif

/* 本地自治接口（由 StartSensorTask 周期驱动 AutonomyTick） */
void Attitude_SetKeyStand(uint8_t req);    /* KEY 站起/坐下意图：1=想站起 0=想坐下 */
void Attitude_SetCloudStand(uint8_t req);  /* 云端 balance_enable=1 意图 */
void Attitude_SetCloudSit(uint8_t req);    /* 云端 balance_enable=0 意图 */
void Attitude_SetNetOnline(uint8_t on);    /* 网络在线状态（由 StartNetworkTask 边沿同步：CONNACK OK→1 / 掉线→0） */
uint8_t Attitude_IsHealthy(void);          /* 传感器/姿态健康代理 */
void Attitude_AutonomyTick(void);          /* 周期执行 FSM 决策，更新 g_enabled 与电机运行态 */

/* POST 传感器自检入口（按 APP_ENABLE_SENSOR 门控）：MPU6050/QMC5883L 数据合理性校验。
 * 实现见 attitude.c 尾部（与姿态解算同文件，避免 Postest.c 越权包含 IMU 内部细节）。 */
int Sensor_Test(void);

#endif /* _ATTITUDE_H */
