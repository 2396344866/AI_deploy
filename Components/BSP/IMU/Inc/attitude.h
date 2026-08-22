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
void Attitude_ProcessCommand(const char *cmd, uint16_t len);  /* 串口命令 T/P/K 解析（由任务路由调用） */
void Attitude_RunController(void);              /* 计算并下发电机目标（任务里调用） */
void Attitude_GetTargets(int32_t *tgtA, int32_t *tgtB);  /* 供 VOFA 显示 */

/* 调试遥测 getter（供 Components/Debug 固定帧使用） */
float    Attitude_GetPitchRef(void);                 /* 目标俯仰角(deg) */
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

/* 磁力计 getter（阶段1 9轴，供 VOFA CH37-43）：raw=原始计数，calib=减硬铁偏移后，
   heading=atan2 磁航向(deg)。阶段1 未标定，calib 等价 raw。 */
void    Attitude_GetRawMag(int16_t mag[3]);
void    Attitude_GetCalibMag(float mag[3]);
float   Attitude_GetMagHeading(void);
int     Attitude_MagInit(void);       /* 运行时重探测 QMC5883L（M init 命令调用） */

#endif /* _ATTITUDE_H */
