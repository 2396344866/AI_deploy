#ifndef _MADGWICK_H
#define _MADGWICK_H

#include <stdint.h>

/* =============================================================================
 * Madgwick AHRS（姿态融合库，C 移植自 xioTechnologies/MadgwickAHRS）
 *   纯 C、无依赖，适合 Keil AC6 直接编译（避免 C++ 混编复杂度）。
 *   若你更想用官方仓库或 DMP，可整体替换本文件，接口保持一致即可。
 *   约定：UpdateIMU 输入 陀螺=°/s（内部转 rad/s）、加速度=g。
 * ============================================================================= */

void Madgwick_Init(float sampleRateHz);
void Madgwick_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float Madgwick_GetRoll(void);    /* deg */
float Madgwick_GetPitch(void);   /* deg */
float Madgwick_GetYaw(void);     /* deg */

#endif /* _MADGWICK_H */
