/* Madgwick AHRS 算法 C 移植（xioTechnologies/MadgwickAHRS, MIT）。
   输入：陀螺 °/s，加速度 g；输出：roll/pitch/yaw deg。 */
#include "madgwick.h"
#include <math.h>

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float beta = 0.1f;          /* 收敛增益，越大跟手越快越抖 */
static float invSampleFreq = 0.005f;

void Madgwick_Init(float sampleRateHz)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    if (sampleRateHz > 0.0f) invSampleFreq = 1.0f / sampleRateHz;
}

void Madgwick_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float _q0q0, _q1q1, _q2q2, _q3q3;

    /* 陀螺 °/s -> rad/s */
    gx *= 0.0174532925f; gy *= 0.0174532925f; gz *= 0.0174532925f;

    /* 加速度归一化（零向量保护） */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        _q0q0 = q0 * q0; _q1q1 = q1 * q1; _q2q2 = q2 * q2; _q3q3 = q3 * q3;

        /* 梯度下降校正步（IMU-only，与 xioTechnologies 原版一致）。
           历史版本 s1/s2/s3 把 q1/q2/q3 与 ax/ay/az 抄混，导致静止时四元数持续漂移、
           欧拉角周期性锯齿跳变（VOFA L12-L14 波形异常）。 */
        s0 = _4q0 * _q2q2 + _2q2 * ax + _4q0 * _q1q1 - _2q1 * ay;
        s1 = _4q1 * _q3q3 - _2q3 * ax + 4.0f * _q0q0 * q1 - _2q0 * ay
           - _4q1 + _8q1 * _q1q1 + _8q1 * _q2q2 + _4q1 * az;
        s2 = 4.0f * _q0q0 * q2 + _2q0 * ax + _4q2 * _q3q3 - _2q3 * ay
           - _4q2 + _8q2 * _q1q1 + _8q2 * _q2q2 + _4q2 * az;
        s3 = 4.0f * _q1q1 * q3 - _2q1 * ax + 4.0f * _q2q2 * q3 - _2q2 * ay;

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        /* 四元数微分方程 */
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    }
}

float Madgwick_GetRoll(void)
{
    return atan2f(2.0f * (q0 * q1 + q2 * q3),
                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2957795f;
}

float Madgwick_GetPitch(void)
{
    return asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.2957795f;
}

float Madgwick_GetYaw(void)
{
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2957795f;
}
