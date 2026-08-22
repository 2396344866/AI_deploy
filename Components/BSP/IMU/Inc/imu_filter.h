#ifndef _IMU_FILTER_H
#define _IMU_FILTER_H

#include <stdint.h>

/* =============================================================================
 * 可运行时配置的 IMU 输入滤波（阶段3，重构：编译期宏 -> 运行时变量）
 *   - 默认出厂值由下方 DEFAULT 宏给出；运行时可用串口 F 命令动态改，无需重烧。
 *   - RAW_FILTER_TRIMMED_MEAN（去极值均值）：默认【开启】（用户要求）。
 *   - RAW_FILTER_LAG（一阶滞后）：默认开启。
 *   两个都开时，先去极值均值、再做一阶滞后。
 *   姿态级互补滤波在 attitude.c，由 ATTITUDE_BACKEND 选择，不在这里。
 *
 * 为什么改成运行时：原来滤波是编译期宏，要对比"开/关"效果得烧两次固件；
 *   现在 raw（CH0-5）与滤波后（CH6-11）同时输出，配合 F 命令实时开关，
 *   在 VOFA 同屏即可 A/B 对比，不必重烧。
 * ============================================================================= */

/* 默认出厂值（运行时可覆盖） */
#define RAW_FILTER_LAG_DEFAULT    1      /* 一阶滞后默认开 */
#define RAW_FILTER_TRIM_DEFAULT   1      /* 去极值均值默认开（用户要求） */
#define LAG_ALPHA_DEFAULT         0.20f  /* 0~1，越大越跟手越噪、越小越平滑越滞后 */
#define TRIM_WIN_DEFAULT          5      /* 窗口大小（3~8 为宜） */
#define TRIM_WIN_MAX              8      /* 运行时窗口上限（数组固定大小，改大需同步数组） */

/* 运行时滤波配置（全局，供 F 命令与 ImuFilter_Update 共用） */
typedef struct {
    uint8_t lag_en;      /* 一阶滞后使能 */
    uint8_t trim_en;     /* 去极值均值使能 */
    float   lag_alpha;   /* 一阶滞后系数 0~1 */
    uint8_t trim_win;    /* 去极值均值窗口 3~TRIM_WIN_MAX */
} ImuFilterCfg_t;

extern ImuFilterCfg_t g_imu_filter;

typedef struct {
    int16_t raw_accel[3];   /* 原始计数（未滤波），供 VOFA 对照 */
    int16_t raw_gyro[3];
    float   ax, ay, az;     /* 物理量：g（滤波后） */
    float   gx, gy, gz;     /* 物理量：°/s（滤波后） */
} ImuData_t;

void ImuFilter_Init(void);
/* 输入原始计数，按运行时配置滤波，输出物理量（raw 同时保留供 VOFA） */
void ImuFilter_Update(const int16_t raw_accel[3], const int16_t raw_gyro[3], ImuData_t *out);

/* 运行时配置接口（F 命令调用） */
void ImuFilter_SetLag(uint8_t en, float alpha);
void ImuFilter_SetTrim(uint8_t en, uint8_t win);
void ImuFilter_GetCfg(uint8_t *lag_en, float *alpha, uint8_t *trim_en, uint8_t *win);

#endif /* _IMU_FILTER_H */
