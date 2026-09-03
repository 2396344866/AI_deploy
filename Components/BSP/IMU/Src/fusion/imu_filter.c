/* =============================================================================
 * 输入滤波（运行时可配置）：去极值均值 + 一阶滞后，作用物理量层（g / °/s）。
 *   与姿态融合解耦，便于 VOFA 单独评估；raw 计数始终保留供对照。
 *   配置见 ImuFilterCfg_t（g_imu_filter），可用 F 命令动态改，无需重烧。
 * =============================================================================
 */
#include "imu_filter.h"
#include "imu_mpu6050.h"   /* 取 MPU_ACCEL_LSB_PER_G / MPU_GYRO_LSB_PER_DPS */
#include <string.h>
#include <math.h>

/* 运行时配置（默认出厂值来自 imu_filter.h 的 DEFAULT 宏） */
ImuFilterCfg_t g_imu_filter = {
    .lag_en    = RAW_FILTER_LAG_DEFAULT,
    .trim_en   = RAW_FILTER_TRIM_DEFAULT,
    .lag_alpha = LAG_ALPHA_DEFAULT,
    .trim_win  = TRIM_WIN_DEFAULT,
};

static int16_t s_win[6][TRIM_WIN_MAX];   /* [0..2]=accel, [3..5]=gyro，每通道独立滑动窗口 */
static uint8_t s_widx[6]   = {0};        /* 每通道环形写指针 */
static uint8_t s_wcount[6] = {0};        /* 每通道已填充样本数（warmup 期 < trim_win） */
static ImuData_t s_last;                 /* 一阶滞后上一拍 */

void ImuFilter_Init(void)
{
    memset(s_win, 0, sizeof(s_win));
    for (int i = 0; i < 6; i++) { s_widx[i] = 0; s_wcount[i] = 0; }
    memset(&s_last, 0, sizeof(s_last));
}

/* 窗口内去掉一个最大、一个最小后求平均；窗口<=2 时直接平均。
   sum 用 int32_t：accel 计数 (-15000 量级) × 窗口(5) 会超出 int16_t，导致滤波结果完全失真。 */
static int16_t trim_mean(const int16_t *w, uint8_t n)
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

void ImuFilter_Update(const int16_t raw_accel[3], const int16_t raw_gyro[3], ImuData_t *out)
{
    int16_t src[6];
    for (int i = 0; i < 3; i++) src[i]     = raw_accel[i];
    for (int i = 0; i < 3; i++) src[3 + i] = raw_gyro[i];

    /* 1) 去极值均值（运行时开关；修正：warmup 期只统计已填充样本，不读未初始化的 0） */
    if (g_imu_filter.trim_en) {
        uint8_t w = g_imu_filter.trim_win;
        if (w < 3) w = 3;
        if (w > TRIM_WIN_MAX) w = TRIM_WIN_MAX;
        for (int i = 0; i < 6; i++) {
            int16_t *b = s_win[i];
            /* 先写入新样本，再统计——保证新样本参与均值（旧逻辑在 n 计算前写入，
               导致新样本落在 [0..n-1] 之外被剔除，且 warmup 会读到未填充的 0） */
            b[s_widx[i]] = src[i];
            if (s_wcount[i] < w) s_wcount[i]++;
            s_widx[i] = (s_widx[i] + 1U) % w;
            /* 有效样本数 = 已填充数（warmup）或窗口大小（已满）。
               顺序写入期间有效样本恒在 buf[0..n-1]；已满后环形覆盖仍为全有效 */
            uint8_t n = (s_wcount[i] < w) ? s_wcount[i] : w;
            src[i] = trim_mean(b, n);
        }
    }

    /* 2) 保留原始计数（未滤波）供 VOFA 对照 */
    out->raw_accel[0] = raw_accel[0]; out->raw_accel[1] = raw_accel[1]; out->raw_accel[2] = raw_accel[2];
    out->raw_gyro[0]  = raw_gyro[0];  out->raw_gyro[1]  = raw_gyro[1];  out->raw_gyro[2]  = raw_gyro[2];

    /* 3) 计数 -> 物理量 */
    float a[3], g[3];
    for (int i = 0; i < 3; i++) a[i] = (float)src[i]       / MPU_ACCEL_LSB_PER_G;
    for (int i = 0; i < 3; i++) g[i] = (float)src[3 + i]   / MPU_GYRO_LSB_PER_DPS;

    /* 4) 一阶滞后（运行时开关） */
    if (g_imu_filter.lag_en) {
        a[0] = s_last.ax + g_imu_filter.lag_alpha * (a[0] - s_last.ax);
        a[1] = s_last.ay + g_imu_filter.lag_alpha * (a[1] - s_last.ay);
        a[2] = s_last.az + g_imu_filter.lag_alpha * (a[2] - s_last.az);
        g[0] = s_last.gx + g_imu_filter.lag_alpha * (g[0] - s_last.gx);
        g[1] = s_last.gy + g_imu_filter.lag_alpha * (g[1] - s_last.gy);
        g[2] = s_last.gz + g_imu_filter.lag_alpha * (g[2] - s_last.gz);
    }

    out->ax = a[0]; out->ay = a[1]; out->az = a[2];
    out->gx = g[0]; out->gy = g[1]; out->gz = g[2];

    /* 始终更新上一拍（lag 关时也跟随），避免重新开 lag 瞬间跳变 */
    s_last = *out;
}

void ImuFilter_SetLag(uint8_t en, float alpha)
{
    g_imu_filter.lag_en = en ? 1 : 0;
    if (alpha >= 0.0f && alpha <= 1.0f) g_imu_filter.lag_alpha = alpha;
}

void ImuFilter_SetTrim(uint8_t en, uint8_t win)
{
    g_imu_filter.trim_en = en ? 1 : 0;
    if (win >= 3 && win <= TRIM_WIN_MAX) {
        g_imu_filter.trim_win = win;
        /* 窗口变化：清空缓存，避免新旧数据混合 */
        memset(s_win, 0, sizeof(s_win));
        for (int i = 0; i < 6; i++) { s_widx[i] = 0; s_wcount[i] = 0; }
    }
}

void ImuFilter_GetCfg(uint8_t *lag_en, float *alpha, uint8_t *trim_en, uint8_t *win)
{
    if (lag_en)  *lag_en  = g_imu_filter.lag_en;
    if (alpha)   *alpha   = g_imu_filter.lag_alpha;
    if (trim_en) *trim_en = g_imu_filter.trim_en;
    if (win)     *win     = g_imu_filter.trim_win;
}
