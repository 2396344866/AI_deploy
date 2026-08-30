/*
 * watchdog_heartbeat.c —— 看门狗心跳实现（运行期任务存活探针）
 * 详见 watchdog_heartbeat.h。本文件位于 Components/Debug/Src（用户区），
 * CubeMX 重生成零影响；需加入 Keil 的 Debug 源组（与 dbg_telemetry.c 同组）才参与编译。
 */
#include "watchdog_heartbeat.h"
#include "app_config.h"   /* APP_ENABLE_X：决定哪些任务被监视（编译掉的跳过） */

volatile uint32_t g_wdt_tick_cnt = 0;
volatile uint32_t g_hb_ticks[HB_COUNT] = {0};
static uint8_t s_armed = 0;

/* 任务主循环调用：打时间戳（必须 < HB_COUNT，防越界） */
void task_heartbeat_kick(hb_task_id_t id)
{
    if ((int)id < (int)HB_COUNT) {
        g_hb_ticks[id] = g_wdt_tick_cnt;
    }
}

/* POST 收尾调用：置 armed，并新鲜戳所有被监视任务，
 * 避免 POST 刚结束、业务任务尚未跑第一轮 kick 的瞬间被误判冻结。 */
void watchdog_arm(void)
{
    g_hb_ticks[HB_MOTOR]     = g_wdt_tick_cnt;
    g_hb_ticks[HB_SENSOR]    = g_wdt_tick_cnt;
    g_hb_ticks[HB_INFERENCE] = g_wdt_tick_cnt;
    g_hb_ticks[HB_NETWORK]   = g_wdt_tick_cnt;
    g_hb_ticks[HB_ESP32S3]   = g_wdt_tick_cnt;
    g_hb_ticks[HB_SCREEN]    = g_wdt_tick_cnt;
    g_hb_ticks[HB_FLASH]     = g_wdt_tick_cnt;
    s_armed = 1;
}

/* TIM7 ISR 喂狗前调用：armed 且所有被监视任务心跳都新鲜才返回 1。
 * 用 #if defined(APP_ENABLE_X) && APP_ENABLE_X 判定被监视集合（未定义宏按 0，安全）。 */
int watchdog_should_feed(void)
{
#if !defined(APP_ENABLE_WATCHDOG) || !APP_ENABLE_WATCHDOG
    return 0;  /* 调试关狗：TIM7 永不喂 */
#endif
    uint32_t now;
    if (!s_armed) return 0;
    now = g_wdt_tick_cnt;

#if defined(APP_ENABLE_MOTOR) && APP_ENABLE_MOTOR
    if (now - g_hb_ticks[HB_MOTOR] > HB_STALE_TICKS) return 0;
#endif
#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
    if (now - g_hb_ticks[HB_SENSOR] > HB_STALE_TICKS) return 0;
#endif
#if defined(APP_ENABLE_INFERENCE) && APP_ENABLE_INFERENCE
    if (now - g_hb_ticks[HB_INFERENCE] > HB_STALE_TICKS) return 0;
#endif
#if defined(APP_ENABLE_NETWORK) && APP_ENABLE_NETWORK
    if (now - g_hb_ticks[HB_NETWORK] > HB_STALE_TICKS) return 0;
#endif
#if defined(APP_ENABLE_ESP32S3) && APP_ENABLE_ESP32S3
    if (now - g_hb_ticks[HB_ESP32S3] > HB_STALE_TICKS) return 0;
#endif
#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
    if (now - g_hb_ticks[HB_SCREEN] > HB_STALE_TICKS) return 0;
#endif
#if defined(APP_ENABLE_FLASH) && APP_ENABLE_FLASH
    if (now - g_hb_ticks[HB_FLASH] > HB_STALE_TICKS) return 0;
#endif
    return 1;
}
