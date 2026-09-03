/*
 * Postest.c — 上电自检（POST）编排层
 * ---------------------------------------------------------------------------
 * 上电后由 Task_Test(osPriorityHigh) 在应用任务运行前一次性跑完 Postest_RunAll()，
 * 跑完即 osThreadTerminate 自删。各 Xxx_Test 实现位于各自组件 .c（见 Postest.h）。
 *
 * 边界：POST 仅做「静态/离线」自检（USART 句柄 / JEDEC / 模型加载 / 传感器 WHO_AM_I 等）；
 *       WiFi·MQTT·ESP32 链路等「运行期握手」由各任务（StartNetworkTask 等）在 POST 之后完成，不在此阶段。
 * 自检表 g_postests[] 与 APP_ENABLE_X 同一真相源：未使能模块不编译、不进表 → 根除 IWDG 复位环。
 * 关键失败→刷黑匣子+停机；非关键→继续。收尾 watchdog_arm()（运行期心跳喂狗接管）。
 * 里程碑走 Channel B 同步直发（LOG_EMIT_DIRECT INFO / LOG_F），不被运行级 / 抽空任务饿死。
 * ---------------------------------------------------------------------------
 */


#include "cmsis_os2.h"
#include "main.h"          /* HAL_GetTick */
#include "logger.h"        /* LOG_*, LOG_EMIT_DIRECT, logger_flush_to_flash */
#include "dbg_config.h"    /* DBG_LOG_POSTEST */
#include "app_config.h"    /* APP_ENABLE_*：自检表与门控同一真相源 */
#include "watchdog_heartbeat.h" /* POST 收尾 watchdog_arm() + TIM7 运行期心跳喂狗 */
#include "iwdg.h"          /* log_wdt_feed */

/* 各组件自检入口声明（实现分散在对应 .c，详见 Postest.h） */
#include "BSP_W25Q64.h"    /* Flash_Test */
#include "ai_infer.h"      /* FaultDiag_ML_Test */
#include "attitude.h"      /* Sensor_Test */
#include "esp32s3.h"       /* Esp32S3_Test */
#include "motor.h"         /* Motor_Test */
#include "esp01s.h"        /* Network_Test */
/* Logger_Test 声明见上方 logger.h（同 TU 已包含） */

/* 日志分级：里程碑=LOG_EMIT_DIRECT(INFO) Channel B（绕过运行级/抽空任务必落线）；
 *   CRITICAL FAIL=LOG_F（终止系统）；non-critical=LOG_W（继续）；
 *   编排探针=LOG_EMIT_DIRECT(DEBUG) 由 DBG_LOG_POSTEST 门控；模块细节归各自域。均不经 DBG_LOG_<TASK>。 */

/* 看门狗三阶段（详见 iwdg.c）：①上电→POST 开始不监管（耗时不可控、无喂狗点，提前启动必致复位环）；
 * ②POST 期间 IWDG_Start() 已跑，各 Xxx_Test 用 log_wdt_feed() 协作喂（真卡死仍被 IWDG 抓到复位）；
 * ③收尾 watchdog_arm() → TIM7 心跳接管，被监视任务须持续 kick 才喂。 */

/* ===================== 屏幕自检桩（无驱动，保留在框架内） ===================== */
#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
int Screen_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Screen_Test enter");
#endif
    log_wdt_feed();
    /* 工程当前无 Components 级屏幕驱动（仅有 Doc/ 参考代码），无法做真实自检。
       保持 SKIP，不臆造不存在的函数；后续接入 OLED/LCD 驱动再补 Init+Clean+图案自检。 */
    LOG_I("POSTEST", "Screen_Test: no in-tree driver linked -> SKIP");
    return 0;
}
#endif

/* ===================== 一站式调度 ===================== */
typedef struct {
    const char *name;
    int (*test)(void);
    int  critical;   /* 1=关键（失败→存档+停机）；0=非关键（失败→继续） */
} postest_entry_t;

/* 自检表与 APP_ENABLE_X 同一真相源：未使能模块的函数不编译、也不进表，
 * 故 logger 等最小 profile 下数组只含 Logger 一项，无任何硬件/无模型自测噪声。 */
static const postest_entry_t g_postests[] = {
#if defined(APP_ENABLE_FLASH) && APP_ENABLE_FLASH
    {"Flash",     Flash_Test,          1},
#endif
#if defined(APP_ENABLE_INFERENCE) && APP_ENABLE_INFERENCE
    {"Inference", FaultDiag_ML_Test,   1},
#endif
#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
    {"Sensor",    Sensor_Test,         1},
#endif
#if defined(APP_ENABLE_ESP32S3) && APP_ENABLE_ESP32S3
    {"Esp32S3",   Esp32S3_Test,        0},
#endif
#if defined(APP_ENABLE_MOTOR) && APP_ENABLE_MOTOR
    {"Motor",     Motor_Test,          0},
#endif
#if defined(APP_ENABLE_NETWORK) && APP_ENABLE_NETWORK
    {"Network",   Network_Test,        0},
#endif
#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
    {"Screen",    Screen_Test,         0},
#endif
#if defined(APP_ENABLE_LOGGER) && APP_ENABLE_LOGGER
    {"Logger",    Logger_Test,         0},
#endif
};

int Postest_RunAll(void)
{
    IWDG_Start();   /* 阶段②开始：POST 期间狗生效，各 Xxx_Test 用 log_wdt_feed() 协作喂（真卡死仍被抓） */
    LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "=== Power-On Self-Test start (%u modules) ===",
          (unsigned)(sizeof(g_postests) / sizeof(g_postests[0])));
    uint32_t post_t0 = HAL_GetTick();   /* POST 总耗时基准（DEBUG 级汇总，见结尾） */
    /* 里程碑走 Channel B 同步直发（不被 StartLoggerTask 抽空饿死）；POST 期狗靠 log_wdt_feed() 协作喂。 */

    for (size_t i = 0; i < sizeof(g_postests) / sizeof(g_postests[0]); i++) {
        const postest_entry_t *e = &g_postests[i];
        log_wdt_feed();   /* 每模块进测前协作喂狗：某测试短暂阻塞也不致 IWDG 误复位 */
        #if DBG_LOG_POSTEST
            LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Postest_RunAll: entering [%u/%u] %s (critical=%d)",
                  (unsigned)(i + 1),
                  (unsigned)(sizeof(g_postests) / sizeof(g_postests[0])), e->name, e->critical);
        #endif
        LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "[%u/%u] %s ...", (unsigned)(i + 1),
              (unsigned)(sizeof(g_postests) / sizeof(g_postests[0])), e->name);

        int rc = e->test();
        if (rc != 0) {
            if (e->critical) {
                LOG_F("POSTEST", "CRITICAL %s FAIL rc=%d -> flush blackbox + halt",
                      e->name, rc);
                logger_flush_to_flash();   /* 崩溃前把最近日志落盘 */
                /* 安全停机，不复位：此时 IWDG 已在跑，必须持续喂狗。否则 4.1s 后
                 * 复位 → 重跑 POST → 再次刷写黑匣子，既丢现场又磨 W25Q。 */
                for (;;) { log_wdt_feed(); }
                
            } else {
                LOG_W("POSTEST", "non-critical %s FAIL rc=%d (continue)", e->name, rc);
            }
        } else {
            LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "%s OK", e->name);
        }
    }

    LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "=== Self-Test done ===");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Self-Test total elapsed=%lu ms (POST 段耗时；运行期握手在各自任务，不计入)",
              (unsigned long)(HAL_GetTick() - post_t0));
    #endif
    watchdog_arm();   /* POST 收尾：arming 看门狗，TIM7 此后接管、要求各被监视任务持续 kick 才喂 */
    return 0;
}
