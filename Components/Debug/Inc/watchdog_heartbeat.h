#ifndef WATCHDOG_HEARTBEAT_H
#define WATCHDOG_HEARTBEAT_H
#include <stdint.h>

/* =============================================================================
 * 看门狗心跳（运行期任务存活探针）
 * -----------------------------------------------------------------------------
 * 设计定位（与日志/功能宏正交）：
 *   - 独立于 DBG_LOG_<TASK> 日志宏，常开，生产也生效（任务冻结是真实故障，
 *     不能因为关了调试宏就失效）。
 *   - 各关键任务在主循环调 task_heartbeat_kick(id)；TIM7 ISR 喂狗前经
 *     watchdog_should_feed() 检查"所有被监视任务"心跳是否新鲜，
 *     任一超时即不喂 -> IWDG 复位，抓运行期冻结。
 *   - 监视集合与 APP_ENABLE_X 对齐：编译掉的模块(=0/未定义)不监视，
 *     避免"不存在的任务永不 kick"导致假复位。
 *
 * 与 POST 策略配合（见 main.c TIM7 / selftest.c）：
 *   上电->POST 期间 TIM7 不喂狗(g_wdt_tick_cnt 仍走)，由 POST 测试代码
 *   log_wdt_feed() 协作式喂（卡死测试->IWDG 抓到，慢测试活过）；
 *   POST 收尾 watchdog_arm() 置 armed + 新鲜戳，此后 TIM7 接管、
 *   且要求各被监视任务持续 kick 才喂。
 * =============================================================================
 */

typedef enum {
    HB_MOTOR = 0,
    HB_SENSOR,
    HB_INFERENCE,
    HB_NETWORK,
    HB_ESP32S3,
    HB_SCREEN,
    HB_FLASH,
    HB_COUNT
} hb_task_id_t;

#define HB_STALE_TICKS  2000U  /* 心跳超 2000 个 TIM7 tick(≈2s) 未刷新即判冻结 */

extern volatile uint32_t g_wdt_tick_cnt;        /* TIM7 1ms 计数（kick 打时间戳用） */
extern volatile uint32_t g_hb_ticks[HB_COUNT];  /* 各任务最近一次 kick 的 TIM7 tick */

void task_heartbeat_kick(hb_task_id_t id);
void watchdog_arm(void);          /* POST 收尾调用：置 armed + 新鲜戳所有被监视任务 */
int  watchdog_should_feed(void);  /* armed && 所有被监视任务心跳新鲜 -> 可喂 */

#endif /* WATCHDOG_HEARTBEAT_H */
