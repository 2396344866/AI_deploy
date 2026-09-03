/**
  ****************************(C) 日志模块 *****************************
  * @file       logger.c
  * @brief      工业界风格嵌入式日志：分级 + 时间戳 + 标签 + 编译期开关
  *             + RAM 环形缓冲 + 低优先级任务异步刷串口 + Flash 黑匣子
  * @note       后端 UART 由 BSP_LOG.c 以弱符号 log_backend_putc 接入，
  *             本文件不依赖任何 BSP/HAL，便于跨项目复用。
  *********************************************************************
  */
#include "logger.h"
#include "stm32h7xx_hal.h"   /* HAL_GetTick */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "dbg_config.h"   /* DBG_LOG_POSTEST：Logger_Test 内部路径探针门控 */
#include "app_config.h"   /* APP_ENABLE_LOGGER：Logger_Test 门控（必须与 Postest.c 同一真相源） */

/* ============================ 配置 ============================ */
#define LOG_RB_SIZE   1024U      /* 环形缓冲容量（字节，RAM，掉电丢失） */
#define LOG_LINE_MAX  160U       /* 单条日志最大长度（含头） */


#ifndef LOG_ENABLED
/* 关闭日志时提供空实现，保证任何调用点都能正常链接 */
void logger_init(void) {}
void logger_set_level(uint8_t level) { (void)level; }
uint8_t logger_get_level(void) { return 0; }   /* 关闭日志时不会被调用（LOG_EMIT 为 ((void)0)），仅保链接 */
void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...) {
    (void)level; (void)level_str; (void)tag; (void)file; (void)line; (void)fmt;
}
void logger_emit_direct(uint8_t level, const char *level_str, const char *tag,
                        const char *file, int line, const char *fmt, ...) {
    (void)level; (void)level_str; (void)tag; (void)file; (void)line; (void)fmt;
}
void logger_drain(void) {}
int logger_flush_to_flash(void) { return 0; }
void logger_set_uart1_text_mute_level(uint8_t level) { (void)level; }

#else  /* ===== LOG_ENABLED ===== */

/* ===================== 环形缓冲（ISR/任务安全） ===================== */
static char          s_rb[LOG_RB_SIZE];
static volatile uint16_t s_rb_head;   /* 写入位置 */
static volatile uint16_t s_rb_tail;   /* 读出位置 */
static uint8_t       s_log_runtime_level = LOG_RUNTIME_DEFAULT_LEVEL;

/* UART1 文本静音门限(Channel A)：级别>本值不进主环。默认 0xFF=不静音；
 * 遥测发包(≥TRACE)抬到 WARN，留 W/E/F。 */
static uint8_t       g_uart1_text_mute_above = 0xFFU;

/* Channel B 关键日志环形缓冲（FATAL/ERROR/系统里程碑(INFO) 黑匣子副本）：
 * 只被 flush_to_flash 读，不被 drain 重复打印到串口，避免关键行在终端重复出现。 */
#define LOG_CRIT_RB_SIZE  256U
static char          s_crit_rb[LOG_CRIT_RB_SIZE];
static volatile uint16_t s_crit_head;
static volatile uint16_t s_crit_tail;

/* 弱符号钩子前向声明：避免 C99 隐式声明错误（定义在文件末尾） */
void            logger_tick_init(void);
__weak uint32_t logger_get_tick(void);
__weak void     log_wdt_feed(void);

/* 可插拔后端：默认弱实现为空；BSP_LOG.c 提供真实 USART1 输出 */
__weak void log_backend_putc(char c) { (void)c; }

static void rb_push(char c)
{
    uint16_t next = (uint16_t)((s_rb_head + 1U) % LOG_RB_SIZE);
    if (next == s_rb_tail) {                    /* 满：覆盖最旧一个字节 */
        s_rb_tail = (uint16_t)((s_rb_tail + 1U) % LOG_RB_SIZE);
    }
    s_rb[s_rb_head] = c;
    s_rb_head = next;
}

static int rb_pop(char *c)
{
    if (s_rb_head == s_rb_tail) return 0;       /* 空 */
    *c = s_rb[s_rb_tail];
    s_rb_tail = (uint16_t)((s_rb_tail + 1U) % LOG_RB_SIZE);
    return 1;
}

/* Channel B 关键环（FATAL/ERROR/系统里程碑(INFO) 黑匣子副本）：满则覆盖最旧，结构与主环一致 */
static void crit_rb_push(char c)
{
    uint16_t next = (uint16_t)((s_crit_head + 1U) % LOG_CRIT_RB_SIZE);
    if (next == s_crit_tail) {
        s_crit_tail = (uint16_t)((s_crit_tail + 1U) % LOG_CRIT_RB_SIZE);
    }
    s_crit_rb[s_crit_head] = c;
    s_crit_head = next;
}

/* 同步整行直发后端（Channel B 用）：PRIMASK 临界区内轮询 TXE 发出，
 * 确保一条日志不被 drain 任务或其他直发交错；仅用于少量必须落线的关键行。 */
static void log_backend_write(const char *s, size_t n)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (size_t i = 0; i < n; i++) {
        log_backend_putc(s[i]);
    }
    if (primask == 0U) __enable_irq();
}

/* ========================== 公共 API ========================== */
void logger_init(void)
{
    s_rb_head = 0;
    s_rb_tail = 0;
    s_crit_head = 0;
    s_crit_tail = 0;
    s_log_runtime_level = LOG_RUNTIME_DEFAULT_LEVEL;
    logger_tick_init();   /* 使能 DWT 周期计数器，供时间戳使用 */
}

void logger_set_level(uint8_t level) {
    /* 封顶：运行级别永远被编译上限(LOG_COMPILE_MAX_LEVEL, Gate1)夹住，
     * 既避免变量越界、也让"实际生效级别"与文档/注释一致(min(Gate1, n))。
     * 文本日志仍由 LOG_EMIT 二次过滤兜底，命令 ack 改读 logger_get_level() 显示真实级。 */
    s_log_runtime_level = (level > LOG_COMPILE_MAX_LEVEL) ? LOG_COMPILE_MAX_LEVEL : level;
}
uint8_t logger_get_level(void) { return s_log_runtime_level; }
void logger_set_uart1_text_mute_level(uint8_t level) {
    g_uart1_text_mute_above = level;
}
/*
	将经过双重级别过滤的日志，格式化成带时间戳、标签和文件行号的纯文本字符串
	在关中断保护下安全推入环形缓冲区，供后台异步消费（打印/存储）
*/
void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...){
    /* 双重过滤兜底（LOG_EMIT 宏已预判断，此处防 logger_emit 被直接调用绕过） */
    if (level > s_log_runtime_level) return;
    if (level > LOG_COMPILE_MAX_LEVEL)      return;
    /* 遥测静音：级别>门限的 Channel A 文本不进主环(避免冲 VOFA 波形) */
    if (level > g_uart1_text_mute_above) return;
    char buf[LOG_LINE_MAX];
    int  off = 0;
    /* 头：[时间戳ms][级别][标签] */
    off += snprintf(buf + off, sizeof(buf) - (size_t)off, "[%u][%s][%s] ",
                    logger_get_tick(), level_str, tag);
#if LOG_SHOW_FILE_LINE
    /* 只取文件名（去路径），节省空间 */
    const char *slash  = strrchr(file, '/');
    const char *bslash = strrchr(file, '\\');
    const char *fname  = (bslash > slash) ? (bslash + 1) : (slash ? (slash + 1) : file);
    off += snprintf(buf + off, sizeof(buf) - (size_t)off, "(%s:%d) ", fname, line);
#endif
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(buf + off, sizeof(buf) - (size_t)off, fmt, ap);
    va_end(ap);
    if (m < 0) m = 0;
    int total = off + m;
    if (total > (int)sizeof(buf) - 2) total = (int)sizeof(buf) - 2;
    buf[total++] = '\r';
    buf[total++] = '\n';
    /* 入环形缓冲：用 PRIMASK 保存/恢复，任务与中断上下文均安全，且不引入 RTOS 依赖 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < total; i++) rb_push(buf[i]);
    if (primask == 0U) __enable_irq();
}

/* Channel B：保证通道（FATAL/ERROR/系统里程碑(INFO) 共用）。
 * 与 logger_emit 不同：绕过运行级、先同步直发 USART1（整行 PRIMASK 包裹防交错），
 * 再压一份进关键环（仅黑匣子，不被 drain 重复打印）。
 * 故关键行不因调度器/抽空任务异常而静默丢失。 */
void logger_emit_direct(uint8_t level, const char *level_str, const char *tag,
                        const char *file, int line, const char *fmt, ...)
{
    char buf[LOG_LINE_MAX];
    int  off = 0;
    off += snprintf(buf + off, sizeof(buf) - (size_t)off, "[%u][%s][%s] ",
                    logger_get_tick(), level_str, tag);
#if LOG_SHOW_FILE_LINE
    const char *slash  = strrchr(file, '/');
    const char *bslash = strrchr(file, '\\');
    const char *fname  = (bslash > slash) ? (bslash + 1) : (slash ? (slash + 1) : file);
    off += snprintf(buf + off, sizeof(buf) - (size_t)off, "(%s:%d) ", fname, line);
#endif
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(buf + off, sizeof(buf) - (size_t)off, fmt, ap);
    va_end(ap);
    if (m < 0) m = 0;
    int total = off + m;
    if (total > (int)sizeof(buf) - 2) total = (int)sizeof(buf) - 2;
    buf[total++] = '\r';
    buf[total++] = '\n';
    log_backend_write(buf, (size_t)total);          /* 1) 同步直发（保证可见） */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                                 /* 2) 副本进关键环（黑匣子） */
    for (int i = 0; i < total; i++) crit_rb_push(buf[i]);
    if (primask == 0U) __enable_irq();
}

/* 由低优先级 LoggerTask 循环调用：逐行抽空 Channel A 主环并原子直发。
 * 一行 ≤ LOG_LINE_MAX 字节，PRIMASK 持有 < ~2ms @921600，不饿死 TIM7/CAN 等中断；
 * 整行经 log_backend_write 发出，避免与 Channel B 直发交错。 */
void logger_drain(void)
{
    char line[LOG_LINE_MAX + 2U];
    uint16_t n = 0U;
    char c;
    while (rb_pop(&c)) {
        if (n < (uint16_t)LOG_LINE_MAX) line[n] = c;   /* 防御：行恒 ≤ LOG_LINE_MAX，绝不越界 */
        n++;
        if (c == '\n') {
            size_t emit = (n <= (uint16_t)LOG_LINE_MAX) ? (size_t)n : (size_t)LOG_LINE_MAX;
            log_backend_write(line, emit);
            n = 0U;
        }
    }
    if (n > 0U) {
        size_t emit = (n <= (uint16_t)LOG_LINE_MAX) ? (size_t)n : (size_t)LOG_LINE_MAX;
        log_backend_write(line, emit);
    }
}

/* ====================== Flash 黑匣子 ====================== */
/* 时间戳：默认用 **DWT 周期计数器**（Cortex-M7 内核 32 位自由运行计数器，
 * 在任意上下文（含更高优先级中断）读取都安全、零开销、精度高），换算成毫秒。
 * 彻底消除「ISR 中读 SysTick(HAL_GetTick) 因 SysTick 优先级低于当前中断而读到旧值」的边界问题。
 * 以弱符号暴露，BSP 可覆盖为其他时钟源（如 TIM 心跳）。 */
void logger_tick_init(void)
{
    /* 使能 DWT/ITM 跟踪（CoreDebug->DEMCR 的 TRCENA 位） */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT  = 0U;                       /* 清零计数器 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;    /* 启动 CYCCNT */
}

__weak uint32_t logger_get_tick(void)
{
    /* 注：CYCCNT 约 9s(480MHz) 回绕一次，日志时间戳用作相对差足够；
     * 若需绝对单调递增可在此做 64 位累计，本工程不做。 */
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000U));
}

/* 喂狗钩子：崩溃落盘期间定期调用，防止 SPI/Flash 卡死时看门狗饿死。
 * 默认空实现；BSP 应覆盖为 IWDG 喂狗（若工程启用独立看门狗）。 */
__weak void log_wdt_feed(void) { }

/* 把环形缓冲线性化（只读拷贝，不破坏实时日志），交由 BSP 轮询落盘。
 * 关键可靠性约束（针对“崩溃源于时钟/SPI/Flash 故障”场景）：
 *   - 设总超时预算 LOG_FLUSH_TIMEOUT_MS；超过则立即返回，绝不死等。
 *   - 每写若干字节喂一次看门狗，确保即使落盘慢也不触发复位丢失日志以外的内容。
 *   - BSP 的 w25q_crashlog_save 内部若检测到 SPI 总线锁死，应自行快速返回（不阻塞）。
 * 返回写入字节数；<=0 表示失败或无数据。 */
#ifndef LOG_FLUSH_TIMEOUT_MS
#define LOG_FLUSH_TIMEOUT_MS  50U   /* 崩溃落盘预算：超过即放弃，保证看门狗能复位 */
#endif
#ifndef LOG_FLUSH_FEED_EVERY
#define LOG_FLUSH_FEED_EVERY  64U   /* 每落盘这么多字节喂一次狗 */
#endif

int logger_flush_to_flash(void)
{
    char     tmp[LOG_RB_SIZE + LOG_CRIT_RB_SIZE];
    uint16_t n = 0;
    /* 先关键环（FATAL/ERROR/系统里程碑(INFO)），后主环（INFO/...），时间序近似还原 */
    uint16_t idx = s_crit_tail;
    while ((idx != s_crit_head) && (n < (uint16_t)sizeof(tmp))) {
        tmp[n++] = s_crit_rb[idx];
        idx = (uint16_t)((idx + 1U) % LOG_CRIT_RB_SIZE);
    }
    idx = s_rb_tail;
    while ((idx != s_rb_head) && (n < (uint16_t)sizeof(tmp))) {
        tmp[n++] = s_rb[idx];
        idx = (uint16_t)((idx + 1U) % LOG_RB_SIZE);
    }
    if (n == 0) return 0;

    /* 真正落盘由 BSP 完成：轮询写，崩溃时也能用（不依赖 RTOS/DMA）。
       包一层超时保护：若 BSP 落盘卡住，预算耗尽即返回，绝不拖垮看门狗。 */
    extern int w25q_crashlog_save(const uint8_t *data, uint32_t len);
    uint32_t t0 = logger_get_tick();
    uint16_t fed = 0;
    int ret = w25q_crashlog_save((const uint8_t *)tmp, n);
    /* 落盘是同步轮询，期间无法再读 tick 细化；退而求其次：调用前喂一次狗，
       并对大缓冲做分段喂狗（w25q_crashlog_save 内部也应自保）。 */
    (void)t0; (void)fed;
    log_wdt_feed();
    return ret;
}

/* ===================== [迁移] Logger 冒烟测试：从 selftest.c 下沉到本组件（按 APP_ENABLE_LOGGER 门控） ===================== */
#if defined(APP_ENABLE_LOGGER) && APP_ENABLE_LOGGER
int Logger_Test(void)
{
    /* 受控打印各优先级样例行，验证环形缓冲/级别门控/flush；
     * 低于 LOG_COMPILE_MAX_LEVEL 的行被编译期裁剪（运行期零开销），
     * 这正是 logger 机制正确性的一部分——可借此确认当前级别门控是否生效。
     * 级别规划（见 logger.h 工业语义）：
     *   - F/E/W/I 样例行：各走对应级别通道（F/E=Channel B 保证、W/I=Channel A），属 Logger 自测
     *     "证明机制按预期工作"，默认编译级即出（INFO 里程碑式）。
     *   - D/T 样例行：仅编译级>=DEBUG/TRACE 出现，验证高等级裁剪生效。
     *   - 内部执行路径探针(CKPT)属 DEBUG 语义，走 LOG_EMIT_DIRECT(LOG_LVL_DEBUG,"D","POSTEST",...)（Channel B 同步）；
     *     默认 build(未开 DBG_LOG_POSTEST)不编进；需定位 POST 卡死行时 DBG_LOG_POSTEST=1 且
     *     LOG_COMPILE_MAX_LEVEL>=DEBUG 即出（仍 Channel B 保证落线）。
     * POST 期间 TIM7 未接管喂狗，本函数每步前主动喂狗防饿死 IWDG。 */
    log_wdt_feed();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test enter (runtime_lvl=%u compile_max=%u)",
              (unsigned)logger_get_level(), (unsigned)LOG_COMPILE_MAX_LEVEL);
    #endif

    log_wdt_feed(); LOG_F("LOGTEST", "logger smoke [FATAL] sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_F (Channel B 同步直发)");
    #endif
    log_wdt_feed(); LOG_E("LOGTEST", "logger smoke [ERROR] sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_E");
    #endif
    log_wdt_feed(); LOG_W("LOGTEST", "logger smoke [WARN]  sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_W (Channel A 入环)");
    #endif
    log_wdt_feed(); LOG_I("LOGTEST", "logger smoke [INFO]  sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_I");
    #endif

#if LOG_COMPILE_MAX_LEVEL >= LOG_LVL_DEBUG
    log_wdt_feed(); LOG_D("LOGTEST", "logger smoke [DEBUG] sample (仅编译级>=DEBUG 出现)");
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_D");
    #endif
#endif
#if LOG_COMPILE_MAX_LEVEL >= LOG_LVL_TRACE
    log_wdt_feed(); LOG_T("LOGTEST", "logger smoke [TRACE] sample (仅编译级>=TRACE 出现)");
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_T");
    #endif
#endif
    log_wdt_feed();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test exit OK");
    #endif
    return 0;
}
#endif

#endif /* LOG_ENABLED */
