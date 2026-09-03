#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * 工业嵌入式日志（前端宏 + 环形缓冲 + 可插拔后端）
 * 分级: FATAL0/ERROR1/WARN2/INFO3/DEBUG4/TRACE5（数字越小越严重、越低级越可见）。
 * 每条带: 时间戳(ms)+级别字符+tag+文件:行号。
 * 双裁剪: LOG_COMPILE_MAX_LEVEL(编进上限) + logger_set_level()(运行级)，有效级=min。
 * 异步: 生产任务仅 格式化+入主环，低优先级 LoggerTask 抽空刷 USART1（避阻塞 printf 反模式）。
 * 双通道: A=异步业务(INFO/DEBUG/TRACE/WARN 入主环)；B=保证(FATAL/ERROR 自动直发+系统里程碑
 *         LOG_EMIT_DIRECT(INFO))，整行 PRIMASK 同步直发+副本进关键环(黑匣子)，不依赖调度器健康。
 * 崩溃: logger_flush_to_flash() 刷最近日志入 Flash，有超时+喂狗保护，绝不死等（保看门狗能复位）。
 * 解耦: 驱动只调 LOG_*；后端由 BSP 弱符号 log_backend_putc 接入。
 * ============================================================================= */

/* 1.总开关：发布版注释掉本行即彻底关日志(零开销)；开发/烧录建议保留并把运行级调低以留故障诊断窗口 */
#define LOG_ENABLED


#ifdef LOG_ENABLED
/* 2.级别(工业语义) — 数字越小越严重、越低级默认越可见 ----
 * FATAL0: 不可恢复、致系统终止/重启（永编译进、不受运行级过滤）。
 * ERROR1: 意外 Bug 但已恢复可继续（如网络失败重试成功）。
 * WARN2 : 非错误但暗示隐患（如内存池>90%）。
 * INFO3 : 重要周期/用户事件，证明系统按预期（线上默认最低级）。
 * DEBUG4: 调试详情(变量/路径)，生产通常关。
 * TRACE5: 比 DEBUG 更细更频(每迭代/每样本)，仅特定短时开，主供 VOFA 波形。 */


#define LOG_LVL_FATAL 0
#define LOG_LVL_ERROR 1
#define LOG_LVL_WARN  2
#define LOG_LVL_INFO  3
#define LOG_LVL_DEBUG 4
#define LOG_LVL_TRACE 5

/* 编译进二进制的最高级别（可被单个 .c 文件用 #define LOG_LOCAL_LEVEL 覆盖） */
#ifndef LOG_COMPILE_MAX_LEVEL
#define LOG_COMPILE_MAX_LEVEL LOG_LVL_DEBUG
#endif
/* 运行期默认级：boot 初值，logger_set_level() 可现场改写，但永远被编译上限封顶 */
#ifndef LOG_RUNTIME_DEFAULT_LEVEL
#define LOG_RUNTIME_DEFAULT_LEVEL     LOG_LVL_DEBUG
#endif

/* 一致性铁律: LOG_COMPILE_MAX_LEVEL(编译上限) 必须 >= LOG_RUNTIME_DEFAULT_LEVEL(运行默认)。
 * 有效级=min(编译上限,运行当前)，故 logger_set_level() 永远被编译上限封顶。
 * 运行默认>编译上限=请求了二进制里没编的级别(白设)；相等(均DEBUG)开发态合法。唯一非法->#error。 */
#if LOG_RUNTIME_DEFAULT_LEVEL > LOG_COMPILE_MAX_LEVEL
#error "LOG_RUNTIME_DEFAULT_LEVEL must be <= LOG_COMPILE_MAX_LEVEL (runtime default cannot request verbosity absent from the binary; effective level = min(compile, runtime))"
#endif
/* 是否打印 文件:行号（定位更准但更占空间；发布版可置 0） */
#ifndef LOG_SHOW_FILE_LINE
#define LOG_SHOW_FILE_LINE 1
#endif

/* ---------- 3. 前端宏（业务代码调用这些） ---------- */
/* tag 为模块/任务标签，如 "DIAG" "MOTOR" "NET" "FLASH" "SPI" "MAIN" */
#define LOG_F(tag, fmt, ...) LOG_EMIT(LOG_LVL_FATAL, "F", tag, fmt, ##__VA_ARGS__)
#define LOG_E(tag, fmt, ...) LOG_EMIT(LOG_LVL_ERROR, "E", tag, fmt, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) LOG_EMIT(LOG_LVL_WARN,  "W", tag, fmt, ##__VA_ARGS__)
#define LOG_I(tag, fmt, ...) LOG_EMIT(LOG_LVL_INFO,  "I", tag, fmt, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...) LOG_EMIT(LOG_LVL_DEBUG, "D", tag, fmt, ##__VA_ARGS__)
#define LOG_T(tag, fmt, ...) LOG_EMIT(LOG_LVL_TRACE, "T", tag, fmt, ##__VA_ARGS__)

/* Channel B 保证通道：仅编译期闸门、绕过运行级、必落线；供系统里程碑(boot/POST/critical) 直接 LOG_EMIT_DIRECT(LOG_LVL_INFO,"I",tag,...) 调用，等级常量显式写在调用点（无 LOG_SYS 别名）。 */
#define LOG_EMIT_DIRECT(lvl, chr, tag, fmt, ...)                            \
    do {                                                                   \
        if ((lvl) <= LOG_COMPILE_MAX_LEVEL) {                             \
            logger_emit_direct((lvl), (chr), (tag), __FILE__, __LINE__,    \
                               (fmt), ##__VA_ARGS__);                      \
        }                                                                  \
    } while (0)

/* 编译期+运行期双重裁剪（参数零开销）：lvl > LOG_COMPILE_MAX_LEVEL 或 lvl > logger_get_level() 时 if 为死代码/短路，
logger_emit 及其参数（如 compute()）均不求值；((void)0) 空宏同理（未引用参数预处理期即丢弃）
—— 二者皆零开销，无"参数照常运行"陷阱。 */
#define LOG_EMIT(lvl, chr, tag, fmt, ...)                                  \
    do {                                                                   \
        if ((lvl) <= LOG_COMPILE_MAX_LEVEL) {                              \
            if ((lvl) <= LOG_LVL_ERROR) {                                  \
                /* Channel B：保证通道，绕过运行级，直发+黑匣子（FATAL/ERROR 永可见） */ \
                logger_emit_direct((lvl), (chr), (tag), __FILE__, __LINE__, \
                                   (fmt), ##__VA_ARGS__);                  \
            } else if ((lvl) <= logger_get_level()) {                      \
                /* Channel A：异步业务日志，受运行级过滤，入主环由 drain 抽空 */ \
                logger_emit((lvl), (chr), (tag), __FILE__, __LINE__,       \
                            (fmt), ##__VA_ARGS__);                         \
            }                                                              \
        }                                                                  \
    } while (0)

#else /* LOG_ENABLED 未定义 -> 所有宏为空，零开销（并保证链接不报错） */

#define LOG_E(tag, fmt, ...) ((void)0)
#define LOG_W(tag, fmt, ...) ((void)0)
#define LOG_I(tag, fmt, ...) ((void)0)
#define LOG_D(tag, fmt, ...) ((void)0)
#define LOG_T(tag, fmt, ...) ((void)0)

#endif /* LOG_ENABLED */

/* ---------- 4. 后端 API（logger.c 实现；关闭日志时为空实现） ---------- */
void logger_init(void);
void logger_set_level(uint8_t level);
uint8_t logger_get_level(void);   /* 运行时级别（供 LOG_EMIT 短路判断，使超门参数零开销） */
/* 关日志零开销：LOG_COMPILE_MAX_LEVEL 外文本为死代码被删，参数不求值。 */
void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...);
void logger_emit_direct(uint8_t level, const char *level_str, const char *tag,
                        const char *file, int line, const char *fmt, ...);
void logger_drain(void);           /* 由低优先级 LoggerTask 循环调用，刷到串口 */
int  logger_flush_to_flash(void);  /* 崩溃时调用，把最近日志刷入 Flash 黑匣子 */
/* 遥测静音门限(Channel A)：级别>本值不进主环。默认 0xFF=不静音；
 * 发包(≥TRACE)抬到 WARN，静音 INFO/DBG/TRACE，留 W/E/F；Channel B 不受影响。 */
void logger_set_uart1_text_mute_level(uint8_t level);

/* 喂狗钩子（弱符号默认空；BSP 覆盖为 IWDG 喂狗）。崩溃落盘与常态喂狗共用，保看门狗不被饿死。 */
void log_wdt_feed(void);

/* POST Logger 冒烟测试入口（按 APP_ENABLE_LOGGER 门控）：分级/门控/flush 自检。
 * 实现见 logger.c 尾部。 */
int Logger_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
