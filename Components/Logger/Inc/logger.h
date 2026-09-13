#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 工业日志：分级 FATAL0<ERROR1<WARN2<INFO3<DEBUG4<TRACE5（越小越严重、越低级越可见），
 * 每条带 时间戳(ms)+级别字符+tag+文件:行号。
 * 双裁剪：LOG_COMPILE_MAX_LEVEL(编进上限) + logger_set_level()(运行级)，有效级=min。
 * 异步：生产任务只 格式化+入主环，LoggerTask 抽空刷 USART1（避阻塞 printf 反模式）。
 * 双通道：A=异步(INFO/DEBUG/TRACE/WARN 入主环)；
 *         B=保证(FATAL/ERROR + LOG_EMIT_DIRECT)，PRIMASK 同步直发+副本进黑匣子，不依赖调度器健康。
 * 崩溃：logger_flush_to_flash() 刷最近日志入 Flash，带超时+喂狗，绝不死等（保看门狗能复位）。
 * 解耦：业务只调 LOG_*；后端由 BSP 弱符号 log_backend_putc 接入。 */

/* 总开关：发布版注释掉本行即彻底关日志（零开销） */
#define LOG_ENABLED


#ifdef LOG_ENABLED
/* 级别（工业语义）：
 * FATAL 不可恢复、终止/重启（永编译进、不受运行级过滤）| ERROR 意外但已恢复（如重试成功）
 * WARN 非错误但暗示隐患 | INFO 周期/用户事件（线上默认）| DEBUG 调试详情（生产关）
 * TRACE 比 DEBUG 更细更频（每迭代/样本），仅短时开，主供 VOFA 波形 */


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

/* 铁律：编译上限 >= 运行默认（有效级=min，运行级永被封顶），违反 #error */
#if LOG_RUNTIME_DEFAULT_LEVEL > LOG_COMPILE_MAX_LEVEL
#error "LOG_RUNTIME_DEFAULT_LEVEL must be <= LOG_COMPILE_MAX_LEVEL (effective level = min(compile, runtime))"
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

/* Channel B 保证通道：仅编译期闸门、绕过运行级、必落线；等级常量显式写在调用点（无 LOG_SYS 别名） */
#define LOG_EMIT_DIRECT(lvl, chr, tag, fmt, ...)                            \
    do {                                                                   \
        if ((lvl) <= LOG_COMPILE_MAX_LEVEL) {                             \
            logger_emit_direct((lvl), (chr), (tag), __FILE__, __LINE__,    \
                               (fmt), ##__VA_ARGS__);                      \
        }                                                                  \
    } while (0)

/* 双重裁剪（参数零开销）：lvl 超编译上限或运行级 -> if 短路，logger_emit 及其参数（如 compute()）不求值 */
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

#else /* LOG_ENABLED 未定义 -> 全空，零开销（并保证链接不报错） */

#define LOG_E(tag, fmt, ...) ((void)0)
#define LOG_W(tag, fmt, ...) ((void)0)
#define LOG_I(tag, fmt, ...) ((void)0)
#define LOG_D(tag, fmt, ...) ((void)0)
#define LOG_T(tag, fmt, ...) ((void)0)

#endif /* LOG_ENABLED */

/* ---------- 4. 后端 API（logger.c 实现；关闭日志时为空实现） ---------- */
void logger_init(void);
void logger_set_level(uint8_t level);
uint8_t logger_get_level(void);   /* 运行级（供 LOG_EMIT 短路判断，使超门参数零开销） */
void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...);
void logger_emit_direct(uint8_t level, const char *level_str, const char *tag,
                        const char *file, int line, const char *fmt, ...);
void logger_drain(void);           /* 由低优先级 LoggerTask 循环调用，刷到串口 */
int  logger_flush_to_flash(void);  /* 崩溃时把最近日志刷入 Flash 黑匣子 */
/* 遥测静音(Channel A)：级别>本值不进主环；0xFF=不静音。发包(≥TRACE)抬到 WARN，留 W/E/F；Channel B 不受影响 */
void logger_set_uart1_text_mute_level(uint8_t level);

/* 喂狗钩子（弱符号默认空，BSP 覆盖为 IWDG）：崩溃落盘与常态喂狗共用，防看门狗饿死 */
void log_wdt_feed(void);

/* POST 冒烟入口（APP_ENABLE_LOGGER 门控）：分级/门控/flush 自检，实现见 logger.c 尾部 */
int Logger_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
