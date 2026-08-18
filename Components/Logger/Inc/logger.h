#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * 工业界嵌入式日志模块（前端宏 + 环形缓冲中间件 + 可插拔后端）
 *
 * 设计要点：
 *  - 分级：ERROR / WARN / INFO / DEBUG / TRACE
 *  - 每条日志自动带：时间戳(ms, HAL_GetTick) + 级别字符 + 模块/任务标签 + 文件:行号
 *  - 编译期一键关闭：不定义 LOG_ENABLED 时，所有 LOG_* 宏展开为空，零开销（发布版关闭）
 *  - 两级裁剪：LOG_MAX_LEVEL（编译进二进制的上限）+ 运行时 logger_set_level()
 *  - 生产任务只做“格式化 + 入环形缓冲”（几微秒），后台低优先级任务异步刷串口
 *    -> 避免阻塞式 printf 破坏实时性（这是本项目原 fputc 轮询输出的反模式）
 *  - 崩溃时 logger_flush_to_flash() 把最近日志刷入 Flash（黑匣子），崩溃最后一行不丢
 *
 * 放置原则：本模块是独立组件，驱动（BSP/HAL）只“调用” LOG_* 宏，不内含日志实现；
 *           后端（UART）由 BSP 以弱符号 log_backend_putc 接入，模块本身与 BSP 解耦。
 * ============================================================================= */

/* ---------- 1. 总开关：发布版注释掉下面这行即可彻底关闭全部日志 ---------- */
#define LOG_ENABLED

#ifdef LOG_ENABLED

/* ---------- 2. 级别 ---------- */
#define LOG_LVL_ERROR 1
#define LOG_LVL_WARN  2
#define LOG_LVL_INFO  3
#define LOG_LVL_DEBUG 4
#define LOG_LVL_TRACE 5

/* 编译进二进制的最高级别（可被单个 .c 文件用 #define LOG_LOCAL_LEVEL 覆盖） */
#ifndef LOG_MAX_LEVEL
#define LOG_MAX_LEVEL LOG_LVL_DEBUG
#endif
/* 运行时默认过滤级别（DEBUG 及以上编译进去了，但默认只显示 INFO 及以上） */
#ifndef LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_INFO
#endif
/* 是否打印 文件:行号（定位更准但更占空间；发布版可置 0） */
#ifndef LOG_SHOW_FILE_LINE
#define LOG_SHOW_FILE_LINE 1
#endif

/* ---------- 3. 前端宏（业务代码调用这些） ---------- */
/* tag 为模块/任务标签，如 "INFER" "MOTOR" "NET" "FLASH" "SPI" "MAIN" */
#define LOG_E(tag, fmt, ...) LOG_EMIT(LOG_LVL_ERROR, "E", tag, fmt, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) LOG_EMIT(LOG_LVL_WARN,  "W", tag, fmt, ##__VA_ARGS__)
#define LOG_I(tag, fmt, ...) LOG_EMIT(LOG_LVL_INFO,  "I", tag, fmt, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...) LOG_EMIT(LOG_LVL_DEBUG, "D", tag, fmt, ##__VA_ARGS__)
#define LOG_T(tag, fmt, ...) LOG_EMIT(LOG_LVL_TRACE, "T", tag, fmt, ##__VA_ARGS__)

/* 内部：仅当级别 <= 编译上限时才生成代码（否则连函数调用都不产生） */
#define LOG_EMIT(lvl, chr, tag, fmt, ...)                                  \
    do {                                                                   \
        if ((lvl) <= LOG_MAX_LEVEL) {                                      \
            logger_emit((lvl), (chr), (tag), __FILE__, __LINE__,           \
                        (fmt), ##__VA_ARGS__);                             \
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
void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...);
void logger_drain(void);           /* 由低优先级 LoggerTask 循环调用，刷到串口 */
int  logger_flush_to_flash(void);  /* 崩溃时调用，把最近日志刷入 Flash 黑匣子 */

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
