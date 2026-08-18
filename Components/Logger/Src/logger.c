/**
  ****************************(C) 日志模块 *****************************
  * @file       logger.c
  * @brief      工业界风格嵌入式日志：分级 + 时间戳 + 标签 + 编译期开关
  *             + RAM 环形缓冲 + 低优先级任务异步刷串口 + Flash 黑匣子
  * @note       后端 UART 由 BSP_USART.c 以弱符号 log_backend_putc 接入，
  *             本文件不依赖任何 BSP/HAL，便于跨项目复用。
  *********************************************************************
  */
#include "logger.h"
#include "stm32h7xx_hal.h"   /* HAL_GetTick */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ============================ 配置 ============================ */
#define LOG_RB_SIZE   1024U      /* 环形缓冲容量（字节，RAM，掉电丢失） */
#define LOG_LINE_MAX  160U       /* 单条日志最大长度（含头） */


#ifndef LOG_ENABLED
/* 关闭日志时提供空实现，保证任何调用点都能正常链接 */
void logger_init(void) {}
void logger_set_level(uint8_t level) { (void)level; }
void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...) {
    (void)level; (void)level_str; (void)tag; (void)file; (void)line; (void)fmt;
}
void logger_drain(void) {}
int logger_flush_to_flash(void) { return 0; }

#else  /* ===== LOG_ENABLED ===== */

/* ===================== 环形缓冲（ISR/任务安全） ===================== */
static char          s_rb[LOG_RB_SIZE];
static volatile uint16_t s_rb_head;   /* 写入位置 */
static volatile uint16_t s_rb_tail;   /* 读出位置 */
static uint8_t       s_log_runtime_level = LOG_LEVEL;

/* 可插拔后端：默认弱实现为空；BSP_USART.c 提供真实 USART1 输出 */
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

/* ========================== 公共 API ========================== */
void logger_init(void)
{
    s_rb_head = 0;
    s_rb_tail = 0;
    s_log_runtime_level = LOG_LEVEL;
}

void logger_set_level(uint8_t level) { s_log_runtime_level = level; }

void logger_emit(uint8_t level, const char *level_str, const char *tag,
                 const char *file, int line, const char *fmt, ...)
{
    /* 运行时 + 编译期双重过滤 */
    if (level > s_log_runtime_level) return;
    if (level > LOG_MAX_LEVEL)      return;

    char buf[LOG_LINE_MAX];
    int  off = 0;

    /* 头：[时间戳ms][级别][标签] */
    off += snprintf(buf + off, sizeof(buf) - (size_t)off, "[%u][%s][%s] ",
                    HAL_GetTick(), level_str, tag);

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

/* 由低优先级 LoggerTask 循环调用：把缓冲刷到串口（阻塞，但跑在低优先级任务里） */
void logger_drain(void)
{
    char c;
    while (rb_pop(&c)) {
        log_backend_putc(c);
    }
}

/* ====================== Flash 黑匣子 ====================== */
/* 把环形缓冲线性化（只读拷贝，不破坏实时日志），交由 BSP 轮询落盘。
   返回写入字节数；<=0 表示失败或无数据。 */
int logger_flush_to_flash(void)
{
    char     tmp[LOG_RB_SIZE];
    uint16_t n = 0;
    uint16_t idx = s_rb_tail;
    while ((idx != s_rb_head) && (n < LOG_RB_SIZE)) {
        tmp[n++] = s_rb[idx];
        idx = (uint16_t)((idx + 1U) % LOG_RB_SIZE);
    }
    if (n == 0) return 0;

    /* 真正落盘由 BSP 完成：轮询写，崩溃时也能用（不依赖 RTOS/DMA） */
    extern int w25q_crashlog_save(const uint8_t *data, uint32_t len);
    return w25q_crashlog_save((const uint8_t *)tmp, n);
}

#endif /* LOG_ENABLED */
