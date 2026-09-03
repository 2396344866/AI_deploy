/**
  * @file    uart_rx_dispatcher.c
  * @brief   Unified HAL_UARTEx_RxEventCallback dispatcher
 * @note    HAL_UARTEx_RxEventCallback is ONE global weak symbol; all UARTs using
 *          HAL_UARTEx_ReceiveToIdle_IT (USART1=LOG/VOFA console, USART6=ESP32-S3,
 *          USART2=ESP-01S) trigger it, so there can be only one strong definition,
 *          otherwise multiple modules redefining it -> link conflict.
  *          This file takes over centrally and dispatches to each BSP module's
  *          *_UART_RxCallback by huart instance.
  *          Placed under Components/BSP (not overwritten by CubeMX regen), no duplicate
  *          definitions in each module.
  */
#include "main.h"        /* huart1 / huart2 / huart6 */
#include "usart.h"       /* HAL UART 句柄声明（huart1/2/4/6；main.h 不声明） */
#include "esp32s3.h"     /* ESP32S3_UART_RxCallback */
#include "esp01s.h"      /* ESP01S_UART_RxCallback */
#include "BSP_LOG.h"     /* BSP_LOG_UART1_OnRxEvent / log_backend_putc */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart6) {
        ESP32S3_UART_RxCallback(huart, size);   /* ESP32-S3 image result frame */
    } else if (huart == &huart2) {
        ESP01S_UART_RxCallback(huart, size);    /* ESP-01S AT/MQTT response */
    } else if (huart == &huart1) {
        BSP_LOG_UART1_OnRxEvent(size);          /* USART1 调试控制台帧（HAL_UARTEx_ReceiveToIdle_IT） */
    }
    /* Other UARTs (USART3/4) using ReceiveToIdle can add branches here */
}

/* ============================================================================
 * UART RX 错误回调（强定义，覆盖 HAL 弱空实现）
 * ----------------------------------------------------------------------------
 * 策略（用户拍板 2026-08-30，B+C 合并）：
 *   B 分级  —— 关键链路(huart1/2/6) 保留致命语义；可选外设(huart4 屏) 非致命。
 *   C 治本  —— ISR 内【绝不 while(1)】。回调只做：清错误标志 + 重启接收 +
 *             存 volatile 快照；致命性裁决下沉到任务上下文（LoggerTask 调
 *             uart_err_monitor）：flush 黑匣子 + NVIC_SystemReset。
 *
 * 为什么 ISR 不能挂死：ISR 中 while(1) 冻结的是整个调度器（含喂狗 TIM7 与
 *   日志任务），一次线路噪声(FE/ORE/NE 在 UART/RS485 极常见)即升级为整机死锁。
 *   这些是可恢复错误，应清错重收，而非停机。
 *
 * ISR 安全约束：本回调只做 HAL 句柄操作 + 写 volatile 快照，不调 LOG_/阻塞/
 *   不调 RTOS（重启接收用 HAL_UARTEx_ReceiveToIdle_IT / HAL_UART_Receive_DMA，
 *   与 RxCallback 内同类调用一致，ISR 安全）。致命复位一律交给任务上下文。
 *
 * 覆盖：huart1(控制台)/huart2(ESP-01S)/huart6(ESP32-S3) 走 IT 接收；
 *   huart4(屏, DMA) 走 DMA 接收。错误全部汇聚到此回调。
 * ==========================================================================*/
#include "logger.h"   /* LOG_W / LOG_E, logger_flush_to_flash */

/* 致命升级阈值：关键链路连续错误达到该值 -> 判定链路卡死 -> 黑匣子+复位。
 * 单次瞬时 ORE/FE（921600 链路常见）不致命，重收后自愈；只有持续故障才升级。 */
#define UART_ERR_FATAL_THRESHOLD  3U

typedef enum {
    UART_ROLE_OPTIONAL = 0,   /* huart4 屏：非致命，重收+计数+继续 */
    UART_ROLE_CRITICAL = 1,   /* huart1/2/6：致命语义，任务上下文裁决复位 */
} uart_role_t;

/* 任务上下文消费的快照（ISR 只写，LoggerTask 只读，单消费者无竞争） */
static volatile struct {
    UART_HandleTypeDef *huart;     /* 出错的实例（用于重启/分级） */
    uint32_t            errorcode; /* HAL_UART_ERROR_* 位掩码 */
    uint32_t            count;     /* 累计错误次数（单调，用于阈值裁决） */
    uint32_t            ts_ms;     /* HAL_GetTick() 时间戳（ISR 安全） */
    uint8_t             pending;   /* 1 = 有未处理事件，任务侧清 0 */
} s_uart_err = {0};

/* UART4 DMA 接收缓冲（定义于 Core/Src/main.c，仅在 SCREEN 使能时启动） */
extern uint8_t rx_buf[];

static const char *uart_which(UART_HandleTypeDef *h)
{
    if (h == &huart6) return "USART6(ESP32-S3@921600)";
    if (h == &huart2) return "USART2(ESP-01S)";
    if (h == &huart1) return "USART1(LOG console)";
    if (h == &huart4) return "UART4(SCREEN)";
    return "OTHER";
}

static uart_role_t uart_role_of(UART_HandleTypeDef *h)
{
    return (h == &huart4) ? UART_ROLE_OPTIONAL : UART_ROLE_CRITICAL;
}

/* 错误后重启接收：清标志 + 按实例重武装 RX。ISR 上下文调用（HAL 调用与
 * RxCallback 内同类，安全）。 */
static void uart_err_recover(UART_HandleTypeDef *huart, uint32_t err)
{
    /* 清 PE/FE/NE/ORE 标志；ORE 需读 RDR 才能清除 FIFO 残留 */
    if (err & HAL_UART_ERROR_ORE) {
        (void)huart->Instance->RDR;
    }
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart == &huart1) {
        BSP_LOG_UART1_RxStart();
    } else if (huart == &huart2) {
        (void)ESP01S_UART_RxStart();
    } else if (huart == &huart6) {
        (void)ESP32S3_UART_RxStart();
    } else if (huart == &huart4) {
        HAL_UART_DMAStop(huart);
        HAL_UART_Receive_DMA(huart, rx_buf, RX4_BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t err = huart->ErrorCode;

    /* C：ISR 绝不 while(1)。仅快照 + 重启接收；致命性交给任务上下文。 */
    s_uart_err.huart     = huart;
    s_uart_err.errorcode = err;
    s_uart_err.count++;
    s_uart_err.ts_ms     = HAL_GetTick();
    s_uart_err.pending   = 1U;

    uart_err_recover(huart, err);
    /* 注意：此处不打印、不挂死。证据经 s_uart_err 留存，由 uart_err_monitor 落盘。 */
}

/* ----------------------------------------------------------------------------
 * 任务上下文裁决（由 StartLoggerTask 每轮调用；仅在 s_uart_err.pending 时动作）
 * --------------------------------------------------------------------------*/
void uart_err_monitor(void)
{
    if (s_uart_err.pending == 0U) return;

    UART_HandleTypeDef *huart = s_uart_err.huart;
    uint32_t err    = s_uart_err.errorcode;
    uint32_t count  = s_uart_err.count;
    uint32_t ts     = s_uart_err.ts_ms;
    s_uart_err.pending = 0U;   /* 先 ack，避免 ISR 重入时漏处理（下次 pending 再置） */

    const char *which = uart_which(huart);

    if (uart_role_of(huart) == UART_ROLE_OPTIONAL) {
        LOG_W("UART", "%s rx err recovered: code=0x%08X count=%u t=%ums (non-fatal)",
              which, (unsigned)err, (unsigned)count, (unsigned)ts);
        return;  /* 可选外设：重收后继续运行 */
    }

    /* 关键链路：先告警 + 自愈；持续故障才升级为致命复位（避免单次瞬断误杀） */
    LOG_W("UART", "%s rx err: code=0x%08X count=%u (recovered; escalate if persists)",
          which, (unsigned)err, (unsigned)count);
    if (count >= UART_ERR_FATAL_THRESHOLD) {
        LOG_E("UART", "%s wedged after %u errors -> flush blackbox + reset",
              which, (unsigned)count);
        logger_flush_to_flash();   /* 崩溃前把最近日志落盘（带喂狗保护） */
        NVIC_SystemReset();        /* 不返回 */
    }
}

/* ----------------------------------------------------------------------------
 * 测试接口：软件注入一次 RX 错误（无需 USB-TTL）。
 * 复用与真实硬件 ISR 完全相同的 HAL_UART_ErrorCallback 恢复/快照路径，
 * 仅用于联调验证 huart4(非致命) / huart6(致命) 行为。
 *   e4 -> 注入 UART4(屏)        FE 错误（非致命：清标志+重武装+计数，不复位）
 *   e6 -> 注入 USART6(ESP32-S3) FE 错误（关键链路：累计到阈值升级黑匣子+复位）
 * 调用方：DbgConsole_Process（freertos.c）。正常 ISR 行为不受影响。
 * --------------------------------------------------------------------------*/
void uart_err_inject_test(UART_HandleTypeDef *huart, uint32_t code)
{
    if (huart == NULL) return;
    huart->ErrorCode = code;
    HAL_UART_ErrorCallback(huart);   /* 与真实 ISR 完全相同的逻辑 */
}
