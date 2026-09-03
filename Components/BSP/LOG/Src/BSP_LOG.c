/**
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  * @file       BSP_LOG.c/h
  * @brief      将HAL库串口函数进行二次封装，并在串口中断中接收数据
  * @note      	
  * @history
  *  Version    Date            Author          Modification
  *  V3.0.0     2020.7.14     	              	1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  */
#include "BSP_LOG.h"
#include <stdarg.h>
#include <stdio.h>
#include "dbg_config.h"   // 调试开关集中管理（DBG_TELEMETRY_UART_RX 等）；与 freertos.c 同源
// 新版 ARM Compiler 6 的底层标准库已经内置了 struct __FILE 的定义


/* 最小同步发送：等 TXE(0x80) 而非 TC(0x40)，且有界超时。
 * 原实现等 TC 且零超时：USART 异常时 TC 不拉起即在关中断临界区内自旋 ->
 * TIM7 喂狗 ISR 进不来 -> 看门狗超时复位(狗开) / 永久挂起(狗关)。
 * 现改等 TXE(装下一字节即可) + ~20us 超时丢字放行，绝不自旋。ISR/HardFault 上下文安全。 */
static void bsp_uart1_emit(char c)
{
    volatile uint32_t guard = 8000U;   /* @400MHz ~20us，覆盖一帧传输，健康 USART 必就绪 */
    while ((USART1->ISR & 0x80U) == 0U) {
        if (guard-- == 0U) return;     /* 超时丢字，不自旋 */
    }
    USART1->TDR = (uint8_t)c;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    bsp_uart1_emit((char)ch);
    return ch;
}

/* 日志模块的可插拔后端：logger.c 通过弱符号接入此函数，把字符送到 USART1。
 * 与 fputc 共用同一发送等待，但解耦了日志模块对 stdio/FILE 的依赖。 */
void log_backend_putc(char c)
{
    bsp_uart1_emit(c);
}

/* 轮询方式发送：等 TXE 后写 TDR，不依赖 HAL/RTOS 状态，可在中断上下文安全调用。
 * 用于 USART1 收帧后的即时回显（RX 链路探针），也可供业务层在 ISR 内安全打印。 */
void BSP_LOG_UART1_SendPoll(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;
    for (uint16_t i = 0U; i < len; i++) {
        bsp_uart1_emit((char)data[i]);
    }
}


/* ---- USART1 RX：与 ESP(USART6/USART2) 统一走 HAL_UARTEx_ReceiveToIdle_IT 范式 ----
 * 真因（非"手写 IDLE 只触发一次"）：① 原手写 DMA+IDLE(BSP_UART1_IdleHandler) 本就清 IDLE+重武装，
 *   且 UART4 至今同套路仍正常，证手写 IDLE 可用；② 真正坑是 H7 上 HAL_UARTEx_ReceiveToIdle_DMA
 *   的 IDLE 回调 size 恒为 0（HAL 真 bug），故 ESP(本就无 DMA) 走 IT 是对的；
 *   ③ USART1 为 115200 控制台，IT 由 CPU 同核写 RAM、无 D-Cache 一致性问题，无需 DMA，并入此范式。
 *   注意：勿因本注释把 UART4 也"修"成 IT —— UART4 手写 DMA+IDLE 仍正确，无需动。 */
static uint8_t s_uart1_rx[USART1_RX_BUF_SIZE];

#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
volatile uint32_t g_dbg_idle_irq   = 0U;  /* RxEventCallback 调用次数（定位用） */
volatile uint32_t g_dbg_idle_hits  = 0U;  /* 产出有效帧次数（size>0） */
#endif

void BSP_LOG_UART1_RxStart(void)
{
    /* CubeMX 为 USART1 初始化了 hdma_usart1_rx（DMA1_Stream4）。
       本工程 USART1 控制台改走 IT 模式，必须先把残留的 DMA 状态清干净，
       否则 DMA + IT 同时挂在 RX 上，HAL 状态机腐蚀，首个 RX 中断就会踩栈/跑飞。
       断开 hdmarx 可防止 HAL_UART_IRQHandler 再去处理 DMA 标志。 */
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_DMAStop(&huart1);
    if (huart1.hdmarx != NULL) {
        __HAL_DMA_DISABLE(huart1.hdmarx);   /* 强制关流，忽略残留 TC/HT 标志 */
        huart1.hdmarx = NULL;               /* 与 CubeMX DMA 解耦，USART1 RX 完全走 IT */
    }
    huart1.RxState = HAL_UART_STATE_READY;  /* 清 HAL 内部 busy 锁 */

    /* 启动一次 IT 收帧；IDLE 触发后 HAL 调 RxEventCallback -> OnRxEvent，
       回调末重武装才能持续收（与 esp01s.c:83 / esp32s3.c:145 同范式）。 */
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);
    (void)st;   /* 可在 Keil Watch 看 st；非 HAL_OK 说明启动失败（如仍 BUSY） */
}

/* HAL_UARTEx_RxEventCallback(USART1) 入口：size = 本次 IDLE 前收到的字节数，∈[0,BUF_SIZE] */
void BSP_LOG_UART1_OnRxEvent(uint16_t size)
{
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
    g_dbg_idle_irq++;
#endif
    if (size > 0U && size <= (uint16_t)USART1_RX_BUF_SIZE)
    {
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
        g_dbg_idle_hits++;
#endif
        /* IT 模式：数据已由 HAL ISR 写入 s_uart1_rx[0,size)，CPU 同核读写，无 D-Cache 一致性问题。
           先拷出线性帧再重武装（重武装会复位 pRxBuffPtr，避免与 OnFrame 复用缓冲冲突）。 */
        static uint8_t frame[USART1_RX_BUF_SIZE];
        for (uint16_t i = 0U; i < size; i++) frame[i] = s_uart1_rx[i];
        BSP_LOG_UART1_OnFrame(frame, size);   /* 强定义在 freertos.c，推 g_cmd_qHandle */
    }
    /* 重武装：ToIdle 收完 UART 回 READY、HAL 关闭 IDLE IE，必须重启用才能收下一帧 */
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);
}

/**
  * @brief  帧接收完成回调（弱符号，业务层重定义即可覆盖）
  * @note   默认行为：回显，便于先用串口助手验证 DMA+IDLE 是否打通
  */
__weak void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 0xFFFF);
}

/**
	* @brief          串口接收完成回调（DMA 循环模式缓冲翻转时由 HAL 调用）
  * @param[in]     \thuart 串口序号
  * @retval         none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART1 走 HAL_UARTEx_ReceiveToIdle_IT，帧在 HAL_UARTEx_RxEventCallback ->
       BSP_LOG_UART1_OnRxEvent 处理，这里无需动作；UART4 同理。
       保留空回调以兼容 HAL 弱符号约定。 */
    (void)huart;
}

void UART4_Printf(const char *format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    /* vsnprintf 失败返回负值；超界也须钳制，否则转 uint16_t 后变成巨大值 -> 越界发送 64KB 垃圾。
     * 超时由 0xFFFF(≈65s) 改为 100ms：屏未接/异常时不再长时间阻塞（若在任务里调用会饿死看门狗）。 */
    if (len > 0 && (uint32_t)len < (uint32_t)sizeof(buffer)) {
        HAL_UART_Transmit(&huart4, (uint8_t*)buffer, (uint16_t)len, 100U);
    }
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
