/**
  ****************************(C) COPYRIGHT 2021 Boring_TECH*********************
  * @file       BSP_USART.c/h
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
#include "BSP_USART.h"
#include <stdarg.h>
#include <stdio.h>
// 新版 ARM Compiler 6 的底层标准库已经内置了 struct __FILE 的定义


int fputc(int ch, FILE *f)
{ 	
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR=(u8)ch;      
	return ch;
}

/* 日志模块的可插拔后端：logger.c 通过弱符号接入此函数，把字符送到 USART1。
   与 fputc 共用同一发送等待，但解耦了日志模块对 stdio/FILE 的依赖。 */
void log_backend_putc(char c)
{
    while ((USART1->ISR & 0x40U) == 0U);   /* 等待 TXE */
    USART1->TDR = (uint8_t)c;
}

/* 轮询方式发送：仅等 TXE 后写 TDR，不依赖 HAL/RTOS 状态，可在中断上下文安全调用。
   用于 USART1 收帧后的即时回显（RX 链路探针），也可供业务层在 ISR 内安全打印。 */
void BSP_UART1_SendPoll(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;
    for (uint16_t i = 0U; i < len; i++) {
        while ((USART1->ISR & 0x40U) == 0U);   /* 等待 TXE */
        USART1->TDR = data[i];
    }
}


/* ---- USART1 RX：CubeMX 生成的 hdma_usart1_rx（DMA1_Stream4, Circular）+ 空闲线中断 ---- */
static uint8_t s_uart1_rx[USART1_RX_BUF_SIZE];

/**
  * @brief  启动 USART1 的 DMA 循环接收 + 空闲中断
  * @note   IDLE 必须在启动 DMA 之前使能，否则首个 IDLE 不触发
  */
void BSP_UART1_RxStart(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE);
}

/**
  * @brief  USART1 空闲中断处理：取一帧长度，交给回调，再重启 DMA
  * @note   由 stm32h7xx_it.c 的 USART1_IRQHandler 调用
  */
void BSP_UART1_IdleHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);                            /* 必须先清 IDLE 标志 */
        uint16_t rx_len = USART1_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx); /* 注意：hdmarx 已是指针，不加 & */
        HAL_UART_DMAStop(&huart1);
        /* 排除整缓冲帧：rx_len==缓冲大小多为"未收到有效数据却触发 IDLE"的异常情形，
           此时 DMA 缓冲仍是 BSS 初值(全 0)，直接丢弃避免产生 \0 空帧。 */
        if (rx_len > 0U && rx_len < (uint16_t)USART1_RX_BUF_SIZE)
        {
            /* STM32H7 D-Cache 一致性：DMA 写入 s_uart1_rx 的内存，CPU 可能从 D-Cache
               读到旧值(0)。读之前先失效对应 Cache 行，强制从 RAM 取真实数据。
               地址需 32 字节对齐，长度向上取整到 32。 */
            uint32_t addr = (uint32_t)s_uart1_rx & 0xFFFFFFE0U;
            uint32_t size = ((uint32_t)s_uart1_rx - addr) + (uint32_t)rx_len;
            size = (size + 31U) & ~31U;
            SCB_InvalidateDCache_by_Addr((void *)addr, size);
            BSP_UART1_OnFrame(s_uart1_rx, rx_len);
        }
        HAL_UART_Receive_DMA(&huart1, s_uart1_rx, USART1_RX_BUF_SIZE); /* 重启，NDTR 复位 */
    }
}

/**
  * @brief  帧接收完成回调（弱符号，业务层重定义即可覆盖）
  * @note   默认行为：回显，便于先用串口助手验证 DMA+IDLE 是否打通
  */
__weak void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len)
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
    /* USART1 走 DMA+IDLE，帧在 BSP_UART1_IdleHandler 处理，这里无需动作；
       UART4 同理。保留空回调以兼容 HAL 弱符号约定。 */
    (void)huart;
}

void UART4_Printf(const char *format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart4, (uint8_t*)buffer, len, 0xFFFF);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
