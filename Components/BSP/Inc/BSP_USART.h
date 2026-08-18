#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "main.h"
#include "usart.h"

#include <string.h>
#include <stdlib.h>
#include "stdio.h"	
void UART4_Printf(const char *format, ...);
#define RX4_BUFFER_SIZE 128

/* USART1 RX：CubeMX 生成的 hdma_usart1_rx（DMA1_Stream4, Circular）+ 空闲中断 */
#define USART1_RX_BUF_SIZE 128
void BSP_UART1_RxStart(void);
void BSP_UART1_IdleHandler(void);
void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len);
void BSP_UART1_SendPoll(const uint8_t *data, uint16_t len);  /* 轮询发送，ISR 安全，用于回显探针 */

#endif
