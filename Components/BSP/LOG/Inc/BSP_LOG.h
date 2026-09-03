#ifndef _BSP_LOG_H
#define _BSP_LOG_H

/* =============================================================================
 * BSP_LOG — 日志串口驱动（USART1 调试控制台 / VOFA+ 遥测）
 *   - UART4_Printf：格式化打印（VOFA+ 遥测 / 日志刷串口均走它）。
 *   - USART1：HAL_UARTEx_ReceiveToIdle_IT + 回调重武装收帧（BSP_LOG_UART1_RxStart /
 *     _OnRxEvent / _OnFrame），与 ESP(USART6/USART2) 同范式；HAL_UARTEx_RxEventCallback
 *     按 huart 分发；ISR 仅入队 g_cmd_qHandle，命令解析由常驻 StartLoggerTask 调
 *     DbgConsole_Process() 完成。
 *   命名/保护宏约定见 Components/BSP/README.md。
 * ============================================================================= */

#include "main.h"
#include "usart.h"

#include <string.h>
#include <stdlib.h>
#include "stdio.h"	
void UART4_Printf(const char *format, ...);
#define RX4_BUFFER_SIZE 128

/* USART1 RX：与 ESP 一致改用 HAL_UARTEx_ReceiveToIdle_IT（中断收帧，无 DMA），
 * 收完须回调内重武装；由 uart_rx_dispatcher.c 的 HAL_UARTEx_RxEventCallback 按
 * huart 分发到 BSP_LOG_UART1_OnRxEvent。.ioc 里 USART1_RX 的 DMA 流现已闲置可留可删。 */
#define USART1_RX_BUF_SIZE 128
void BSP_LOG_UART1_RxStart(void);
void BSP_LOG_UART1_OnRxEvent(uint16_t size);
void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len);
void BSP_LOG_UART1_SendPoll(const uint8_t *data, uint16_t len);  /* 轮询发送，ISR 安全，用于回显探针 */

/* 后端字节出口（弱符号，由 BSP_LOG.c 提供实现）。轮询直发、ISR 安全，
 * 用于 HardFault / UART 错误回调等故障上下文打点，不依赖 RTOS/HAL 状态。
 * 声明在此模块头，供 uart_rx_dispatcher.c / stm32h7xx_it.c 复用。 */
void log_backend_putc(char c);

#endif
