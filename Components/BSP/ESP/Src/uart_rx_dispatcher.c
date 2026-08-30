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
#include "esp32s3.h"     /* ESP32S3_UART_RxCallback */
#include "esp01s.h"      /* ESP01S_UART_RxCallback */
#include "BSP_LOG.h"     /* BSP_LOG_UART1_OnRxEvent */

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
