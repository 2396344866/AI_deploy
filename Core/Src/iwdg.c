/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"

/* USER CODE BEGIN 0 */
#include "app_config.h"   /* APP_ENABLE_WATCHDOG：调试期关狗宏（单一真相源在 app_config.h） */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg1;

/* IWDG1 init function */
void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */
#if !defined(APP_ENABLE_WATCHDOG) || !APP_ENABLE_WATCHDOG
  return;  /* 调试关狗：跳过 IWDG 启动，固件不再因超时复位，便于暴露真实故障 */
#endif
  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* 覆盖 logger 的喂狗弱符号：崩溃落盘（logger_flush_to_flash）与常态喂狗共用此接口，
 * 防止 SPI/Flash 卡死或日志任务饿死导致独立看门狗触发不可逆复位。
 * 本工程已在 .ioc 启用 IWDG1（LSI 32kHz / 32分频 / Reload 4095 ≈ 4.1s 超时，
 * Window=4095 即不约束提前喂狗）。 */
void log_wdt_feed(void)
{
#if defined(APP_ENABLE_WATCHDOG) && APP_ENABLE_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg1);
#else
    (void)0;  /* 调试关狗：喂狗变 no-op（hiwdg1 未初始化，禁止触碰 IWDG 寄存器） */
#endif
}

/* USER CODE END 1 */

