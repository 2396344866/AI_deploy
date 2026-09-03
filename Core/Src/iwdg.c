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
  /* 上电早期不启动 IWDG：真正启动见 IWDG_Start()（POST 开始时调用）。
   * 本阶段（外设初始化/Motor_App_Init/Attitude_Init/起调度器）耗时不可控且无喂狗点，
   * 若此时狗已在跑，任一步偏慢超 4.1s 即被判死 → 复位 → 重跑 → 复位环。 */
  return;
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

/* ===========================================================================
 * IWDG 三阶段监管（根治"上电即复位环"）
 * ---------------------------------------------------------------------------
 * IWDG 一旦启动，硬件上无法关闭（仅复位能停）。因此监管范围必须与"喂狗点是否
 * 就绪"严格对齐，否则必然把"慢"误判为"死"：
 *
 *   ① 上电 → POST 开始   ：不启动 IWDG。外设初始化 / Motor_App_Init /
 *                          Attitude_Init / 起调度器耗时不可控且无喂狗点。
 *   ② POST 期间          ：IWDG_Start() 起跑（≈4.1s），各 Xxx_Test 用
 *                          log_wdt_feed() 协作喂 → 真卡死仍能被抓到复位。
 *   ③ POST 收尾          ：watchdog_arm() 后 TIM7 心跳接管，要求所有被监视
 *                          任务（APP_ENABLE_X）持续 kick 才喂；任一冻结即复位。
 *
 * 启动总闸仍是 APP_ENABLE_WATCHDOG（=0 时 IWDG_Start() 为空操作，等价改造前）。
 * 参数：.ioc 配置 LSI 32kHz / 32 分频 / Reload 4095 ≈ 4.1s，Window=4095（不约束提前喂）。
 * =========================================================================== */
static uint8_t s_iwdg_running = 0;

void IWDG_Start(void)
{
#if defined(APP_ENABLE_WATCHDOG) && APP_ENABLE_WATCHDOG
    if (s_iwdg_running) return;
    hiwdg1.Instance       = IWDG1;
    hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg1.Init.Window    = 4095;   /* 不约束提前喂狗 */
    hiwdg1.Init.Reload    = 4095;   /* ≈4.1s 超时 */
    if (HAL_IWDG_Init(&hiwdg1) != HAL_OK) { Error_Handler(); }
    s_iwdg_running = 1;
#endif
}

int IWDG_IsRunning(void) { return (int)s_iwdg_running; }

/* 覆盖 logger 的喂狗弱符号（logger.c 有 __weak 空实现）：崩溃落盘
 * （logger_flush_to_flash）与常态协作喂共用此接口，防 SPI/Flash 卡死或
 * 日志任务饿死触发不可逆复位。
 * IWDG 未启动（阶段① 或关狗态）时是 no-op —— 显式跳过而非盲写寄存器，
 * 避免"以为在喂狗、实则狗没跑"的错觉。 */
void log_wdt_feed(void)
{
    if (s_iwdg_running) { HAL_IWDG_Refresh(&hiwdg1); }
}

/* USER CODE END 1 */

