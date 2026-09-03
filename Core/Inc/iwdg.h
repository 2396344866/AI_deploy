/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   This file contains all the function prototypes for
  *          the iwdg.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern IWDG_HandleTypeDef hiwdg1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_IWDG1_Init(void);

/* USER CODE BEGIN Prototypes */

/* IWDG 启动（POST 开始时调用）——上电早期刻意不启动，原因见 iwdg.c 的 IWDG_Start()。
 * 总闸 APP_ENABLE_WATCHDOG=0 时本函数为空操作（调试关狗，等价于改造前行为）。 */
void IWDG_Start(void);
int  IWDG_IsRunning(void);   /* 1=IWDG 已在跑（喂狗接口生效）；0=未启动（喂狗为 no-op） */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H__ */

