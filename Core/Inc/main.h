/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define RED_Pin GPIO_PIN_0
#define RED_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_1
#define GREEN_GPIO_Port GPIOC
#define BLUE_Pin GPIO_PIN_2
#define BLUE_GPIO_Port GPIOC
#define MPU6050_INT_Pin GPIO_PIN_3
#define MPU6050_INT_GPIO_Port GPIOC
#define MPU6050_INT_EXTI_IRQn EXTI3_IRQn
#define TJC_HMI_TX_Pin GPIO_PIN_0
#define TJC_HMI_TX_GPIO_Port GPIOA
#define TJC_HMI_RX_Pin GPIO_PIN_1
#define TJC_HMI_RX_GPIO_Port GPIOA
#define ESP01S_TX_Pin GPIO_PIN_2
#define ESP01S_TX_GPIO_Port GPIOA
#define ESP01S_RX_Pin GPIO_PIN_3
#define ESP01S_RX_GPIO_Port GPIOA
#define W25Q64_CS_Pin GPIO_PIN_4
#define W25Q64_CS_GPIO_Port GPIOA
#define W25Q64_SCK_Pin GPIO_PIN_5
#define W25Q64_SCK_GPIO_Port GPIOA
#define W25Q64_DO_Pin GPIO_PIN_6
#define W25Q64_DO_GPIO_Port GPIOA
#define W25Q64_DI_Pin GPIO_PIN_7
#define W25Q64_DI_GPIO_Port GPIOA
#define TB6612_AIN1_Pin GPIO_PIN_0
#define TB6612_AIN1_GPIO_Port GPIOB
#define TB6612_AIN2_Pin GPIO_PIN_1
#define TB6612_AIN2_GPIO_Port GPIOB
#define TB6612_BIN1_Pin GPIO_PIN_2
#define TB6612_BIN1_GPIO_Port GPIOB
#define MOTOR_TT_PWMB_TIMER_Pin GPIO_PIN_11
#define MOTOR_TT_PWMB_TIMER_GPIO_Port GPIOE
#define MAX485_TX_Pin GPIO_PIN_10
#define MAX485_TX_GPIO_Port GPIOB
#define MAX485_RX_Pin GPIO_PIN_11
#define MAX485_RX_GPIO_Port GPIOB
#define TB6612_STBY_Pin GPIO_PIN_12
#define TB6612_STBY_GPIO_Port GPIOB
#define MAX485_DE_Pin GPIO_PIN_14
#define MAX485_DE_GPIO_Port GPIOB
#define ESP32S3_TX_Pin GPIO_PIN_6
#define ESP32S3_TX_GPIO_Port GPIOC
#define ESP32S3_RX_Pin GPIO_PIN_7
#define ESP32S3_RX_GPIO_Port GPIOC
#define MOTOR_TT_PWMA_TIMER_Pin GPIO_PIN_8
#define MOTOR_TT_PWMA_TIMER_GPIO_Port GPIOA
#define LOG_TX_Pin GPIO_PIN_9
#define LOG_TX_GPIO_Port GPIOA
#define LOG_RX_Pin GPIO_PIN_10
#define LOG_RX_GPIO_Port GPIOA
#define TJA1050_TX_Pin GPIO_PIN_11
#define TJA1050_TX_GPIO_Port GPIOA
#define TJA1050_RX_Pin GPIO_PIN_12
#define TJA1050_RX_GPIO_Port GPIOA
#define TB6612_BIN2_Pin GPIO_PIN_3
#define TB6612_BIN2_GPIO_Port GPIOB
#define MOTOR_Right_ENC_TIMER_A_Pin GPIO_PIN_4
#define MOTOR_Right_ENC_TIMER_A_GPIO_Port GPIOB
#define MOTOR_Right_ENC_TIMER_B_Pin GPIO_PIN_5
#define MOTOR_Right_ENC_TIMER_B_GPIO_Port GPIOB
#define MOTOR_Left_ENC_TIMER_A_Pin GPIO_PIN_6
#define MOTOR_Left_ENC_TIMER_A_GPIO_Port GPIOB
#define MOTOR_Left_ENC_TIMER_B_Pin GPIO_PIN_7
#define MOTOR_Left_ENC_TIMER_B_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_8
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_9
#define MPU6050_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
