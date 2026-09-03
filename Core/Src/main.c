/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "BSP_LOG.h"
#include "motor.h"
#include "attitude.h"       // 姿态解算 + 外环（Components/BSP/IMU）
#include "dbg_telemetry.h"  // 调试遥测（Components/Debug，UART1 firewater）
#include "logger.h"         // 启动文本日志用 #ifdef LOG_ENABLED 控制（生产模式变空）
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint8_t rx_buf[RX4_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
u32 UID_Word0,UID_Word1,UID_Word2;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 启动期复位源判定：每次上电第一行串口输出复位类别，直接回答"是不是 IWDG 复位"。
 * H743 的 RCC->RSR 复位标志位（stm32h743xx.h）：
 *   IWDG1RSTF=bit26  WWDG1RSTF=bit28  SFTRSTF=bit24  PORRSTF=bit23
 *   BORRSTF=bit21    PINRSTF=bit22    LPWRRSTF=bit30
 * 读后清除(RMVF)，便于下次判定。注意：运行期 IWDG 复位 = TIM7_IRQHandler 每 500ms 经
 * watchdog_should_feed() 判定不通过（被监视任务心跳不新鲜→某任务/ISR 冻结）；POST 期 =
 * 各 Xxx_Test 协作喂缺失（卡死→IWDG 抓到）。故本打印显示 IWDG1 时，真因是"某被监视任务/ISR
 * 冻结"，需结合下方 POST 标记 + TIM7 心跳 freshness 定位冻结点。 */
#ifdef LOG_ENABLED
static void print_reset_cause(void)
{
    uint32_t rsr = RCC->RSR;
    const char *src = "UNKNOWN";
    if      (rsr & RCC_RSR_IWDG1RSTF)  src = "IWDG1(独立看门狗超时->有任务/ISR 冻结饿死喂狗点)";
    else if (rsr & RCC_RSR_WWDG1RSTF)  src = "WWDG1(窗口看门狗)";
    else if (rsr & RCC_RSR_SFTRSTF)    src = "SFTRST(软件/NVIC_SystemReset)";
    else if (rsr & RCC_RSR_PORRSTF)    src = "POR(上电)";
    else if (rsr & RCC_RSR_BORRSTF)    src = "BOR(欠压)";
    else if (rsr & RCC_RSR_PINRSTF)    src = "PINRST(NRST引脚)";
    else if (rsr & RCC_RSR_LPWRRSTF)   src = "LPWR(低功耗退出)";
    printf("[BOOT] ResetSrc=%s  RSR=0x%08lX\r\n", src, (unsigned long)rsr);
    __HAL_RCC_CLEAR_RESET_FLAGS();   /* 清除，便于下次判定 */
}
#endif /* LOG_ENABLED */

#include "watchdog_heartbeat.h"   /* 看门狗心跳：POST 收尾 watchdog_arm() 后 TIM7 按任务心跳喂狗 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_IWDG1_Init();
  /* USER CODE BEGIN 2 */
#if !defined(APP_ENABLE_WATCHDOG) || !APP_ENABLE_WATCHDOG
  printf("[BOOT] IWDG DISABLED (debug) - reset masked off\r\n");
#else
  /* 上电早期刻意不启动 IWDG（防复位环），POST 开始才 IWDG_Start()，详见 iwdg.c */
  printf("[BOOT] IWDG armed from POST start (4.1s)\r\n");
#endif

#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
	// 1. DMA Circular
	HAL_UART_Receive_DMA(&huart4, rx_buf, RX4_BUFFER_SIZE);
	// 2. IDLE
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
#endif
	Dbg_Telemetry_Init();   /* 调试遥测：UART1 DMA 接收启动前用 DBG_UART_BAUD 重设波特率 */
	BSP_LOG_UART1_RxStart();
#ifdef LOG_ENABLED
	print_reset_cause();     /* 启动期复位源判定：每次上电首行报告 IWDG1/POR/BOR/... */
#endif
#if !defined(APP_ENABLE_MOTOR) || !APP_ENABLE_MOTOR
    /* 非 Motor profile（如 LOGGER）：Motor 模块未使能，motor.c 不会启动 TIM7；
     * 但看门狗 1ms 心跳依赖 TIM7，此处无条件启动，保证心跳与喂狗不耦合 Motor 是否编入。 */
    HAL_TIM_Base_Start_IT(&htim7);
#endif
	Motor_App_Init();        /* 启动 TIM1 PWM / TIM3-4 编码器 / TIM7 1ms 控制环 */
#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
	if (Attitude_Init() != 0) {
#ifdef LOG_ENABLED
		printf("WARN: Attitude/MPU6050 Init FAILED!\r\n");
#else
		(void)0;
#endif
	} else {
#ifdef LOG_ENABLED
		printf("MPU6050 Init OK\r\n");
#endif
	}         /* 初始化 MPU6050 + 姿态库（I2C1 已由 CubeMX 生成并初始化）；仅 Sensor profile 运行 */
#endif /* APP_ENABLE_SENSOR */
#ifdef LOG_ENABLED
	printf("System Init Success!\r\n");
#endif
    log_wdt_feed();   /* 调度器启动前先踢狗：上电初始化(Motor_App_Init/Attitude_Init)若偏长，
                         避免 boot 阶段 IWDG(≈4.1s)超时复位；喂狗只是写 IWDG 寄存器，启动前调用安全 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* TIM7 分支已迁至 stm32h7xx_it.c 的 TIM7_IRQHandler USER CODE 块，
     此处不再占用共享弱回调，避免与其它定时器的 period-elapsed 回调冲突。 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
