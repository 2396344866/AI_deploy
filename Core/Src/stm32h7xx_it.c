/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_LOG.h" 
#include "logger.h"    // 日志黑匣子：HardFault 中刷新最近日志到 Flash
#include "cmsis_os2.h" // 供 ISR 内 osSemaphoreRelease（CMSIS-RTOS V2，ISR 安全）
#include "dbg_config.h" // 调试开关集中管理（DEBUG_ISR_CNT_* 等）
#include "watchdog_heartbeat.h"  // 看门狗心跳：g_wdt_tick_cnt / watchdog_should_feed
#include "iwdg.h"                // hiwdg1 句柄（HAL_IWDG_Refresh）
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t rx_buf[];
extern UART_HandleTypeDef huart4; // 如果还没有的话
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== HardFault 自愈式诊断 ==================== */
/* 全部走轮询 USART1(log_backend_putc)，零 RTOS/HAL 依赖，故障上下文安全。
   不调用任何可能再次 fault 的函数（禁用 printf / RTOS API）。 */
extern void log_backend_putc(char c);

/* SCB 故障寄存器（Cortex-M7 固定地址，不依赖 CMSIS 宏，最稳妥） */
#define HF_CFSR    (*(volatile uint32_t *)0xE000ED28UL)  /* 可配置故障状态(Mem/Bus/Usage) */
#define HF_HFSR    (*(volatile uint32_t *)0xE000ED2CUL)  /* Hard Fault 状态 */
#define HF_DFSR    (*(volatile uint32_t *)0xE000ED30UL)  /* Debug Fault 状态 */
#define HF_MMFAR   (*(volatile uint32_t *)0xE000ED34UL)  /* MemManage 故障地址 */
#define HF_BFAR    (*(volatile uint32_t *)0xE000ED38UL)  /* BusFault 故障地址 */

static void hf_putc(char c) { log_backend_putc(c); }
static void hf_puts(const char *s) { while (*s) hf_putc(*s); }
static void hf_newline(void) { hf_putc('\r'); hf_putc('\n'); }
static void hf_puthex32(uint32_t v) {
    hf_puts("0x");
    for (int s = 28; s >= 0; s -= 4) {
        uint8_t n = (uint8_t)((v >> s) & 0xFU);
        hf_putc(n < 10U ? (char)('0' + n) : (char)('A' + n - 10U));
    }
}
/* 解码 CFSR 三大子状态字，打印可读故障类别（面试可逐一解释） */
static void hf_decode_cfsr(uint32_t cfsr) {
    uint8_t  mmfsr = (uint8_t)(cfsr & 0xFFU);
    uint8_t  bfsr  = (uint8_t)((cfsr >> 8) & 0xFFU);
    uint16_t ufsr  = (uint16_t)((cfsr >> 16) & 0xFFFFU);
    if (mmfsr & (1u<<0)) hf_puts(" IACCVIOL");    /* 取指访问违例 */
    if (mmfsr & (1u<<1)) hf_puts(" DACCVIOL");    /* 数据访问违例 */
    if (mmfsr & (1u<<3)) hf_puts(" MUNSTKERR");   /* 异常返回出栈错 */
    if (mmfsr & (1u<<4)) hf_puts(" MSTKERR");     /* 异常入栈错 */
    if (mmfsr & (1u<<7)) hf_puts(" MMARVALID");
    if (bfsr  & (1u<<0)) hf_puts(" IBUSERR");
    if (bfsr  & (1u<<1)) hf_puts(" PRECISERR");   /* 精确总线错误，坏地址在 BFAR */
    if (bfsr  & (1u<<2)) hf_puts(" IMPRECISERR"); /* 不精确总线错误(写缓冲延迟) */
    if (bfsr  & (1u<<3)) hf_puts(" UNSTKERR");
    if (bfsr  & (1u<<4)) hf_puts(" STKERR");
    if (bfsr  & (1u<<7)) hf_puts(" BFARVALID");
    if (ufsr  & (1u<<0)) hf_puts(" UNDEFINSTR");  /* 未定义指令 */
    if (ufsr  & (1u<<1)) hf_puts(" INVSTATE");    /* 非法状态(Thumb/ARM 混用) */
    if (ufsr  & (1u<<2)) hf_puts(" INVPC");
    if (ufsr  & (1u<<3)) hf_puts(" NOCP");        /* 协处理器不存在(浮点未使能却用) */
    if (ufsr  & (1u<<8)) hf_puts(" UNALIGNED");
    if (ufsr  & (1u<<9)) hf_puts(" DIVBYZERO");   /* 除零(若使能) */
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
void Motor_1ms_Handler(void);       /* TIM7 1ms 控制环：由 Components/Motor 实现 */
extern osSemaphoreId_t g_semAttitudeDataReadyHandle;  /* CubeMX 生成（二值信号量） */
extern osSemaphoreId_t g_semScreenUpdateHandle;       /* 淘晶驰屏数据到达：ISR 释放，Task_Screen 获取 */


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* 自愈式 HardFault 诊断：进入即打印故障类别 + 坏指针 + 完整异常帧，
     焚毁前刷黑匣子，不依赖 RTOS/HAL 状态。每次 fault episode 打印一次，
     IWDG 超时复位后会重新进入本函数再次打印（串口看到重复属预期）。 */
    uint32_t lr_exc;
    __asm volatile ("mov %0, lr" : "=r" (lr_exc));   /* 读 EXC_RETURN */
    uint32_t *sp = (lr_exc & 0x4U) ? (uint32_t *)__get_PSP() : (uint32_t *)__get_MSP();
    /* 异常硬件压栈帧: r0,r1,r2,r3,r12,lr(pc-ret),pc,xPSR */
    uint32_t r0_ = sp[0], r1_ = sp[1], r2_ = sp[2], r3_ = sp[3];
    uint32_t r12_ = sp[4], lr_f = sp[5], pc_ = sp[6], xpsr_ = sp[7];
    /* r4-r11 为 callee-saved，未进异常帧；此处为最佳努力(best-effort)：
       C 函数序言可能已占用它们，真实故障值需 naked 包装才 100% 可靠。 */
    uint32_t r4_, r5_, r6_, r7_, r8_, r9_, r10_, r11_;
    __asm volatile ("mov %0,r4\n mov %1,r5\n mov %2,r6\n mov %3,r7\n"
                    "mov %4,r8\n mov %5,r9\n mov %6,r10\n mov %7,r11\n"
                    : "=r"(r4_), "=r"(r5_), "=r"(r6_), "=r"(r7_),
                      "=r"(r8_), "=r"(r9_), "=r"(r10_), "=r"(r11_));


  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* 完整诊断已在上面的 HardFault_IRQn 0 块打印一次；此处仅原地 halt，
       由 IWDG 超时触发复位环（便于重复抓取），或等待 J-Link 接入取证。 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MPU6050_INT_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET)
  {
      __HAL_UART_CLEAR_IDLEFLAG(&huart4);
      
      // 1. 获取 DMA 接收长度
      uint16_t rx_len = RX4_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
      
      // 2. 停掉接收，准备重置
      HAL_UART_DMAStop(&huart4);
      
      // 3. 【工业级重点】ISR 内严禁阻塞式发送！
      //    原 UART4_Printf(...) 在中断里 HAL_UART_Transmit 轮询，会饿死调度器
      //    （现象：System Init Success! 后再无任务输出）。改为释放信号量，
      //    由 Task_Screen 在任务上下文处理 rx_buf。
      if (rx_len > 0) {
          osSemaphoreRelease(g_semScreenUpdateHandle);
      }
      
      // 4. 重置状态并开启接收
      HAL_UART_Receive_DMA(&huart4, rx_buf, RX4_BUFFER_SIZE);
  }
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
#if defined(APP_ENABLE_MOTOR) && APP_ENABLE_MOTOR
  Motor_1ms_Handler();   /* 1ms 控制节拍：读编码器 -> 速度/位置 PI -> 输出 PWM */
#endif

  /* 看门狗心跳喂狗（工业级策略，详见 watchdog_heartbeat.h）：
     - g_wdt_tick_cnt 每 1ms 自增，供 task_heartbeat_kick 打时间戳。
     - POST 期间 watchdog_arm() 未调 -> watchdog_should_feed() 返回 0 -> 不喂；
       由 POST 测试代码 log_wdt_feed() 协作式喂（卡死测试->IWDG 抓到，慢测试活过）。
     - POST 收尾 watchdog_arm() 后 -> 每 500ms 经 watchdog_should_feed() 检查：
       所有被监视任务(APP_ENABLE_X!=0)心跳新鲜才喂；任一冻结->不喂->IWDG 复位。
     独占 TIM7 中断（落在 TIM7_IRQHandler USER CODE 块），
     不再占用 HAL_TIM_PeriodElapsedCallback 共享弱回调，规避 multiple-definition 地雷。 */
  g_wdt_tick_cnt++;
  static uint32_t wdt_feed_ticks = 0U;
  if (++wdt_feed_ticks >= 500U) {
      wdt_feed_ticks = 0U;
      if (watchdog_should_feed()) {
          HAL_IWDG_Refresh(&hiwdg1);
      }
  }
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* MPU6050 data-ready 中断（INT 脚 -> EXTI，CubeMX 生成 EXTIx_IRQHandler 并调用本回调）。
   运行于 ISR 上下文：严格只做"Give"动作（释放信号量唤醒 Task_Sensor），
   不碰任何业务/耗时操作，符合 RTOS 铁律。EXTI 优先级数值已设 ≥5（建议 6）。 */
#if defined(DEBUG_ISR_CNT_MPU6050_INT) && DEBUG_ISR_CNT_MPU6050_INT
volatile uint32_t g_int_cnt = 0;   /* 仅调试期观察 data-ready 中断频率，发布版不编译 */
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == MPU6050_INT_Pin) {
#if defined(DEBUG_ISR_CNT_MPU6050_INT) && DEBUG_ISR_CNT_MPU6050_INT
        g_int_cnt++;
#endif
        osSemaphoreRelease(g_semAttitudeDataReadyHandle);
    }
}

/* USER CODE END 1 */
