/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_GPIO.h" // GPIO
#include "BSP_USART.h"// UART 整理
#include "BSP_W25Q64.h"// UART 整理
#include "logger.h"    // 统一日志模块（分级/时间戳/标签/编译开关）
#include "motor.h"     // 双电机 速度/位置闭环（阶段1）
#include "arm_math.h"
#include "model_weights.h"
#include "ai_infer.h"
#include "test_dataset_processed.h"
#include "hardfault_lab.h"   // HardFault 实验室（仅当 HARDFAULT_LAB 宏开启时调用/链接，模块在 Components/HardFaultLab/）
#include "attitude.h"        // 姿态解算 + 外环控制器（Components/BSP/IMU）
#include "imu_mpu6050.h"     // MPU6050 底层 I2C 读取
#include "dbg_telemetry.h"  // 调试遥测（UART1 firewater，见 Components/Debug）
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
/* USER CODE BEGIN Variables */

/* 命令通道：改用 CubeMX 生成的消息队列 g_cmd_qHandle（ItemSize=32, Size=4）。
   ISR(BSP_UART1_OnFrame) 只做 osMessageQueuePut，任务(StartMotorTask) 用
   osMessageQueueGet 收 —— 不再使用裸 volatile 标志 + 临界区拷贝模型。 */

typedef struct {
		uint32_t num_test_samples;
    float overall_accuracy;
    float macro_precision;
    float macro_recall;
    float macro_f1;
    float total_time_ms;
    uint8_t data_is_ready; // 标志位：告诉网络任务数据是否已准备好
} TestResults_t;

TestResults_t g_Test_results = {0}; // 全局共享变量

/* USER CODE END Variables */
/* Definitions for Task_Inference */
osThreadId_t Task_InferenceHandle;
const osThreadAttr_t Task_Inference_attributes = {
  .name = "Task_Inference",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Motor */
osThreadId_t Task_MotorHandle;
const osThreadAttr_t Task_Motor_attributes = {
  .name = "Task_Motor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Network */
osThreadId_t Task_NetworkHandle;
const osThreadAttr_t Task_Network_attributes = {
  .name = "Task_Network",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Sensor */
osThreadId_t Task_SensorHandle;
const osThreadAttr_t Task_Sensor_attributes = {
  .name = "Task_Sensor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Screen */
osThreadId_t Task_ScreenHandle;
const osThreadAttr_t Task_Screen_attributes = {
  .name = "Task_Screen",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Flash */
osThreadId_t Task_FlashHandle;
const osThreadAttr_t Task_Flash_attributes = {
  .name = "Task_Flash",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Task_logger */
osThreadId_t Task_loggerHandle;
const osThreadAttr_t Task_logger_attributes = {
  .name = "Task_logger",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for g_cmd_q */
osMessageQueueId_t g_cmd_qHandle;
const osMessageQueueAttr_t g_cmd_q_attributes = {
  .name = "g_cmd_q"
};
/* Definitions for InferenceDataMutex */
osMutexId_t InferenceDataMutexHandle;
const osMutexAttr_t InferenceDataMutex_attributes = {
  .name = "InferenceDataMutex"
};
/* Definitions for g_semScreenUpdate */
osSemaphoreId_t g_semScreenUpdateHandle;
const osSemaphoreAttr_t g_semScreenUpdate_attributes = {
  .name = "g_semScreenUpdate"
};
/* Definitions for g_semInferenceLock */
osSemaphoreId_t g_semInferenceLockHandle;
const osSemaphoreAttr_t g_semInferenceLock_attributes = {
  .name = "g_semInferenceLock"
};
/* Definitions for g_semFlashDmaDone */
osSemaphoreId_t g_semFlashDmaDoneHandle;
const osSemaphoreAttr_t g_semFlashDmaDone_attributes = {
  .name = "g_semFlashDmaDone"
};
/* Definitions for g_semAttitudeDataReady */
osSemaphoreId_t g_semAttitudeDataReadyHandle;
const osSemaphoreAttr_t g_semAttitudeDataReady_attributes = {
  .name = "g_semAttitudeDataReady"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int DIV2IFSCN_Inference(const float* input, float* outputs);
void W25QXX_Test(void);
/* StartLoggerTask 的原型由 CubeMX 在下方（USER CODE 块外）统一生成，此处不再重复声明 */
/* USER CODE END FunctionPrototypes */

void StartInferenceTask(void *argument);
void StartMotorTask(void *argument);
void StartNetworkTask(void *argument);
void StartSensorTask(void *argument);
void StartScreenTask(void *argument);
void StartFlashTask(void *argument);
void StartLoggerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  logger_init();   /* 初始化日志环形缓冲（必须在任务创建前） */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of InferenceDataMutex */
  InferenceDataMutexHandle = osMutexNew(&InferenceDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of g_semScreenUpdate */
  g_semScreenUpdateHandle = osSemaphoreNew(1, 0, &g_semScreenUpdate_attributes);

  /* creation of g_semInferenceLock */
  g_semInferenceLockHandle = osSemaphoreNew(1, 0, &g_semInferenceLock_attributes);

  /* creation of g_semFlashDmaDone */
  g_semFlashDmaDoneHandle = osSemaphoreNew(1, 0, &g_semFlashDmaDone_attributes);

  /* creation of g_semAttitudeDataReady */
  g_semAttitudeDataReadyHandle = osSemaphoreNew(1, 1, &g_semAttitudeDataReady_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 所有二值信号量已统一由 CubeMX 生成（g_sem*Handle），此处不再手工创建，避免重复定义 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of g_cmd_q */
  g_cmd_qHandle = osMessageQueueNew (4, 32, &g_cmd_q_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_Inference */
  Task_InferenceHandle = osThreadNew(StartInferenceTask, NULL, &Task_Inference_attributes);

  /* creation of Task_Motor */
  Task_MotorHandle = osThreadNew(StartMotorTask, NULL, &Task_Motor_attributes);

  /* creation of Task_Network */
  Task_NetworkHandle = osThreadNew(StartNetworkTask, NULL, &Task_Network_attributes);

  /* creation of Task_Sensor */
  Task_SensorHandle = osThreadNew(StartSensorTask, NULL, &Task_Sensor_attributes);

  /* creation of Task_Screen */
  Task_ScreenHandle = osThreadNew(StartScreenTask, NULL, &Task_Screen_attributes);

  /* creation of Task_Flash */
  Task_FlashHandle = osThreadNew(StartFlashTask, NULL, &Task_Flash_attributes);

  /* creation of Task_logger */
  Task_loggerHandle = osThreadNew(StartLoggerTask, NULL, &Task_logger_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* 日志后台任务 Task_logger 已由 CubeMX 原生生成（见上方 "creation of Task_logger"，
     osThreadNew 在 freertos.c:238），此处删去手写重复创建，避免同一任务被建两次。 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInferenceTask */
/**
  * @brief  Function implementing the Task_AL_deploy thread.
  * @param  argument: Not used
  * @rettest None
  */
/* USER CODE END Header_StartInferenceTask */
void StartInferenceTask(void *argument)
{
  /* USER CODE BEGIN StartInferenceTask */
  /* Infinite loop */
	// 1. 初始化 (只运行一次)
  // 确保推理函数内部的初始化标志已经重置或能够正常运行
  float dummy_out[4];
  AI_Inference((float*)test_features_processed[0], dummy_out);
#if DBG_LOG_ENABLE
  LOG_I("INFER", "AI Task Started. Beginning Batch Inference...");
#endif
  int tp[4] = {0}, fp[4] = {0}, fn[4] = {0};
  int correct_total = 0;
  uint32_t start_time = HAL_GetTick(); 

  // 2. 推理循环
  for(int i = 0; i < NUM_TEST_SAMPLES; i++) {
      float test_output[4] = {0};
      int pred = AI_Inference((float*)test_features_processed[i], test_output);
      int actual = test_labels[i];

      if (pred == actual) { correct_total++; tp[actual]++; }
      else { fp[pred]++; fn[actual]++; }
      
      // 重要：在 RTOS 中必须让出 CPU，否则优先级低的任务无法运行
      if(i % 10 == 0) osDelay(1); 
  }

  // 3. 结果打印
  uint32_t end_time = HAL_GetTick();
  float total_time_ms = (float)(end_time - start_time);
	
	
	float macro_precision = 0.0f;
  float macro_recall = 0.0f;
  float macro_f1 = 0.0f;

  for (int i = 0; i < 4; i++) {
      float p = 0.0f;
      float r = 0.0f;
      
      // 计算 Precision
      if ((tp[i] + fp[i]) > 0) {
          p = (float)tp[i] / (tp[i] + fp[i]);
      }
      // 计算 Recall
      if ((tp[i] + fn[i]) > 0) {
          r = (float)tp[i] / (tp[i] + fn[i]);
      }

      macro_precision += p;
      macro_recall += r;

      // 计算 F1-Score
      if ((p + r) > 0) {
          macro_f1 += (2.0f * p * r) / (p + r);
      }
  }
  // 取平均值
  macro_precision /= 4.0f;
  macro_recall /= 4.0f;
  macro_f1 /= 4.0f;
	float overall_accuracy = (float)correct_total / NUM_TEST_SAMPLES;
  /* 推理结果汇总：分级日志（INFO），由 LoggerTask 异步刷串口 */
#if DBG_LOG_ENABLE
  LOG_I("INFER", "Summary: samples=%d acc=%.4f prec=%.4f recall=%.4f f1=%.4f total=%.2fms per=%.4fms",
        NUM_TEST_SAMPLES, overall_accuracy, macro_precision, macro_recall, macro_f1,
        total_time_ms, total_time_ms / NUM_TEST_SAMPLES);
#endif
  /* 写入共享结构，互斥量保护，供 NetworkTask 读取（生产逻辑，必须保留） */
  if (osMutexAcquire(InferenceDataMutexHandle, osWaitForever) == osOK) {
      g_Test_results.num_test_samples = NUM_TEST_SAMPLES;
      g_Test_results.overall_accuracy = overall_accuracy;
      g_Test_results.macro_precision = macro_precision;
      g_Test_results.macro_recall = macro_recall;
      g_Test_results.macro_f1 = macro_f1;
      g_Test_results.total_time_ms = total_time_ms;
      g_Test_results.data_is_ready = 1;
      osMutexRelease(InferenceDataMutexHandle);
  }

  /* 推理完成，任务进入空闲（优先级继承调试已移除） */
  for(;;) {
    osDelay(1000);
  }
  /* USER CODE END StartInferenceTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the Task_Motor thread.
* @param argument: Not used
* @rettest None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* 电机控制任务（阶段1：速度/位置闭环已在 TIM7 1ms 中断完成）。
     本任务职责：
       1. PC13 按键（低电平）：短按切换 运行/刹车；
       2. 每秒打印一次双电机速度/位置/估算转速（仅供联调，发布版可关）。 */
#if DBG_LOG_ENABLE
  LOG_I("MOTOR", "Motor task started. Press KEY(PC13) to toggle run/brake.");
#endif
  uint32_t log_cnt = 9;   /* 首行遥测提前到 ~100ms 后，便于一眼确认任务存活 */

  /* 串口命令采用"ISR 入队 + 任务出队"模型（见 BSP_UART1_OnFrame → g_cmd_qHandle）。
     命令解析在任务上下文，可安全 LOG / 改电机状态；回显也在此处完成。 */

  for(;;) {
    /* 1. 按键：低电平触发（PC13 已上拉）。做简单消抖：连续两次读到低才动作 */
    static uint8_t key_prev = 1, key_stable = 1;
    uint8_t key_now = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    if (key_now == key_prev) {
        if (key_now != key_stable) {            /* 状态稳定变化 */
            key_stable = key_now;
            if (key_stable == 0) {              /* 下降沿：切换运行/刹车 */
                if (g_motor_sys.running) Motor_EmergencyStop();
                else                    Motor_Resume();
            }
        }
    }
    key_prev = key_now;

    /* 2. 串口命令：从 CubeMX 生成的 g_cmd_qHandle 出队（非阻塞，避免空等拖慢 100ms 周期） */
    char lbuf[32];
    if (osMessageQueueGet(g_cmd_qHandle, lbuf, NULL, 0U) == osOK) {
        uint16_t n = 0U;
        while (lbuf[n] != '\0' && n < 31U) n++;   /* 计算命令长度（无 string.h 依赖） */
        if (lbuf[0] == 'T' || lbuf[0] == 'P' || lbuf[0] == 'K' ||
            lbuf[0] == 'C' || lbuf[0] == 'F' || lbuf[0] == 'D' || lbuf[0] == 'M') {
            Attitude_ProcessCommand(lbuf, n);      /* 姿态外环命令（T/P/K/C/F/D） */
        } else {
            Motor_ProcessCommand(lbuf, n);         /* 电机命令（A/B/S/R） */
        }
        BSP_UART1_SendPoll((const uint8_t *)lbuf, n);  /* 回显：从 ISR 移到任务，符合范式 */
    }

    /* 3. 每秒打印一次遥测（DBG_LOG_ENABLE=0 时整体关闭，避免干扰 VOFA 波形） */
#if DBG_LOG_ENABLE
    if (++log_cnt >= 10) {   /* 10 * 100ms = 1s */
        log_cnt = 0;
        LOG_I("MOTOR", "A spd=%ld pwm=%ld rpm=%.1f | B spd=%ld pwm=%ld rpm=%.1f | run=%d mode=%d",
              Motor_GetSpeed(MOTOR_A), Motor_GetPWM(MOTOR_A), Motor_GetRPM(MOTOR_A),
              Motor_GetSpeed(MOTOR_B), Motor_GetPWM(MOTOR_B), Motor_GetRPM(MOTOR_B),
              g_motor_sys.running, g_motor_sys.mode);

        /* 无仪表探针：回读关键引脚实际电平，确认 PWM/STBY 是否真正出现在物理引脚上 */
        uint32_t hiA = 0, hiB = 0, ticks = 0;
        Motor_Probe_ReadAndClear(&hiA, &hiB, &ticks);
        LOG_I("PROBE", "STBY=%d PWMA_hi=%lu/%lu PWMB_hi=%lu/%lu TIM1span=%lu ENC_A(PB6)=%d ENC_B(PB7)=%d ENC_C(PB4)=%d ENC_D(PB5)=%d",
              HAL_GPIO_ReadPin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin),
              hiA, ticks, hiB, ticks, Motor_Probe_Tim1Span(),
              HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7),
              HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5));
    }
#endif

    osDelay(100);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartNetworkTask */
/**
* @brief Function implementing the Task_Network thread.
* @param argument: Not used
* @rettest None
*/
/* USER CODE END Header_StartNetworkTask */
void StartNetworkTask(void *argument)
{
  /* USER CODE BEGIN StartNetworkTask */
  /* Infinite loop */
  for(;;)
  {
    if (g_Test_results.data_is_ready == 1) {
        if (osMutexAcquire(InferenceDataMutexHandle, osWaitForever) == osOK) {
					

            // 安全读取数据并打印
						#if DBG_LOG_ENABLE
            LOG_I("NET", "Summary (From Network Task): acc=%.4f prec=%.4f recall=%.4f f1=%.4f total=%.2fms",
                  g_Test_results.overall_accuracy, g_Test_results.macro_precision,
                  g_Test_results.macro_recall, g_Test_results.macro_f1,
                  g_Test_results.total_time_ms);
						#endif
            
            g_Test_results.data_is_ready = 0; // 打印后重置标志
            osMutexRelease(InferenceDataMutexHandle);
        }
    }
    osDelay(100); // 不要让任务空转，给其他任务机会
  }
  /* USER CODE END StartNetworkTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the Task_Sensor thread.
* @param argument: Not used
* @rettest None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
	
#ifdef HARDFAULT_LAB
  /* 测试故障诊断用 测试故障诊断用 测试故障诊断用 测试故障诊断用
		 HardFault 实验室（v3.2）：配置四级中断抢占 + MPU 栈守卫带，
     触发四层中断嵌套级联，使嵌套压栈越过守卫带 -> MemManage 违规 ->
     升级为 HardFault。详见 Components/HardFaultLab/（hardfault_lab.c / HardFaultLab_SOP.md）。
     关闭 HARDFAULT_LAB 宏即不编译、不链接，工程行为不变。 */
  HardFaultLab_Run();
#endif
  if (MPU6050_EnableInt() != 0) { printf("WARN: MPU6050 data-ready INT enable FAILED!\r\n"); }   /* 启动 MPU6050 data-ready 中断（放调度器启动后，避免 200Hz ISR 在内核未起时冲击） */
  /* 姿态外环：等 MPU6050 data-ready 中断（ISR 释放信号量）-> 读 -> 滤波 -> 融合 -> 外环PID -> VOFA
     频率由传感器 INT（ATTITUDE_RATE_HZ）决定；osSemaphoreAcquire 带超时看门狗，
     中断未配置/丢失时不永久阻塞。ISR 只 Give 信号量，业务全在此任务，符合 RTOS 铁律。 */
  int16_t ra[3], rg[3];
  ImuData_t imu;
  int32_t tgtA = 0, tgtB = 0;
  for (;;)
  {
    if (osSemaphoreAcquire(g_semAttitudeDataReadyHandle, 100U) != osOK) {
        continue;                       /* 超时：跳过本拍，不阻塞 */
    }
    if (MPU6050_ReadRaw(ra, rg, NULL) != 0) continue;   /* I2C 异常：丢弃本拍 */
    ImuFilter_Update(ra, rg, &imu);
    Attitude_Update(&imu);
    Attitude_RunController();
    Attitude_GetTargets(&tgtA, &tgtB);
    Dbg_Telemetry_Send(&imu, Attitude_Get(), tgtA, tgtB);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartScreenTask */
/**
* @brief Function implementing the Task_Screen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartScreenTask */
void StartScreenTask(void *argument)
{
  /* USER CODE BEGIN StartScreenTask */
	
//	W25QXX_Test();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartScreenTask */
}

/* USER CODE BEGIN Header_StartFlashTask */
/**
* @brief Function implementing the Task_Flash thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlashTask */
void StartFlashTask(void *argument)
{
  /* USER CODE BEGIN StartFlashTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFlashTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the Task_logger thread.
* @param argument: Not used
* @retval None
*/
/* ===========================================================================
 * 日志后台任务 = CubeMX 原生 Task_logger（Entry Function = StartLoggerTask）。
 *   - 函数体位于 CubeMX 生成的 "USER CODE BEGIN StartLoggerTask" stub（freertos.c:521），
 *     CubeMX 重新生成时自动保留，无需手写。
 *   - 任务声明与 osThreadNew 调用也由 CubeMX 生成（freertos.c:164 / :233）。
 * 历史：早期曾把函数体写在生成主体外（USER CODE 块），重生时被冲掉，
 *       链接器报 "L6218E: Undefined symbol StartLoggerTask"；现已整体归 CubeMX 管理，问题消失。
 * 职责：低优先级循环，把 logger 环形缓冲异步刷到串口（logger_drain）。
 *       生产任务只做"格式化+入缓冲"，不阻塞，实时性不受 printf 影响。
 *       logger_init() 已在 MX_FREERTOS_Init 的 USER CODE BEGIN Init 调用。
 * =========================================================================== */
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  (void)argument;
  for (;;) {
      logger_drain();    /* 阻塞刷环形缓冲（跑在低优先级任务里，不抢占控制环） */
      osDelay(2);        /* 释放 CPU，让高优先级任务（Motor/Inference）先跑 */
  }
  /* USER CODE END StartLoggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 帧接收回调：覆盖 BSP_USART.c 的 __weak 默认实现。
   运行于 ISR 上下文（USART1_IDLE 中断），只把整帧拷入本地缓冲并推入
   CubeMX 生成的 g_cmd_qHandle 队列；命令解析与回显留给 StartMotorTask，
   严格符合参考范式"ISR 只 Send/Give，不碰业务"。
   USART1_IRQn 优先级=8 ≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，
   故 osMessageQueuePut 在 ISR 中调用合法；队满(4 槽)则丢弃，不阻塞中断。 */
void BSP_UART1_OnFrame(const uint8_t *data, uint16_t len)
{
    if (len == 0U || len > 31U) return;          /* 越界保护，队列 ItemSize=32(含'\0') */
    char item[32];
    for (uint16_t i = 0U; i < len; i++) item[i] = (char)data[i];
    item[len] = '\0';
    osMessageQueuePut(g_cmd_qHandle, item, 0U, 0U);   /* ISR 安全，非阻塞 */
}

/* USER CODE END Application */

