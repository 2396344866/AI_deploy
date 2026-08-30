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
#include "led.h" // LED indicator
#include "BSP_LOG.h"// LOG 串口
#include "BSP_W25Q64.h"// UART 整理
#include "logger.h"    // 统一日志模块（分级/时间戳/标签/编译开关）
#include "dbg_config.h" // 调试开关集中管理（DBG_LOG_<TASK> 等；对齐开发规范第76行；此前经 dbg_telemetry.h 间接带入）
#include "app_config.h" // L1 功能包含门控（APP_ENABLE_X / APP_PROFILE_*）：决定模块是否编进固件
#include "motor.h"     // 双电机 速度/位置闭环（阶段1）
#include "esp32s3.h"   // ESP32-S3 图像协处理器接收端（自建 BSP 模块）
#include "esp01s.h"     // ESP-01S WiFi / 阿里云 MQTT / OTA（自建 BSP 模块，USART2）
#include "arm_math.h"
#include "model_weights.h"
#include "ai_infer.h"
#include "test_dataset_processed.h"
#include "attitude.h"        // 姿态解算 + 外环控制器（Components/BSP/IMU）
#include "imu_mpu6050.h"     // MPU6050 底层 I2C 读取
  #include "dbg_telemetry.h"  // 调试遥测（UART1 firewater，见 Components/Debug）
  #include "selftest.h"      // 上电自检（POST）框架：Task_Test 调用 Selftest_RunAll（见 Components/POSTest/Src/selftest.c）
  #include "watchdog_heartbeat.h" // POST 收尾 watchdog_arm() + 运行期任务心跳喂狗（TIM7 经其判定）
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

/* 前向声明：g_cmd_qHandle 定义在下方的 CubeMX 生成区（line ~198）。
   DbgConsole_Process() 早于定义即调用 osMessageQueueGet，故先 extern 声明。
   仅声明不定义，避免与生成区重复（重生成保留）。 */
extern osMessageQueueId_t g_cmd_qHandle;

/* 命令通道 = g_cmd_qHandle（ItemSize=32, Size=4，CubeMX 生成）。
   ISR(OnFrame) 仅 osMessageQueuePut；常驻 StartLoggerTask 出队解析 —— 控制台不受 Motor 开关影响。 */

/* 调试控制台（常驻 Logger 任务）：g_cmd_qHandle 唯一消费者，按前缀分发。
 * 分类（单消费者，统一分发，模块逻辑留各自 .c）：
 *   - 全局(常活)：debug<n> 设级 / X ESP32-S3 统计
 *   - Sensor(守卫 APP_ENABLE_SENSOR)：T/P/K/C/F/D/M -> Attitude_ProcessCommand
 *   - Motor(守卫 APP_ENABLE_MOTOR)：A/B/S/R -> Motor_ProcessCommand
 * APP_ENABLE_X 未定义则不调 handler，控制台永活、不向空模块分发。 */
volatile uint32_t g_dbg_cmd_hits    = 0U;  /* DbgConsole_Process 成功出队次数 */
static void DbgConsole_Process(void)
{
    char lbuf[32];
    if (osMessageQueueGet(g_cmd_qHandle, lbuf, NULL, 0U) != osOK) {
        return;   /* 无命令：非阻塞返回，不拖慢 2ms 日志循环 */
    }
    g_dbg_cmd_hits++;   /* 成功出队一次 */
    uint16_t n = 0U;
    while (lbuf[n] != '\0' && n < 31U) n++;   /* 计算命令长度（无 string.h 依赖） */

    if (lbuf[0] == 'X') {
#if APP_ENABLE_ESP32S3
        ESP32S3_PrintStats();                  /* ESP32-S3 接收统计经 UART1 打印（统一调试通道） */
#endif
    } else if (lbuf[0] == 'd' && lbuf[1] == 'e' && lbuf[2] == 'b' &&
               lbuf[3] == 'u' && lbuf[4] == 'g') {
        /* debug<n>：运行期设级，免重烧现场调级。
         * n=0 FATAL/1 ERROR/2 WARN/3 INFO/4 DEBUG/5 TRACE。
         * 生效级 = min(LOG_COMPILE_MAX_LEVEL, n)（编译上限封顶）。
         * 回执绕过门控直发：命令确认任何级可见（FATAL 黑洞修复）。 */
        int lvl = lbuf[5] - '0';
        if (lvl >= 0 && lvl <= 5) {
            logger_set_level((uint8_t)lvl);
            char ack[] = "> log level -> X\r\n";
            ack[15] = (char)('0' + logger_get_level());   /* 显示真实生效级(已被编译上限封顶) */
            BSP_LOG_UART1_SendPoll((const uint8_t *)ack, (uint16_t)(sizeof(ack) - 1U));
        } else {
            static const char usage[] = "> debug usage: debug0..debug5 = FATAL..TRACE\r\n";
            BSP_LOG_UART1_SendPoll((const uint8_t *)usage, (uint16_t)(sizeof(usage) - 1U));
        }
    } else if (lbuf[0] == 'T' || lbuf[0] == 'P' || lbuf[0] == 'K' ||
               lbuf[0] == 'C' || lbuf[0] == 'F' || lbuf[0] == 'D' || lbuf[0] == 'M') {
#if APP_ENABLE_SENSOR
        Attitude_ProcessCommand(lbuf, n);      /* 姿态外环命令（属 Sensor 模块，仅 Sensor 使能时有效） */
#endif
    } else if (lbuf[0] == 'A' || lbuf[0] == 'B' || lbuf[0] == 'S' || lbuf[0] == 'R') {
#if APP_ENABLE_MOTOR
        Motor_ProcessCommand(lbuf, n);         /* 电机命令（A/B/S/R）：保留在 Motor 模块，仅 Motor 使能时有效 */
#else
        static const char moff[] = "> motor module disabled\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)moff, (uint16_t)(sizeof(moff) - 1U));
#endif
    }
    /* 其余未知命令：仅回显（见下方），不分发到任何模块 */
    BSP_LOG_UART1_SendPoll((const uint8_t *)lbuf, n);  /* 回显：从 ISR 移到任务，符合范式 */
}

TestResults_t g_Test_results = {0}; // 全局共享变量（类型 TestResults_t 见 ai_infer.h）

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
/* Definitions for Task_Esp32S3 */
osThreadId_t Task_Esp32S3Handle;
const osThreadAttr_t Task_Esp32S3_attributes = {
  .name = "Task_Esp32S3",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Test */
osThreadId_t Task_TestHandle;
const osThreadAttr_t Task_Test_attributes = {
  .name = "Task_Test",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for g_cmd_q */
osMessageQueueId_t g_cmd_qHandle;
const osMessageQueueAttr_t g_cmd_q_attributes = {
  .name = "g_cmd_q"
};
/* Definitions for g_ImgResultImg_q */
osMessageQueueId_t g_ImgResultImg_qHandle;
const osMessageQueueAttr_t g_ImgResultImg_q_attributes = {
  .name = "g_ImgResultImg_q"
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
int W25QXX_Test(void);
/* StartLoggerTask 的原型由 CubeMX 在下方（USER CODE 块外）统一生成，此处不再重复声明 */
/* USER CODE END FunctionPrototypes */

void StartInferenceTask(void *argument);
void StartMotorTask(void *argument);
void StartNetworkTask(void *argument);
void StartSensorTask(void *argument);
void StartScreenTask(void *argument);
void StartFlashTask(void *argument);
void StartLoggerTask(void *argument);
void StartEsp32S3Task(void *argument);
void StartTestTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  logger_init();   /* 初始化日志环形缓冲（须先于任务创建） */
  ESP32S3_BSP_Init();  /* ESP32-S3：仅强制 USART6 921600（队列/任务由 CubeMX 创建） */

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

  /* creation of g_ImgResultImg_q */
  g_ImgResultImg_qHandle = osMessageQueueNew (8, 72, &g_ImgResultImg_q_attributes);

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

  /* creation of Task_Esp32S3 */
  Task_Esp32S3Handle = osThreadNew(StartEsp32S3Task, NULL, &Task_Esp32S3_attributes);

  /* creation of Task_Test */
  Task_TestHandle = osThreadNew(StartTestTask, NULL, &Task_Test_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInferenceTask */
/**
  * @brief  Function implementing the Task_Inference thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInferenceTask */
void StartInferenceTask(void *argument)
{
  /* USER CODE BEGIN StartInferenceTask */
#if APP_ENABLE_INFERENCE
  /* 上电诊断推理已抽至 FaultDiag_ML_Test(selftest.c)，由 Task_Test 一次性跑；
     结果写 g_Test_results 供 NetworkTask 读。本循环仅待命。 */
  for (;;) {
    task_heartbeat_kick(HB_INFERENCE);   /* 运行期存活探针：TIM7 ISR 据此判 IWDG 是否喂 */
    osDelay(1000);
  }
#endif /* APP_ENABLE_INFERENCE */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartInferenceTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the Task_Motor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
#if APP_ENABLE_MOTOR

  /* 电机任务（阶段1：速度/位置闭环已在 TIM7 1ms 中断完成）。
   * 职责：1) PC13 按键(低电平)短按切 运行/刹车；2) 每秒打双电机遥测(联调)。
   * 命令解析已迁 StartLoggerTask，本任务不处理命令。 */
#if DBG_LOG_MOTOR
  LOG_I("MOTOR", "Motor task started. Press KEY(PC13) to toggle run/brake.");
#endif
  uint32_t log_cnt = 9;   /* 首行遥测提前到 ~100ms 后，便于一眼确认任务存活 */

  for(;;) {
    task_heartbeat_kick(HB_MOTOR);   /* 运行期存活探针：TIM7 ISR 据此判 IWDG 是否喂 */
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

    /* 3. 每秒打印一次遥测（对应任务开关关闭时不出，避免干扰 VOFA 波形） */
#if DBG_LOG_MOTOR
    if (++log_cnt >= 10) {   /* 10 * 100ms = 1s */
        log_cnt = 0;
        LOG_I("MOTOR", "A spd=%ld pwm=%ld rpm=%.1f | B spd=%ld pwm=%ld rpm=%.1f | run=%d mode=%d",
              Motor_GetSpeed(MOTOR_A), Motor_GetPWM(MOTOR_A), Motor_GetRPM(MOTOR_A),
              Motor_GetSpeed(MOTOR_B), Motor_GetPWM(MOTOR_B), Motor_GetRPM(MOTOR_B),
              g_motor_sys.running, g_motor_sys.mode);

        /* 无仪表探针：回读关键引脚电平，确认 PWM/STBY 真出现在物理脚 */
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
#endif /* APP_ENABLE_MOTOR */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartNetworkTask */
/**
  * @brief  Function implementing the Task_Network thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartNetworkTask */
void StartNetworkTask(void *argument)
{
  /* USER CODE BEGIN StartNetworkTask */
#if APP_ENABLE_NETWORK

  /* 上云初始化（一次性；失败每 10s 重试；阻塞超时不影响其它任务） */
  static uint8_t  s_net_up = 0;
  static uint32_t s_last_try = 0;
  /* Infinite loop */
  for(;;)
  {
    task_heartbeat_kick(HB_NETWORK);   /* 运行期存活探针：TIM7 ISR 据此判 IWDG 是否喂 */
    uint32_t s_now = osKernelGetTickCount();
    if (!s_net_up && (s_last_try == 0U || (s_now - s_last_try) > 10000U)) {
        s_last_try = s_now;
        if (ESP01S_Init() == ESP01S_OK &&
            ESP01S_ConnectTCP(ESP01S_MQTT_BROKER, ESP01S_MQTT_PORT) == ESP01S_OK &&
            ESP01S_MQTT_Connect() == ESP01S_OK) {
            s_net_up = 1;
            LOG_I("NET", "Aliyun MQTT online");
        } else {
            LOG_W("NET", "Network not ready, retry in 10s");
        }
    }
    if (g_Test_results.data_is_ready == 1) {
        if (osMutexAcquire(InferenceDataMutexHandle, osWaitForever) == osOK) {

            /* 安全读取数据并打印 */
#if DBG_LOG_NET
            LOG_I("NET", "Summary (From Network Task): acc=%.4f prec=%.4f recall=%.4f f1=%.4f total=%.2fms",
                  g_Test_results.overall_accuracy, g_Test_results.macro_precision,
                  g_Test_results.macro_recall, g_Test_results.macro_f1,
                  g_Test_results.total_time_ms);
#endif

            /* 组装 JSON 并发布到阿里云（MQTT QoS0） */
            float s_roll  = Attitude_GetRoll();
            float s_pitch = Attitude_GetPitch();
            float s_yaw   = Attitude_GetYaw();
            float s_magh  = Attitude_GetMagHeading();
            char s_json[256];
            int s_n = snprintf(s_json, sizeof(s_json),
                "{\"infer\":{\"acc\":%.4f,\"prec\":%.4f,\"rec\":%.4f,\"f1\":%.4f,\"ms\":%.2f},"
                "\"att\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"mag_h\":%.2f}}",
                g_Test_results.overall_accuracy, g_Test_results.macro_precision,
                g_Test_results.macro_recall, g_Test_results.macro_f1,
                g_Test_results.total_time_ms,
                s_roll, s_pitch, s_yaw, s_magh);
            if (s_net_up && s_n > 0 && s_n < (int)sizeof(s_json)) {
                if (ESP01S_MQTT_Pub(ESP01S_MQTT_PUB_TOPIC, s_json) == ESP01S_OK) {
#if DBG_LOG_NET
                    LOG_I("NET", "published %d bytes", s_n);
#endif
                } else {
                    LOG_W("NET", "publish failed");
                }
            }

            g_Test_results.data_is_ready = 0; // 打印后重置标志
            osMutexRelease(InferenceDataMutexHandle);
        }
    }
    osDelay(200); // 上云节奏；推理非每帧 ready
  }
#endif /* APP_ENABLE_NETWORK */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartNetworkTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the Task_Sensor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
#if APP_ENABLE_SENSOR

  if (MPU6050_EnableInt() != 0) { printf("WARN: MPU6050 data-ready INT enable FAILED!\r\n"); }   /* 启动 data-ready 中断（放调度器后，避免 200Hz ISR 在内核未起时冲击） */
  /* 姿态外环：等 data-ready 中断(ISR Give 信号量) -> 读 -> 滤波 -> 融合 -> 外环PID -> VOFA。
   * 频率由传感器 INT(ATTITUDE_RATE_HZ) 定；acquire 带超时看门狗，中断丢失不永久阻塞。
   * ISR 只 Give，业务全在此任务，符合 RTOS 铁律。 */
  int16_t ra[3], rg[3];
  ImuData_t imu;
  int32_t tgtA = 0, tgtB = 0;
  for (;;)
  {
    task_heartbeat_kick(HB_SENSOR);   /* 存活探针放最前：acquire 超时 continue 时仍踢，避免误判冻结 */
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
#endif /* APP_ENABLE_SENSOR */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartScreenTask */
/**
  * @brief  Function implementing the Task_Screen thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartScreenTask */
void StartScreenTask(void *argument)
{
  /* USER CODE BEGIN StartScreenTask */
#if APP_ENABLE_SCREEN

  /* Infinite loop */
  for(;;)
  {
    task_heartbeat_kick(HB_SCREEN);   /* 运行期存活探针：TIM7 ISR 据此判 IWDG 是否喂 */
    osDelay(1000);
  }
#endif /* APP_ENABLE_SCREEN */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartScreenTask */
}

/* USER CODE BEGIN Header_StartFlashTask */
/**
  * @brief  Function implementing the Task_Flash thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFlashTask */
void StartFlashTask(void *argument)
{
  /* USER CODE BEGIN StartFlashTask */
#if APP_ENABLE_FLASH

  /* 一次性上电自检：验证 40MHz SCK 下 W25Q64 读写稳定（改时钟树后必跑）。
   * 打印用 #ifdef LOG_ENABLED 包裹：生产模式(注释 LOG_ENABLED)变空，开发期正常；
   * 不走 per-task 开关（自检只跑一次）。 */
  int test_ret = W25QXX_Test();
#ifdef LOG_ENABLED
  if (test_ret != 0) {
    printf("[FLASH] W25Q64 self-test FAILED (ret=%d), see W25Q TEST log\r\n", test_ret);
  } else {
    printf("[FLASH] W25Q64 self-test PASSED\r\n");
  }
#endif

  /* Infinite loop */
  for(;;)
  {
    task_heartbeat_kick(HB_FLASH);   /* 运行期存活探针：TIM7 ISR 据此判 IWDG 是否喂 */
    osDelay(1);
  }
#endif /* APP_ENABLE_FLASH */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartFlashTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the Task_logger thread.
* @param argument: Not used
* @retval None
*/
/* ===========================================================================
 * 日志后台任务 = CubeMX 原生 Task_logger（Entry = StartLoggerTask）。
 *   - 函数体/声明/osThreadNew 均 CubeMX 生成，重生成自动保留，无需手写。
 * 职责：低优先级循环 -> logger_drain 异步刷环形缓冲到串口；
 *       常驻 DbgConsole_Process（debug<n>/姿态/电机命令，不随 APP_ENABLE_X 失效）。
 * 生产任务仅 格式化+入缓冲，不阻塞，实时性不受 printf 影响。
 * logger_init() 已在 MX_FREERTOS_Init 的 Init 块调用。
 * =========================================================================== */
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  (void)argument;
  for (;;) {
      logger_drain();        /* 阻塞刷环形缓冲（跑在低优先级任务里，不抢占控制环） */
      DbgConsole_Process();  /* 调试命令控制台：常驻，不随任何业务模块开关（APP_ENABLE_X）失效 */
      /* 喂狗点已迁 TIM7_IRQHandler(ISR, NVIC prio3)：运行期每 500ms 经
         watchdog_should_feed() 喂；POST 期由各 Xxx_Test 协作喂(selftest.c log_wdt_feed())。
         本任务不喂狗——杜绝"最低优先级喂狗被忙等饿死 → IWDG 复位环"反模式。 */
      osDelay(2);            /* 释放 CPU，让高优先级任务（Motor/Inference）先跑 */
  }
  /* USER CODE END StartLoggerTask */
}

/* USER CODE BEGIN Header_StartEsp32S3Task */
/**
  * @brief  Function implementing the Task_Esp32S3 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEsp32S3Task */
void StartEsp32S3Task(void *argument)
{
  /* USER CODE BEGIN StartEsp32S3Task */
#if APP_ENABLE_ESP32S3

  /* CubeMX owns the task; implementation lives in BSP (esp32s3.c).
     ESP32S3_Task_Run starts USART6 RX and drains g_ImgResultImg_qHandle. */
  ESP32S3_Task_Run(argument);
#endif /* APP_ENABLE_ESP32S3 */
  osThreadTerminate(osThreadGetId());   /* 模块未使能：任务自删(CMSIS-RTOS2 API)，永不返回 */
  /* USER CODE END StartEsp32S3Task */
}

/* USER CODE BEGIN Header_StartTestTask */
/**
* @brief Function implementing the Task_Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestTask */
void StartTestTask(void *argument)
{
  /* USER CODE BEGIN StartTestTask */
  (void)argument;

  /* 开发期定位：打 __FILE__:__LINE__ 确认 Task_Test 已调度进此函数体。 */
  LOG_I("TEST", "StartTestTask ENTER @ %s:%d", __FILE__, __LINE__);

  /* 上电一站式自检(POST)：Flash→Inference→Sensor→Esp32S3→Motor→Network→Screen。
     关键失败→存档+复位；非关键→打印+继续。长段(ML推理)内部自喂狗。 */
  Selftest_RunAll();

  /* 栈高水位测量：供定 Task_Test 栈大小（osThreadGetStackSpace 返未用栈最小值）。 */
  uint32_t free_b  = osThreadGetStackSpace(osThreadGetId());
  uint32_t total_b = Task_Test_attributes.stack_size;   /* = 2048*4 = 8192 字节（CubeMX 配置） */
  uint32_t used_b  = (total_b > free_b) ? (total_b - free_b) : 0U;
  LOG_I("TEST", "Task_Test stack: used=%lu/%lu bytes (free=%lu)  -> 建议 final stack ≈ %lu",
        used_b, total_b, free_b, used_b + 256U);

  osThreadTerminate(osThreadGetId());   /* 跑完自删当前任务：CMSIS 自删必须传有效句柄，
                                           NULL 被当参数错丢弃(见 cmsis_os2.c:845 hTask==NULL→osErrorParameter)，
                                           故不能用 osThreadTerminate(NULL)；等价裸写法 vTaskDelete(NULL) */
  /* USER CODE END StartTestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 帧接收回调：覆盖 BSP_LOG.c 的 __weak 默认。
   运行于 ISR(USART1_IDLE)：整帧拷本地缓冲并推 g_cmd_qHandle；
   解析/回显留 StartLoggerTask，符合"ISR 只 Send/Give，不碰业务"。
   USART1_IRQn prio=8 ≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，
   故 ISR 内 osMessageQueuePut 合法；队满(4 槽)丢弃，不阻塞中断。 */

volatile uint32_t g_dbg_onframe_hits= 0U;  /* OnFrame 被调用次数 */
void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len)
{
    if (len == 0U || len > 31U) return;          /* 越界保护，队列 ItemSize=32(含'\0') */
    char item[32];
    for (uint16_t i = 0U; i < len; i++) item[i] = (char)data[i];
    item[len] = '\0';
		g_dbg_onframe_hits++;
    osMessageQueuePut(g_cmd_qHandle, item, 0U, 0U);   /* ISR 安全，非阻塞 */
}

/* USER CODE END Application */

