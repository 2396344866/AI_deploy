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
  #include "postest.h"       // 上电自检（POST）编排框架：Task_Test 调用 Postest_RunAll（见 Components/POSTest/Src/Postest.c）
  #include "watchdog_heartbeat.h" // POST 收尾 watchdog_arm() + 运行期任务心跳喂狗（TIM7 经其判定）
  /* 物模型属性 API 已并入 esp01s.h（不再单列 aliot_property.*） */
  #include <string.h>           // strstr：下行 topic 匹配（不依赖 main.h 间接引入）
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
extern void uart_err_inject_test(UART_HandleTypeDef *, uint32_t);  /* 测试接口：软件注入 UART RX 错误（见 uart_rx_dispatcher.c） */

/* 网络状态机全局（StartNetworkTask 写，控制台 `n` 命令读）；APP_ENABLE_NETWORK 未定义则整段不编。
 * 详见 Doc/网络离线态与分区设计.md。 */
#if APP_ENABLE_NETWORK
typedef enum { NET_INIT, NET_CONNECTING, NET_ONLINE, NET_RETRYING, NET_OFFLINE } net_state_t;
static net_state_t g_net_state    = NET_INIT;
static uint32_t    g_net_fail_cnt = 0U;
static uint32_t    g_net_next_try  = 0U;
static uint32_t    g_net_backoff   = 10000U;   /* 初始退避 10s */
#define NET_FAIL_THRESHOLD    6U
#define NET_OFFLINE_RETRY_MS  60000U          /* OFFLINE 每 60s 重连 */
#define NET_BACKOFF_CAP_MS    60000U
#endif

/* 命令通道 = g_cmd_qHandle（ItemSize=64, Size=4，CubeMX 生成；旧值 32 见 .ioc Queues01）。
   ⚠ ItemSize 由 CubeMX 生成，改它必须动 .ioc（FREERTOS.Queues01），勿手改生成区。
   ⚠ 队列按 ItemSize=64 整块拷贝：OnFrame 的 item[] 与 DbgConsole_Process 的 lbuf[]
     必须【同时】≥64，否则 osMessageQueuePut/Get 越界。三处须一起改。
   ISR(OnFrame) 仅 osMessageQueuePut；常驻 StartLoggerTask 出队解析 —— 控制台不受 Motor 开关影响。 */

/* 调试控制台（常驻 Logger 任务）：g_cmd_qHandle 唯一消费者，按前缀分发。
 * 双轨命名（2026-09-02 重构）：旧单字母键【全部保留】，新增「域.动作」长名并存。
 *   - att.* / mag.* / filt.*  -> 在 Attitude_ProcessCommand 内归一化（与姿态逻辑同模块）
 *   - mot.* / sys.*           -> 在本文件 DbgConsole_Normalize() 归一化
 *   - S 急停【永远保留单键】：安全命令不能要求打字。
 * 三条公约：①无参=查询 ②未知命令报错（不再静默丢弃）③改值即回读。
 * 分类（单消费者，统一分发，模块逻辑留各自 .c）：
 *   - 全局(常活)：debug<n>|(sys.lvl) 设级 / X|(sys.s3) ESP32-S3 统计 / ?|help 域索引
 *   - Sensor(守卫 APP_ENABLE_SENSOR)：T/P/K/C/F/D/M/H -> Attitude_ProcessCommand
 *     ⚠ 新增命令必须【同时】改本注释与下方分发表，否则 handler 写了也永远调不到
 *       （H 航向保持曾漏配路由，命令被静默丢弃，见 Error/sensor_error.md E41）
 *     另有【小写长名】att./mag./filt. 需单独一条前缀路由（大写单键分支匹配不到小写），
 *       漏了会导致所有 att.* mag.* filt.* 报 unknown —— 与 E41 同坑，见 E43。
 *   - Motor(守卫 APP_ENABLE_MOTOR)：A/B/S/R|(mot.a/b/run/stop) -> Motor_ProcessCommand
 *   - 未命中：handled=0 -> 统一回 "> ?unknown cmd"（不再静默回显，E41/E43 教训）
 * APP_ENABLE_X 未定义则不调 handler，控制台永活、不向空模块分发。 */
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
volatile uint32_t g_dbg_cmd_hits    = 0U;  /* DbgConsole_Process 成功出队次数 */
volatile uint32_t g_dbg_cmd_toolong = 0U;  /* OnFrame 丢弃的超长帧计数（ISR 写，任务清+提示） */
#endif
/* 双轨命名归一化：把「域.动作」长名改写为既有单键，复用下方分发表。
   规则：命中前缀后写入 repl，再跳过输入里的空格，最后拷贝剩余参数。
     例：sys.lvl 3 -> debug3   mot.a 100 -> A100   mot.stop -> S   sys.net -> n
   各 repl 均不长于其前缀，输出恒 ≤ 输入长度，故可在 lbuf 内原地改写（缓冲 64 B 足够）。
   边界：前缀后必须是 '\0'/空格/Tab，否则不认（避免 mot.astop 之类被误匹配成 mot.a）。
   att./mag./filt. 不在此处理——由 Attitude_ProcessCommand 自行归一化（与姿态逻辑同模块）。 */
static void DbgConsole_Normalize(char *s)
{
    static const struct { const char *pfx; const char *repl; } alias[] = {
        { "mot.stop", "S"     },
        { "mot.run",  "R"     },
        { "mot.a",    "A"     },
        { "mot.b",    "B"     },
        { "sys.lvl",  "debug" },
        { "sys.net",  "n"     },
        { "sys.s3",   "X"     },
    };
    for (size_t i = 0U; i < sizeof(alias) / sizeof(alias[0]); i++) {
        size_t pl = strlen(alias[i].pfx);
        if (strncmp(s, alias[i].pfx, pl) != 0) continue;
        char c = s[pl];
        /* token 边界：'\0'/空格/Tab，或【紧贴参数】的数字（sys.lvl4 -> debug4、mot.a100 -> A100）。
           容忍紧贴是刻意的：旧键 debug4/A100 本就无分隔符，迁移到长名时肌肉记忆会写成 sys.lvl4。
           本表所有 pfx 均以 '.' 结尾，其后紧跟数字只可能是参数，不会误吞命令名。 */
        if (c != '\0' && c != ' ' && c != '\t' && !(c >= '0' && c <= '9')) continue;
        const char *rest = s + pl;
        while (*rest == ' ' || *rest == '\t') rest++;
        char out[64];
        int k = snprintf(out, sizeof(out), "%s%s", alias[i].repl, rest);
        if (k > 0 && (size_t)k < sizeof(out)) {
            for (size_t j = 0U; j <= (size_t)k; j++) s[j] = out[j];   /* 含 '\0' */
        }
        return;
    }
}

static void DbgConsole_Process(void)
{
    char lbuf[64];   /* ⚠ 必须 ≥ 队列 ItemSize(64)：osMessageQueueGet 按 ItemSize 整块拷贝 */
    if (osMessageQueueGet(g_cmd_qHandle, lbuf, NULL, 0U) != osOK) {
        return;   /* 无命令：非阻塞返回，不拖慢 2ms 日志循环 */
    }
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
    g_dbg_cmd_hits++;   /* 成功出队一次 */
#endif
    /* 留一份原文：归一化会改写 lbuf，回显须保持用户输入的原样。
       ⚠ 强制封尾：若尚未 Generate Code（队列 ItemSize 仍为 32），lbuf[32..63] 不会被写入，
       直接扫描会读到栈垃圾。补 '\0' 保证串边界恒定，两种 ItemSize 下都安全。 */
    lbuf[63] = '\0';
    char orig[64];
    uint16_t n0 = 0U;
    while (lbuf[n0] != '\0' && n0 < 63U) n0++;
    for (uint16_t i = 0U; i <= n0; i++) orig[i] = lbuf[i];   /* 含结尾 '\0' */

    /* 超长帧提示：ISR 只能丢弃（ISR 内不打印），计数后在任务上下文补一条回执 */
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
    if (g_dbg_cmd_toolong != 0U) {
        g_dbg_cmd_toolong = 0U;
        static const char toolong[] = "> cmd too long (max 63 chars) - dropped\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)toolong, (uint16_t)(sizeof(toolong) - 1U));
    }
#endif

    DbgConsole_Normalize(lbuf);          /* 长名 -> 旧单键（原地改写） */
    uint16_t n = 0U;
    while (lbuf[n] != '\0' && n < 63U) n++;

    /* 命中标志：乐观置 1，分发表未命中时由链尾 else 置 0（不再静默丢弃） */
    int handled = 1;

    if (lbuf[0] == '?' || strncmp(lbuf, "help", 4) == 0) {
        static const char hlp[] =
            "> domains: att(ref/pid/on/off/cal/dump/yaw) | mag(init/align)\r\n"
            ">          filt(lag/trim/elag/mlag/mtrim/yawmag) | mot(a/b/run/stop)\r\n"
            ">          sys(lvl/net/s3/ch) | chan[<组>] | debug0..5 | ?=this\r\n"
            "> rules: no-arg = query; old keys all kept; S = estop\r\n"
            "> chan[<组>]/sys.ch[<组>]: VOFA 通道清单(与 vofa_panel.json 同步) 例 chan / chan att\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)hlp, (uint16_t)(sizeof(hlp) - 1U));
    } else if ((strncmp(lbuf, "chan", 4) == 0 && (lbuf[4] == '\0' || lbuf[4] == ' ' || lbuf[4] == '\t')) ||
               (strncmp(lbuf, "sys.ch", 6) == 0 && (lbuf[6] == '\0' || lbuf[6] == ' ' || lbuf[6] == '\t'))) {
        /* chan [<组>] / sys.ch [<组>]：打印 VOFA 通道清单（三处一致见 vofa_panel.json / motor_vofa_telemetry.md）。
           ⚠ 直接判前缀而非走 Normalize：sys.ch 带参时 Normalize 会丢空格。 */
#if defined(DBG_TELEMETRY_ENABLE) && (DBG_TELEMETRY_ENABLE == 1)
        const char *cp = (lbuf[0] == 'c') ? (lbuf + 4) : (lbuf + 6);
        while (*cp == ' ' || *cp == '\t') cp++;
        Dbg_Telemetry_PrintChannels(*cp == '\0' ? (const char *)NULL : cp);
#else
        static const char cmsg[] = "> chan: requires TRACE level + telemetry (DBG_TELEMETRY_ENABLE=1)\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)cmsg, (uint16_t)(sizeof(cmsg) - 1U));
#endif /* DBG_TELEMETRY_ENABLE && telemetry on */
    } else if (lbuf[0] == 'X') {
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
#if defined(DBG_UART_ERR_INJECT_TEST) && DBG_UART_ERR_INJECT_TEST
    } else if (lbuf[0] == 'e' && (lbuf[1] == '4' || lbuf[1] == '6')) {
        /* e4=注入 UART4(屏) RX 错误(非致命路径)；e6=注入 USART6(ESP32-S3) RX 错误(致命路径)。
           免 USB-TTL 验证 B+C 恢复策略；e6 累计到阈值升级黑匣子+复位。
           编译期由 DBG_UART_ERR_INJECT_TEST 门控（需 APP_ENABLE_SCREEN / APP_ENABLE_ESP32S3 其一）。 */
#if APP_ENABLE_SCREEN
        if (lbuf[1] == '4') {
            uart_err_inject_test(&huart4, HAL_UART_ERROR_FE);
        }
#endif
#if APP_ENABLE_ESP32S3
        if (lbuf[1] == '6') {
            uart_err_inject_test(&huart6, HAL_UART_ERROR_FE);
        }
#endif
        static const char ack[] = "> injected UART rx err (see LOG_W/UART)\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)ack, (uint16_t)(sizeof(ack) - 1U));
#endif
    } else if (lbuf[0] == 'T' || lbuf[0] == 'P' || lbuf[0] == 'K' ||
               lbuf[0] == 'C' || lbuf[0] == 'F' || lbuf[0] == 'D' ||
               lbuf[0] == 'M' || lbuf[0] == 'H') {   /* H=航向保持(E39)：attitude.c:745 已实现，
                                                        此前漏配路由导致命令静默丢弃，见 Error/sensor_error.md E41 */
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
    } else if (strncmp(lbuf, "mot", 3) == 0 && (lbuf[3] == '\0' || lbuf[3] == ' ')) {
        /* mot（无参）= 查询（公约①）：电机域概览。
           注：mot.a/mot.b/mot.run/mot.stop 已在 DbgConsole_Normalize() 归一化为 A/B/R/S，
           归一化后不再以 "mot" 开头，故不会落到这里。区分大小写：大写 M 属姿态域(mag)。 */
#if APP_ENABLE_MOTOR
        char ms[96];
        int mk = snprintf(ms, sizeof(ms),
                          "> mot: run=%u mode=%u | A tgt=%ld spd=%ld pwm=%ld | B tgt=%ld spd=%ld pwm=%ld\r\n",
                          (unsigned)g_motor_sys.running, (unsigned)g_motor_sys.mode,
                          (long)Motor_GetTargetSpeed(MOTOR_A), (long)Motor_GetSpeed(MOTOR_A), (long)Motor_GetPWM(MOTOR_A),
                          (long)Motor_GetTargetSpeed(MOTOR_B), (long)Motor_GetSpeed(MOTOR_B), (long)Motor_GetPWM(MOTOR_B));
        if (mk > 0 && (size_t)mk < sizeof(ms)) BSP_LOG_UART1_SendPoll((const uint8_t *)ms, (uint16_t)mk);
#else
        static const char moff2[] = "> motor module disabled\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)moff2, (uint16_t)(sizeof(moff2) - 1U));
#endif
    } else if (strncmp(lbuf, "sys", 3) == 0 && (lbuf[3] == '\0' || lbuf[3] == ' ')) {
        /* sys（无参）= 查询（公约①）：系统域概览（日志级是最常被问的全局状态） */
        char ss[80];
        int sk = snprintf(ss, sizeof(ss),
                          "> sys: log_level=%u (sys.lvl 0..5) | domains: att/mag/filt/mot/sys | ?=help\r\n",
                          (unsigned)logger_get_level());
        if (sk > 0 && (size_t)sk < sizeof(ss)) BSP_LOG_UART1_SendPoll((const uint8_t *)ss, (uint16_t)sk);
    } else if ((strncmp(lbuf, "att", 3) == 0 && (lbuf[3] == '\0' || lbuf[3] == ' ' || lbuf[3] == '\t' || lbuf[3] == '.')) ||
               (strncmp(lbuf, "mag", 3) == 0 && (lbuf[3] == '\0' || lbuf[3] == ' ' || lbuf[3] == '\t' || lbuf[3] == '.')) ||
               (strncmp(lbuf, "filt", 4) == 0 && (lbuf[4] == '\0' || lbuf[4] == ' ' || lbuf[4] == '\t' || lbuf[4] == '.'))) {
        /* att.* / mag.* / filt.*（小写长名）→ 送姿态模块，归一化在 Attitude_ProcessCommand 内部做。
           ⚠ 必须在路由层单独判前缀：长名是小写，而上面的大写单键分支（T/P/K/C/F/D/M/H）匹配不到，
              若漏这条路由，att.ref/mag/filt 会全部掉进链尾报 unknown（归一化发生在 handler 内，路由看不到）。
           ⚠ 用精确前缀而非 lbuf[0]=='a'/'m'/'f'：否则 "mot"/"sys" 会被 'm' 抢走。
           扩展：新增 att.* 子命令只需改 attitude.c 的 alias 表，本路由不必动。 */
#if APP_ENABLE_SENSOR
        Attitude_ProcessCommand(lbuf, n);
#else
        static const char soff[] = "> sensor module disabled\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)soff, (uint16_t)(sizeof(soff) - 1U));
#endif
    }
#if APP_ENABLE_NETWORK
    else if (lbuf[0] == 'n') {
        static const char *nst[5] = { "INIT", "CONNECTING", "ONLINE", "RETRYING", "OFFLINE" };
        uint32_t now = osKernelGetTickCount();
        uint32_t rem = (g_net_next_try > now) ? (g_net_next_try - now) : 0U;
        char ns[72];
        int nn = snprintf(ns, sizeof(ns), "> net: %s fail=%u next=%u ms\r\n",
                         nst[(int)g_net_state], (unsigned)g_net_fail_cnt, (unsigned)rem);
        if (nn > 0 && (size_t)nn < sizeof(ns)) BSP_LOG_UART1_SendPoll((const uint8_t *)ns, (uint16_t)nn);
    }
#endif
    else { handled = 0; }   /* 未命中任何域：链尾统一报错，不再静默回显（E41 静默丢弃教训） */

    if (!handled) {
        static const char unk[] = "> ?unknown cmd (try: help | att / mag / filt / mot / sys)\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)unk, (uint16_t)(sizeof(unk) - 1U));
    }
    /* 回显【原文】（归一化前的用户输入），从 ISR 移到任务，符合范式。
       串口助手未勾「发送新行」时 orig 不带 \n，回显会与下一条回执粘连，故兜底补 \r\n。 */
    BSP_LOG_UART1_SendPoll((const uint8_t *)orig, n0);
    if (n0 == 0U || orig[n0 - 1U] != '\n') {
        static const char crlf[] = "\r\n";
        BSP_LOG_UART1_SendPoll((const uint8_t *)crlf, 2U);
    }
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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
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
/* Definitions for i2c1_mutex */
osMutexId_t i2c1_mutexHandle;
const osMutexAttr_t i2c1_mutex_attributes = {
  .name = "i2c1_mutex"
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

  /* creation of i2c1_mutex */
  i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes);

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
  g_cmd_qHandle = osMessageQueueNew (4, 64, &g_cmd_q_attributes);

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
  /* 上电诊断推理已抽至 FaultDiag_ML_Test(ai_infer.c)，由 Task_Test 一次性跑；
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
            if (key_stable == 0) {              /* 下降沿：切换站起/坐下意图 */
                /* KEY 发站起/坐下意图，由 FSM 仲裁（详见 balance_autonomy_plan.md） */
                static uint8_t key_want_up = 0;
                key_want_up ^= 1U;
                Attitude_SetKeyStand(key_want_up);
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

  /* 网络状态机：NET_INIT→CONNECTING→NET_ONLINE⇄NET_RETRYING→NET_OFFLINE
   * - 任何态每轮 task_heartbeat_kick(HB_NETWORK) → OFFLINE 不饿死 IWDG。
   * - 离线时本地任务(推理/黑匣子/IMU/屏/电机)照常，仅 MQTT 发布挂起。
   * - 离线期最新推理帧暂存 s_infer_buf，重连后补传（最小缓冲；完整 Flash 环形留二次开发）。
   * - 上云分两路：物模型属性(thing/event/property/post) 与 推理指标(自定义 user/infer)。
   * 详见 Doc/网络离线态与分区设计.md。 */
  /* 缓冲全部 static：Task_Network 栈仅 512*4 = 2048 B，这些帧缓冲不能压栈 */
  static char s_infer_buf[256];                  /* 推理指标 + 姿态 -> 自定义 user/infer topic */
  static int  s_infer_len = 0;
  static char s_prop_buf[ESP01S_JSON_MAX];       /* 物模型 10 属性 -> thing/event/property/post */
  static char s_dl_topic[ESP01S_TOPIC_MAX];      /* 下行 topic（订阅 property/set） */
  static char s_dl_payload[ESP01S_PAYLOAD_MAX];  /* 下行 payload（property/set JSON） */
  static char s_reply[ESP01S_REPLY_MAX];         /* set_reply JSON */
  static uint32_t s_next_prop_post = 0U;         /* 下次属性周期上报时刻 */
  static uint32_t s_next_ping      = 0U;         /* 下次 MQTT PINGREQ 时刻 */
  static uint8_t  s_was_online     = 0U;         /* P3 D3：上轮网络在线态（边沿检测，避免每轮刷日志） */

  ESP01S_AliIot_Init();   /* 属性路由初始化：LED 灭、电机停机（上电安全态） */

  for (;;)
  {
    task_heartbeat_kick(HB_NETWORK);   /* 运行期存活探针：TIM7 ISR 据此判 IWDG 是否喂（OFFLINE 也踢） */
    uint32_t s_now = osKernelGetTickCount();

    /* 1) 拉取最新一帧到本地缓冲（无论在线与否），避免离线丢数据 */
    if (g_Test_results.data_is_ready == 1) {
      if (osMutexAcquire(InferenceDataMutexHandle, 100U) == osOK) {
        float s_roll  = Attitude_GetRoll();
        float s_pitch = Attitude_GetPitch();
        float s_yaw   = Attitude_GetYaw();
        float s_magh  = Attitude_GetMagHeading();
        int s_n = snprintf(s_infer_buf, sizeof(s_infer_buf),
            "{\"infer\":{\"acc\":%.4f,\"prec\":%.4f,\"rec\":%.4f,\"f1\":%.4f,\"ms\":%.2f},"
            "\"att\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"mag_h\":%.2f}}",
            g_Test_results.overall_accuracy, g_Test_results.macro_precision,
            g_Test_results.macro_recall, g_Test_results.macro_f1, g_Test_results.total_time_ms,
            s_roll, s_pitch, s_yaw, s_magh);
        if (s_n > 0 && s_n < (int)sizeof(s_infer_buf)) {
          s_infer_len = s_n;
          g_Test_results.data_is_ready = 0;   /* 取走即复位（缓冲已持有副本） */
        }
        osMutexRelease(InferenceDataMutexHandle);
      }
    }

    /* 2) 状态机：ONLINE 收下行 + 两类上报；其余态到点才尝试连接 */
    if (g_net_state == NET_ONLINE) {
      /* 2a) MQTT keepalive：CONNECT 声明 60s，这里 50s 一发，防 broker 静默踢链 */
      if ((int32_t)(s_now - s_next_ping) >= 0) {
        s_next_ping = s_now + 50000U;
        (void)ESP01S_MQTT_Ping();
      }

      /* 2b) 下行：云端 property/set -> 本地动作 -> set_reply */
      if (ESP01S_MQTT_PollPublish(s_dl_topic,   (uint16_t)sizeof(s_dl_topic),
                                  s_dl_payload, (uint16_t)sizeof(s_dl_payload),
                                  NULL, NULL) == ESP01S_OK) {
        if (strstr(s_dl_topic, "property/set") != NULL) {
          int hits = ESP01S_AliIot_HandleSet(s_dl_payload, s_reply, (uint16_t)sizeof(s_reply));
          (void)ESP01S_MQTT_Pub(ESP01S_MQTT_SET_REPLY_TOPIC, s_reply);
#if DBG_LOG_NET
          LOG_I("NET", "property/set handled: hits=%d", hits);
#else
          (void)hits;   /* 宏关闭时抑制 -Wunused-variable */
#endif
        }
        /* 其余下行帧（阿里云对上报的 property/post 应答、property/post_reply 等）为预期内：
           设备上线即自动订阅 thing/event/#，无需处理；静默丢弃、不打印，避免刷屏造成误解。 */
      }

      /* 2c) 推理帧（自定义 user/infer topic，不进物模型，避免未定义标识符被拒） */
      if (s_infer_len > 0) {
        if (ESP01S_MQTT_Pub(ESP01S_MQTT_PUB_TOPIC_INFER, s_infer_buf) == ESP01S_OK) {
#if DBG_LOG_NET
          LOG_I("NET", "published infer %d bytes", s_infer_len);
#endif
          s_infer_len = 0;
        } else {
          LOG_W("NET", "publish failed, link may be down -> RETRYING");
          g_net_state = NET_RETRYING;
          g_net_next_try = s_now + g_net_backoff;
          uint32_t nb = g_net_backoff * 2U;
          if (nb > NET_BACKOFF_CAP_MS) nb = NET_BACKOFF_CAP_MS;
          g_net_backoff = nb;
        }
      }

      /* 2d) 物模型属性周期上报（欧拉角/温度/开关态，5s 一帧） */
      if ((int32_t)(s_now - s_next_prop_post) >= 0) {
        s_next_prop_post = s_now + 5000U;
        int plen = ESP01S_AliIot_BuildReport(s_prop_buf, (uint16_t)sizeof(s_prop_buf));
        if (plen < 0) {
          LOG_W("NET", "property report build failed (buffer too small)");
        } else if (ESP01S_MQTT_Pub(ESP01S_MQTT_PUB_TOPIC, s_prop_buf) != ESP01S_OK) {
          LOG_W("NET", "property post failed -> RETRYING");
          g_net_state = NET_RETRYING;
          g_net_next_try = s_now + g_net_backoff;
          uint32_t nb = g_net_backoff * 2U;
          if (nb > NET_BACKOFF_CAP_MS) nb = NET_BACKOFF_CAP_MS;
          g_net_backoff = nb;
        }
      }
    } else {
      if ((int32_t)(s_now - g_net_next_try) >= 0) {
        int net_rc_init  = ESP01S_Init();
        int net_rc_tcp   = (net_rc_init == ESP01S_OK) ? ESP01S_ConnectTCP(ESP01S_MQTT_BROKER, ESP01S_MQTT_PORT) : ESP01S_ERR_LINK;
        int net_rc_mqtt = (net_rc_tcp  == ESP01S_OK) ? ESP01S_MQTT_Connect() : ESP01S_ERR_LINK;
        if (net_rc_init == ESP01S_OK && net_rc_tcp == ESP01S_OK && net_rc_mqtt == ESP01S_OK) {
          g_net_state = NET_ONLINE;
          g_net_fail_cnt = 0U;
          g_net_backoff = 10000U;
          LOG_I("NET", "Aliyun MQTT online");

          /* 订阅云端属性设置（下发控制入口）。失败不致命：仍能上报，只是收不到下行。 */
          if (ESP01S_MQTT_Sub(ESP01S_MQTT_SUB_TOPIC) != ESP01S_OK) {
            LOG_W("NET", "subscribe property/set FAILED (downlink disabled)");
          }
          s_next_prop_post = s_now;             /* 上线即报一帧属性 */
          s_next_ping      = s_now + 50000U;

          if (s_infer_len > 0 &&
              ESP01S_MQTT_Pub(ESP01S_MQTT_PUB_TOPIC_INFER, s_infer_buf) == ESP01S_OK) {
#if DBG_LOG_NET
            LOG_I("NET", "flushed offline-buffered infer frame");
#endif
            s_infer_len = 0;
          }
        } else {
          g_net_fail_cnt++;
#if DBG_LOG_NET
          LOG_T("NET", "connect failed: init=%d tcp=%d mqtt=%d", net_rc_init, net_rc_tcp, net_rc_mqtt);
#endif
          if (g_net_fail_cnt >= NET_FAIL_THRESHOLD) {
            g_net_state = NET_OFFLINE;
            g_net_next_try = s_now + NET_OFFLINE_RETRY_MS;
            LOG_I("NET", "entering OFFLINE (reconnect every %u s)", NET_OFFLINE_RETRY_MS / 1000U);
          } else {
            g_net_state = NET_RETRYING;
            g_net_next_try = s_now + g_net_backoff;
            uint32_t nb = g_net_backoff * 2U;
            if (nb > NET_BACKOFF_CAP_MS) nb = NET_BACKOFF_CAP_MS;
            LOG_W("NET", "connect failed (%u/%u), retry in %u ms",
                  g_net_fail_cnt, NET_FAIL_THRESHOLD, g_net_backoff);
            g_net_backoff = nb;
          }
        }
      }
    }

    /* P3 FSM D3 同步：将 g_net_state 的 ONLINE 状态镜像到自治 FSM（边沿触发、幂等、不每轮刷）。
       CONNACK OK→1（云端意图生效）；掉线/超时→0（云端意图锁存、仅 KEY 可驱动）。 */
    {
        uint8_t now_online = (g_net_state == NET_ONLINE) ? 1U : 0U;
        if (now_online != s_was_online) {
            Attitude_SetNetOnline(now_online);
            if (now_online) LOG_I("NET", "network ONLINE -> autonomy D3: cloud intent active");
            else            LOG_W("NET", "network DOWN -> autonomy D3: cloud intent latched, KEY only");
            s_was_online = now_online;
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
#if APP_ENABLE_NETWORK
  int16_t mpu_t = 0;                    /* MPU6050 温度原始值（供物模型 temp 属性上报） */
#endif
  for (;;)
  {
    task_heartbeat_kick(HB_SENSOR);   /* 存活探针放最前：acquire 超时 continue 时仍踢，避免误判冻结 */
    if (osSemaphoreAcquire(g_semAttitudeDataReadyHandle, 100U) != osOK) {
        continue;                       /* 超时：跳过本拍，不阻塞 */
    }
    /* 温度沿用本次已有的 I2C 读取（第 3 参数），不新增总线访问、不引入 I2C 竞争 */
#if APP_ENABLE_NETWORK
    if (MPU6050_ReadRaw(ra, rg, &mpu_t, 0) != 0) continue;   /* 热路径：抢不到 I2C1 锁即丢一帧（C/M 标定持锁期间） */
    ESP01S_AliIot_UpdateTempRaw(mpu_t);
#else
    if (MPU6050_ReadRaw(ra, rg, NULL, 0) != 0) continue;     /* 热路径：抢不到 I2C1 锁即丢一帧 */
#endif
    ImuFilter_Update(ra, rg, &imu);
    Attitude_Update(&imu);
    Attitude_RunController();
    Attitude_AutonomyTick();          /* v4b 本地自治 FSM 决策（FSM 永久接管） */
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
  extern void uart_err_monitor(void);   /* 见 Components/BSP/uart_rx_dispatcher.c：ISR 存快照，此处裁决 */
  for (;;) {
      logger_drain();        /* 阻塞刷环形缓冲（跑在低优先级任务里，不抢占控制环） */
      DbgConsole_Process();  /* 调试命令控制台：常驻，不随任何业务模块开关（APP_ENABLE_X）失效 */
      uart_err_monitor();    /* UART RX 错误快照消费：非致命重收+计数；关键链路持续故障升级为黑匣子+复位 */
      /* 喂狗点已迁 TIM7_IRQHandler(ISR, NVIC prio3)：运行期每 500ms 经
         watchdog_should_feed() 喂；POST 期由各 Xxx_Test 协作喂(Postest.c/各组件 log_wdt_feed())。
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
  Postest_RunAll();

  /* 栈高水位测量：供定 Task_Test 栈大小（osThreadGetStackSpace 返未用栈最小值）。 */
  uint32_t free_b  = osThreadGetStackSpace(osThreadGetId());
  uint32_t total_b = Task_Test_attributes.stack_size;   /* Cubemx配置为1024 */
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

#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
volatile uint32_t g_dbg_onframe_hits= 0U;  /* OnFrame 被调用次数 */
#endif
void BSP_LOG_UART1_OnFrame(const uint8_t *data, uint16_t len)
{
    /* ⚠ item[] 必须 ≥ 队列 ItemSize(64)：osMessageQueuePut 按 ItemSize 整块读取。
       三处须同步：.ioc Queues01 的 g_cmd_q item size / 本函数 item[] / DbgConsole_Process 的 lbuf[]。 */
    char item[64];
    if (len == 0U) return;
    if (len > 63U) {
        /* 超长帧：ISR 内不做阻塞发送（中断里不打印），仅计数；
           由 DbgConsole_Process 在任务上下文补一条 "> cmd too long" 回执。 */
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
        g_dbg_cmd_toolong++;
#endif
        return;
    }
    for (uint16_t i = 0U; i < len; i++) item[i] = (char)data[i];
    item[len] = '\0';
#if defined(DBG_TELEMETRY_UART_RX) && DBG_TELEMETRY_UART_RX
    g_dbg_onframe_hits++;
#endif
    osMessageQueuePut(g_cmd_qHandle, item, 0U, 0U);   /* ISR 安全，非阻塞 */
}

/* USER CODE END Application */

