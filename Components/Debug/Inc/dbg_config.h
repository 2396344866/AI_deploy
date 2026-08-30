/*
 * dbg_config.h — L3 调试/遥测总开关（编译期）
 * 单 UART1 输出（VOFA firewater 44ch + 控制台）；CH340 实测 2Mbps，故波特率做宏。
 * 用法：改下方宏 -> Rebuild -> Download；dbg_telemetry.c 须进工程、Components/Debug/Inc 已进 Include Path。
 *       VOFA+ 接 UART1，波特率=DBG_UART_BAUD，firewater 协议。
 * 铁律：DBG_FRAME_N=44 与 dbg_telemetry.c/README 通道映射三者须一致，勿改。
 */
#ifndef DBG_CONFIG_H
#define DBG_CONFIG_H

#include "logger.h"    /* LOG_LVL_* / LOG_COMPILE_MAX_LEVEL / LOG_RUNTIME_DEFAULT_LEVEL：下方 #if 门控依赖 */




/* ===== 三轴正交门控（出不出 = 各轴 AND）=====
 * [A] 级别(递进): FATAL0<ERR1<WARN2<INFO3<DEBUG4<TRACE5；级=TRACE 时 DEBUG 也出。
 * [B] 模块(并列): DBG_LOG_<TASK> 选哪任务产文本，与 [A] AND。
 * [C] 特征(并列): DBG_TELEMETRY_ENABLE 选遥测帧是否编进；分组再独立 OR。
 * 铁律: 1) 观测宏/计数器统一定义于此，禁 .c 散落 #define DBG_；
 *       2) 发布态 DBG_LOG_ENABLE=0 + LOG_ENABLED=0 全静音；
 *       3) LOG_COMPILE_MAX_LEVEL>=LOG_RUNTIME_DEFAULT_LEVEL(#error)，文本须先编进、级够才出；
 *       4) 遥测帧=DBG_TELEMETRY_ENABLE && LOG_ENABLED && (IMU||MOTOR||SYSTEM) && 级>=TRACE(logger_set_level(TRACE) 即开)；
 *       5) POST=常开 INFO 里程碑，不经 DBG_LOG_<TASK>；细节探针 DBG_LOG_POSTEST(默认0) 走 Channel B 直发；
 *       6) 新增先在此加宏；7) 本文件(L3)与 app_config.h(L1)正交：APP_ENABLE_X=0 模块消失，DBG_LOG_<TASK>=0 仅静音。 */






/* ---- Per-task 调试日志子开关（FreeRTOS 任务级）----
 * 仅 DBG_LOG_ENABLE=1 时求值；每任务默认 0，手动开单个。总闸关子开关强制 0(宏占位避 -Wundef)。
 * 调用处：LOG_D("NET",...) 包 #if DBG_LOG_NET，tag=任务名即可读，无需 NET_LOG_D 别名宏。
 * 注：StartInferenceTask 离线测试日志用 DBG_LOG_DIAG；其 for(;;) 待命为正式代码，不受开关影响。 */
#if LOG_RUNTIME_DEFAULT_LEVEL >= LOG_LVL_DEBUG
	#ifndef DBG_LOG_ENABLE
			/* ---- 任务级调试文本总闸 ----
			 * 前提: LOG_ENABLED 开 + LOG_RUNTIME_DEFAULT_LEVEL>=DEBUG。两级 AND。
			 * =1 允各任务 DBG_LOG_<TASK> 生效；=0 关全部任务级文本(不管 VOFA 遥测，由 DBG_TELEMETRY_ENABLE 管)。
			 * 上电自检(W25QXX_Test 等)详情由 DBG_LOG_POSTEST 控。 */
			#define DBG_LOG_ENABLE         1   /* 默认 0 = 任务级调试文本全关（发布态） */
			#if DBG_LOG_ENABLE == 1
				#define DBG_LOG_DIAG    0   /* 离线推理测试(上电一次) */
				#define DBG_LOG_MOTOR   0   /* StartMotorTask 电机环 */
				#define DBG_LOG_NET     0   /* StartNetworkTask 网络摘要 */
				#define DBG_LOG_SENSOR  0   /* StartSensorTask 姿态/滤波文本 */
				#define DBG_LOG_SCREEN  0   /* StartScreenTask 屏显 */
				#define DBG_LOG_FLASH   0   /* StartFlashTask Flash 文本 */
				#define DBG_LOG_LOGGER  0   /* StartLoggerTask 日志任务自身 */
				#define DBG_LOG_ESP32S3   0   /* StartEsp32S3Task 图像结论帧 */
				#define DBG_LOG_POSTEST   1   /* Selftest 逐函数探针(定位卡死/复位环)，仅 DEBUG build 出 */
			#else
				/* 总闸关：子开关强制 0（宏占位避 -Wundef） */
				#define DBG_LOG_DIAG    0
				#define DBG_LOG_MOTOR   0
				#define DBG_LOG_NET     0
				#define DBG_LOG_SENSOR  0
				#define DBG_LOG_SCREEN  0
				#define DBG_LOG_FLASH   0
				#define DBG_LOG_LOGGER  0
				#define DBG_LOG_ESP32S3   0
				#define DBG_LOG_POSTEST   0
			#endif
			/* ===== 串口与速率 ===== */
			#define DBG_UART_BAUD          921600U   /* UART1/VOFA 口；本机 CH340 在 921600 正常。VOFA 须同步 921600；44ch@200Hz≈70KB/s < 115KB/s 上限 */
	#endif

#endif



#if LOG_RUNTIME_DEFAULT_LEVEL >= LOG_LVL_TRACE
	/* ===== 遥测总开关（特征层 [C]）=====
	 * 0=遥测不编进固件(UART1 仅文本/命令)；=1 仅表"特征编进"，真发包还需：
	 *   LOG_ENABLED + 任一分组(IMU/MOTOR/SYSTEM)开 + 运行级>=TRACE(LOG_COMPILE_MAX_LEVEL>=TRACE 才编得进)。
	 * 现场看波形=logger_set_level(TRACE)，回 DEBUG 即停，无需重烧。 */
	#define DBG_TELEMETRY_ENABLE   1
	#define DBG_TELEMETRY_DECIMATE 1   /* 44ch帧≈353B，200Hz≈70KB/s，满速率不降采样 */

	#endif
	#if DBG_TELEMETRY_ENABLE == 1
				/* 临时 ISR 计数器：DEBUG_ISR_CNT_<SRC>_<EVT>，默认0不编译；调中断链路时改1配 Keil Watch。禁散写业务代码。 */
				#ifndef DEBUG_ISR_CNT_MPU6050_INT
				#define DEBUG_ISR_CNT_MPU6050_INT   0
				#endif
				/* 遥测分组（各自独立 OR，不再依附 DBG_LOG_MOTOR）；真发包需 LOG_ENABLED+任一分组开+级>=TRACE。 */
				#ifndef DBG_TELEMETRY_IMU
					#define DBG_TELEMETRY_IMU      1   /* 组A: MPU6050 原始/滤波+姿态角+参考角误差+增益 (0-2 accel LSB/3-5 gyro LSB/6-8 filt g/9-11 filt °/s/12-14 姿态/15-20 参考角误差/21-23 增益/34-36 原始欧拉/37-43 磁力计) */
				#endif
				#ifndef DBG_TELEMETRY_MOTOR
					#define DBG_TELEMETRY_MOTOR    1   /* 组B: 电机速度环 (24-26 A:速/PWM/目标;27-29 B;30 周期ms;31 模式;32 运行;33 转向) */
				#endif
				#ifndef DBG_TELEMETRY_SYSTEM
					#define DBG_TELEMETRY_SYSTEM   0   /* 组C: 系统(周期/模式/运行/转向)，默认关 */
				#endif
				#define DBG_FRAME_N            44   /* 固定帧宽：与 dbg_telemetry.c/README 映射一致，勿改 */

	#endif





#endif /* DBG_CONFIG_H */
