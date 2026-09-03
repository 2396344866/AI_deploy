/*
 * dbg_config.h — L3 调试/遥测总开关（编译期）
 * 单 UART1 输出（VOFA firewater 59ch + 控制台）；CH340 实测 2Mbps，故波特率做宏。
 * 用法：改下方宏 -> Rebuild -> Download；dbg_telemetry.c 须进工程、Components/Debug/Inc 已进 Include Path。
 *       VOFA+ 接 UART1，波特率=DBG_UART_BAUD，firewater 协议。
 * 铁律：DBG_FRAME_N=59 与 dbg_telemetry.c/motor_vofa_telemetry.md 通道映射三者须一致，勿改。
 */
#ifndef DBG_CONFIG_H
#define DBG_CONFIG_H

#include "app_config.h"   /* L1 APP_ENABLE_X：DBG_LOG_<TASK> 正交门控依赖，须先于下方 #if 块可见 */
#include "logger.h"    /* LOG_LVL_* / LOG_COMPILE_MAX_LEVEL / LOG_RUNTIME_DEFAULT_LEVEL：下方 #if 门控依赖 */

/* ===== 三轴正交门控（出不出 = 各轴 AND）=====
 * [A] 级别(递进嵌套): F0<E1<W2<I3<D4<T5；TRACE⊃DEBUG⊃INFO。
 *     嵌套 #if 表达：>=DEBUG 包 >=TRACE，TRACE 是 DEBUG 子域；TRACE 层声明遥测开关，互斥= sink 级静音(见⑧)。
 * [B] 模块(并列): DBG_LOG_<TASK> 选任务产文本，与 [A] AND。
 * [C] 特征(并列): DBG_TELEMETRY_ENABLE 仅声明于 >=TRACE 子域(运行级<TRACE 时宏不存在)，分组再 OR。
 * 铁律: 1) 观测宏/计数器统一定义于此，禁 .c 散落 #define DBG_；
 *       2) 发布态 DBG_LOG_ENABLE=0 + LOG_ENABLED=0 全静音；
 *       3) LOG_COMPILE_MAX_LEVEL>=LOG_RUNTIME_DEFAULT_LEVEL(#error)，文本须先编进、级够才出；
 *       4) 遥测帧=DBG_TELEMETRY_ENABLE && LOG_ENABLED && (IMU||MOTOR||SYSTEM) && 级>=TRACE(logger_set_level(TRACE) 即开)；
 *       5) POST=常开 INFO 里程碑，不经 DBG_LOG_<TASK>；细探针 DBG_LOG_POSTEST 走 Channel B 直发；
 *       6) 新增先在此加宏；7) 本文件(L3)与 app_config.h(L1)正交：APP_ENABLE_X=0 模块消失，DBG_LOG_<TASK>=0 仅静音；
 *       8) 遥测互斥=sink级静音(非逐宏覆盖)：发包(≥TRACE)将 UART1 文本门限压到 WARN，
 *          静音 INFO+DBG_LOG_*+TRACE探针，留 W/E/F；Channel B 永不被静音；POST 因时序豁免仍可见。 */

/* ---- Per-task 调试日志子开关（FreeRTOS 任务级，DEBUG 层）----
 * 仅 DBG_LOG_ENABLE=1 时求值；每任务默认 0，手动开单个。总闸关 子开关强制 0(宏占位避 -Wundef)。
 * 调用处：LOG_D("NET",...) 包 #if DBG_LOG_NET，tag=任务名即可读，无需 NET_LOG_D 别名宏。
 * 注：StartInferenceTask 离线测试日志用 DBG_LOG_DIAG；其 for(;;) 待命为正式代码，不受开关影响。
 * 本层只管"总闸 + 模块编译"派生文本，不引用遥测开关（互斥见铁律⑧）。 */
#if LOG_RUNTIME_DEFAULT_LEVEL >= LOG_LVL_DEBUG
	#ifndef DBG_LOG_ENABLE
		/* ---- 任务级调试文本总闸 ----
		 * 前提: LOG_ENABLED 开 + LOG_RUNTIME_DEFAULT_LEVEL>=DEBUG。两级 AND。
		 * =1 允各任务 DBG_LOG_<TASK> 生效；=0 关全部任务级文本。 */
		#define DBG_LOG_ENABLE         1   /* 默认 1 = 开发态任务文本开；发布态改 0 全静音 */
		/* ---- 逐任务 DBG_LOG_<TASK>：DEBUG 层，仅总闸 + 模块编译 ---- */
		#define DBG_UART_BAUD          921600U   /* UART1/VOFA 口；本机 CH340 在 921600 正常。VOFA 须同步 921600；59ch@100Hz(DECIMATE=2)≈47KB/s < 921600/10≈90KB/s 可用上限，留余量 */
		#if DBG_LOG_ENABLE == 1
			/* DBG_LOG_POSTEST：POST 为常驻核心自检框架，无独立 APP_ENABLE_X，恒随总闸。 */
			#define DBG_LOG_POSTEST     1
			#if defined(APP_ENABLE_INFERENCE) && APP_ENABLE_INFERENCE
				#define DBG_LOG_DIAG    0   /* DIAG 模块内部 detail（POST 内 FaultDiag_ML_Test 的 init/sample/metrics 由本开关门控，非 POST 框架） */
			#else
				#define DBG_LOG_DIAG    0
			#endif
			#if defined(APP_ENABLE_MOTOR) && APP_ENABLE_MOTOR
				#define DBG_LOG_MOTOR   0   /* StartMotorTask 电机环 */
			#else
				#define DBG_LOG_MOTOR   0
			#endif
			#if defined(APP_ENABLE_NETWORK) && APP_ENABLE_NETWORK
				#define DBG_LOG_NET     0   /* StartNetworkTask 网络摘要 */
			#else
				#define DBG_LOG_NET     0
			#endif
			#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
				#define DBG_LOG_SENSOR  1   /* StartSensorTask 姿态/滤波文本 */
			#else
				#define DBG_LOG_SENSOR  0
			#endif
			#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
				#define DBG_LOG_SCREEN  0  /* StartScreenTask 屏显 */
			#else
				#define DBG_LOG_SCREEN  0
			#endif
			#if defined(APP_ENABLE_FLASH) && APP_ENABLE_FLASH
				#define DBG_LOG_FLASH   0  /* Flash 模块内部 detail（POST 内 Flash_Test 的 step 进度由本开关门控，非 POST 框架） */
			#else
				#define DBG_LOG_FLASH   0
			#endif
			#if defined(APP_ENABLE_LOGGER) && APP_ENABLE_LOGGER
				#define DBG_LOG_LOGGER  1   /* StartLoggerTask 日志任务自身 */
			#else
				#define DBG_LOG_LOGGER  0
			#endif
			#if defined(APP_ENABLE_ESP32S3) && APP_ENABLE_ESP32S3
				#define DBG_LOG_ESP32S3   0   /* ESP32S3 模块内部 detail（POST 内 Esp32S3_Test 由本开关门控，非 POST 框架） */
			#else
				#define DBG_LOG_ESP32S3   0
			#endif

			/* ===== TRACE 子域（嵌套于 DEBUG 内；TRACE⊃DEBUG，层层递进）=====
			 * 仅当运行级 >= TRACE 才进入；遥测开关在此声明（运行级<TRACE 时本宏根本不存在，遥测逻辑不编进）。 */
			#if LOG_RUNTIME_DEFAULT_LEVEL >= LOG_LVL_TRACE
				/* ===== 遥测开关（特征层 [C]，仅本 TRACE 子域有效）=====
				 * 0=遥测不编进固件(UART1 仅文本/命令)；=1 仅表"特征编进"，真发包前置条件见铁律④
				 *   （LOG_ENABLED + 任一分组(IMU/MOTOR/SYSTEM)开 + 运行级>=TRACE，且须 LOG_COMPILE_MAX_LEVEL>=TRACE）。
				 * 现场看波形=logger_set_level(TRACE)，回 DEBUG 即停，无需重烧。
				 * 遥测开时 UART1 文本由 BSP_LOG 后端 sink 级静音（见铁律⑧），不逐宏覆盖 DBG_LOG_*。 */
				#ifndef DBG_TELEMETRY_ENABLE
					#define DBG_TELEMETRY_ENABLE   1  /* 遥测编进固件；发包(≥TRACE)自动静音 UART1 文本(<WARN)，留 W/E/F */
				#endif
				/* 实测单帧 408B（59ch × ~6.9B）。发送走【阻塞轮询、逐字节自旋等 TXE 写 TDR】
				 * （BSP_LOG.c: bsp_uart1_emit），且直接嵌在 200Hz(5ms) 传感器任务体内：
				 *   408B @921600(8N1) ≈ 4.43ms 忙等 -> 占满 5ms 周期的 88.5%，
				 *   挤占 I2C 读取 + Madgwick + 控制器时间，且忙等不让出 CPU。
				 * DECIMATE=2：每 2 拍发 1 帧(100Hz)，忙等摊薄为 2.21ms/10ms(≈44%)，
				 *   波特占用 ≈ 408B×100Hz = 40.8KB/s < 921600 可用上限 ~90KB/s，安全。
				 * 注：44ch 时代 DECIMATE=1 的结论已失效，59ch 下勿改回 1（详见 logger_error.md E18 更正）。
				 * 根治方向：改 DMA / TXE 中断 + 环形缓冲（涉及 ISR 安全，另立事件，勿顺手改）。 */
				#define DBG_TELEMETRY_DECIMATE 2

				#if DBG_TELEMETRY_ENABLE == 1
					/* 遥测分组（各自独立 OR，不再依附 DBG_LOG_MOTOR）；真发包需 LOG_ENABLED+任一分组开+级>=TRACE。
					 * 下列索引区间为【59ch 当前布局】；权威源三处必须一致：
					 *   dbg_telemetry.c 枚举 / Ref/vofa_panel.json / Ref/motor_vofa_telemetry.md
					 * ⚠ 44ch 旧布局的索引（24-26 电机、30-33 系统等）已全部作废，勿再引用。 */
					#ifndef DBG_TELEMETRY_IMU
						/* 0-8   ACC  组：raw(g) / flt(g) / res(g)=raw-flt（驱动去极值均值 + 一阶滞后） */
						/* 9-17  GYRO 组：raw(°/s) / flt / res                                        */
						/*               ⚠ 此处是驱动直出 gx，【未扣】Init 标定的 s_gyro_offset，     */
						/*                 故静态仍见 ≈-3.0/-1.9 °/s；融合内部已扣，属正常显示差异。   */
						/* 18-26 MAG  组：raw(counts) / flt(轴系对齐+去极值均值+一阶滞后) / res        */
						/* 27-35 ATT  组：roll,pitch,yaw(后低通) + raw(融合直出) + yaw_gyro/mag_hdg/yaw_innov */
						/* 36-38 GBIAS组：在线零偏估计(°/s)（Init 已扣 s_gyro_offset，静态≈0 为正常）   */
						/* 39-58 CTRL 组：ref/err ×3 轴 + kp/ki/kd + 电机A/B + 系统 + hdg_err          */
					#define DBG_TELEMETRY_IMU      1   /* 组A: ACC/GYRO/MAG/ATT/GBIAS/CTRL 全分组（59ch 固定帧） */
					#endif
					#ifndef DBG_TELEMETRY_MOTOR
						/* 48-50 电机A：速度 / PWM / 目标速度 */
						/* 51-53 电机B：速度 / PWM / 目标速度 */
					#define DBG_TELEMETRY_MOTOR    1   /* 组B: 电机速度环（当前开，可见 motA/B 的 spd/pwm/tgt） */
					#endif
					#ifndef DBG_TELEMETRY_SYSTEM
					/* 54 循环周期(ms)；55 模式；56 运行；57 转向；58 航向误差(°) */
					/* ⚠⚠ CH54 loop_ms 的三重失真（判读必读，详见 sensor_error.md E40）：
					 *   ① 测的是【发包间隔】，不是任务周期：采样点在降采样门之后（dbg_telemetry.c:173，
					 *      而 decimate return 在 108-113 行）→ loop_ms = 拍间隔 × DECIMATE = 5ms×2 = 10ms。
					 *   ② 传感器任务【不是固定 osDelay 周期】，而是等 MPU6050 data-ready 中断：
					 *      freertos.c:691 osSemaphoreAcquire(g_semAttitudeDataReady, 100)，
					 *      节奏由 MPU6050 SMPLRT_DIV=4（1000/(1+4)=200Hz）给出，与 STM32 1ms SysTick
					 *      不同源 → 相位持续漂移；而 dt 用 HAL_GetTick()（1ms 分辨率）相减，
					 *      故真值 9.9x/10.0x 会被量化成 9/10/11，且图案随相位漂移周期性变化
					 *      （"一阵 11-10-9 起伏、一阵恒定 10"）—— 这是 1ms 量化锯齿 + 拍频，非故障。
					 *   ③ 采样点还在任务体中段（I2C读+滤波+融合+外环PID+自治FSM 之后），
					 *      dt 混入了这段执行时间 τ 的逐拍差值（被 TIM7 1ms ISR / Motor(High,100ms) /
					 *      Esp32S3 同级轮转抢占而波动）→ 不能拿它判断真实周期稳定性。
					 *   另：带发送那一拍 SendPoll 阻塞忙等 4.41ms(406B@921600) + I2C/计算 ≈5.2~5.7ms
					 *      > 5ms 采样周期 → 该拍必积压 1 个 data-ready 信号量，下一拍背靠背补跑。
					 *      2 拍窗口内总量守恒（均值仍 10ms），但边界相位被推挤，放大 ±1 tick 抖动。
					 *   兼容性结论：姿态解算 dt 用写死的 1/ATTITUDE_RATE_HZ（attitude.c:259/341，
					 *      Madgwick_Init 同），不读 loop_ms，故此抖动【不进入积分/融合】，无需处理。
					 *   要测真实周期：采样点前移到 acquire 之后第一行，并换 DWT_CYCCNT µs 时基。 */
					#define DBG_TELEMETRY_SYSTEM   1   /* 组C: 系统(周期/模式/运行/转向/航向误差)，当前开 */
					#endif
					#define DBG_FRAME_N            59   /* 固定帧宽：与 dbg_telemetry.c / vofa_panel.json / motor_vofa_telemetry.md 一致，勿改 */
				#endif

				/* ---- UART1 RX 中断链路命中计数器（调试探针，DBG_TELEMETRY_* 家族）----
				 * 仅 LOG_ENABLED 且 LOG_COMPILE_MAX_LEVEL>=TRACE 才编进；遥测开时不编（避免污染波形）。 */
				#if defined(LOG_ENABLED) && (LOG_COMPILE_MAX_LEVEL >= LOG_LVL_TRACE) && (DBG_TELEMETRY_ENABLE == 0)
					#define DBG_TELEMETRY_UART_RX   0    /* 默认开启 DBG_TELEMETRY_UART_RX  */
					#define DEBUG_ISR_CNT_MPU6050_INT   0 /* 默认开启MPU6050 INT 时钟*/
				#else
					#define DBG_TELEMETRY_UART_RX   0
					#define DEBUG_ISR_CNT_MPU6050_INT   0
				#endif
				/* ---- 串口错误注入测试命令（e4/e6，免 USB-TTL 验证 B+C UART 恢复策略）----
				 *   e4=UART4(屏)        -> 需 APP_ENABLE_SCREEN
				 *   e6=USART6(ESP32-S3) -> 需 APP_ENABLE_ESP32S3
				 * 二者皆未使能(如 LOGGER profile)则该命令分支编译期整体删除，零开销；控制台永活、不向空模块分发。 */
				#if (defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN && (DBG_TELEMETRY_ENABLE == 0)) || \
					(defined(APP_ENABLE_ESP32S3) && APP_ENABLE_ESP32S3 && (DBG_TELEMETRY_ENABLE == 0))
					#define DBG_UART_ERR_INJECT_TEST   0  /* 默认开启 UART_ERR_INJECT_TEST */
				#else
					#define DBG_UART_ERR_INJECT_TEST   0
				#endif
			#endif /* >= TRACE (嵌套于 DEBUG) */
		#endif /* DBG_LOG_ENABLE == 1 */
		#else
		/* 总闸已由外部定义(=0)：子开关强制 0（宏占位避 -Wundef） */
		#define DBG_LOG_DIAG    0
		#define DBG_LOG_MOTOR   0
		#define DBG_LOG_NET     0
		#define DBG_LOG_SENSOR  0
		#define DBG_LOG_SCREEN  0
		#define DBG_LOG_FLASH   0
		#define DBG_LOG_LOGGER  0
		#define DBG_LOG_ESP32S3   0
		#define DBG_LOG_POSTEST   0
		#endif /* DBG_LOG_ENABLE (ifndef) */
#endif /* >= DEBUG */
#endif /* DBG_CONFIG_H */
