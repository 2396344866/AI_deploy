/*
 * app_config.h (Components/Debug/Inc) — L1 功能包含门控（编译期）
 * 决定"哪些模块编进固件 / 哪些 FreeRTOS 任务被创建"。
 * 三层正交：本文件=L1 功能(APP_ENABLE_X 未定义=0 -> 模块不编、任务不建)；
 *           logger.h=L2 级别(LOG_COMPILE_MAX_LEVEL/LOG_RUNTIME_DEFAULT_LEVEL)；
 *           dbg_config.h=L3 文本(DBG_LOG_<TASK>=0 -> 模块跑但静音)。
 * 用户接口：顶部 #define 一个 APP_PROFILE_* 即选"目标+上游依赖+LOGGER"；不选=默认全功能。
 *           APP_ENABLE_X 是 profile 派生宏，勿手动 #define(需未覆盖组合就新建 APP_PROFILE_*)。
 *           未定义即 0（#if 按 0 安全）；Logger 常驻核心，每 profile 与默认全功能均含。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H


#define APP_PROFILE_LOGGER     /* 仅 LOGGER（日志底座） */
//#define APP_PROFILE_SENSOR   /* SENSOR + 上游依赖 + LOGGER */


/* 预设组（选 1 个，其余注释掉）；自动开 目标+上游依赖+LOGGER。
 * 依赖: ESP32S3->Inference; Sensor->Inference/Motor/Screen; Inference->Network; Logger 被所有依赖。 */
#ifdef APP_PROFILE_SENSOR
  #define APP_ENABLE_SENSOR  1
  #define APP_ENABLE_LOGGER  1
#endif
#ifdef APP_PROFILE_MOTOR
  #define APP_ENABLE_MOTOR   1
  #define APP_ENABLE_SENSOR  1
  #define APP_ENABLE_LOGGER  1
#endif
#ifdef APP_PROFILE_ESP32S3
  #define APP_ENABLE_ESP32S3 1
  #define APP_ENABLE_LOGGER  1
#endif
#ifdef APP_PROFILE_INFERENCE
  #define APP_ENABLE_INFERENCE 1
  #define APP_ENABLE_ESP32S3   1
  #define APP_ENABLE_SENSOR    1
  #define APP_ENABLE_LOGGER    1
#endif
#ifdef APP_PROFILE_NETWORK
  #define APP_ENABLE_NETWORK   1
  #define APP_ENABLE_INFERENCE 1
  #define APP_ENABLE_ESP32S3   1
  #define APP_ENABLE_SENSOR    1
  #define APP_ENABLE_LOGGER    1
#endif
#ifdef APP_PROFILE_SCREEN
  #define APP_ENABLE_SCREEN  1
  #define APP_ENABLE_SENSOR  1
  #define APP_ENABLE_LOGGER  1
#endif
#ifdef APP_PROFILE_FLASH
  #define APP_ENABLE_FLASH   1
  #define APP_ENABLE_LOGGER  1
#endif
#ifdef APP_PROFILE_LOGGER
  #define APP_ENABLE_LOGGER  1
  /* 其余 7 个业务 APP_ENABLE_X 保持未定义=0：模块整段不编、任务空跑，仅 Logger 干活 */
#endif
/* 看门狗调试开关：1=正常(IWDG+TIM7 心跳，冻结即复位)；0=调试(跳过 IWDG 初始化，
 *   log_wdt_feed()/TIM7 喂狗全 no-op，真实故障留串口便于定位)。定位完改回 1！可用 Keil Define 覆盖。 */
#ifndef APP_ENABLE_WATCHDOG
  #define APP_ENABLE_WATCHDOG  0
#endif


/* 默认全功能（未选任何 profile）：所有 APP_ENABLE_X=1，与改造前一致；
 * 选任意 profile 后，未列其内的模块保持未定义=0 不编译。 */
#if !defined(APP_PROFILE_SENSOR) && !defined(APP_PROFILE_MOTOR) && \
    !defined(APP_PROFILE_ESP32S3) && !defined(APP_PROFILE_INFERENCE) && \
    !defined(APP_PROFILE_NETWORK) && !defined(APP_PROFILE_SCREEN) && \
    !defined(APP_PROFILE_FLASH) && !defined(APP_PROFILE_LOGGER)
  #define APP_ENABLE_INFERENCE 1
  #define APP_ENABLE_MOTOR     1
  #define APP_ENABLE_NETWORK   1
  #define APP_ENABLE_SENSOR    1
  #define APP_ENABLE_SCREEN    1
  #define APP_ENABLE_FLASH     1
  #define APP_ENABLE_ESP32S3   1
  #define APP_ENABLE_LOGGER    1
#endif

#endif /* APP_CONFIG_H */
