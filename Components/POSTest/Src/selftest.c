/*
 * selftest.c —— 上电自检（POST）实现
 * ---------------------------------------------------------------------------
 * 由独立 Task_Test（CubeMX 生成，osPriorityHigh）在调度器启动后一次性调用
 * Selftest_RunAll()。每个 Xxx_Test 由自身的 APP_ENABLE_X 门控：
 *   - 模块已编进固件(APP_ENABLE_X!=0) -> 跑该硬件/运行时自检；
 *   - 模块被 profile 关掉(APP_ENABLE_X 未定义/0) -> 打 SKIPPED 并返回 0，
 *     不跑、不卡死（与 watchdog_heartbeat 的监视集合保持同一真相源 APP_ENABLE_X）。
 * 故 POST 自动跟随当前 profile：最小 profile(如 LOGGER)下只跑 Logger_Test，
 * 不会去跑无模型/无硬件的自测而冻结 -> 根除 IWDG 复位环。
 * ---------------------------------------------------------------------------
 */

/* 编译期自检标记：构建日志（Build Output）里若看不到这一行，说明本文件
 * 未加入 Keil 源组 -> 不会生成 selftest.o -> 链接期 L6218E: Undefined symbol
 * Selftest_RunAll。出现 L6218E 第一反应：看构建日志有没有下面这行。 */


#include "cmsis_os2.h"
#include "main.h"          /* HAL_GetTick */
#include "ai_infer.h"      /* AI_Inference + TestResults_t（类型已提至本头） */

/* 推理测试数据集仅在 APP_ENABLE_INFERENCE 编进固件时使用；最小 profile（如 LOGGER）下
 * FaultDiag_ML_Test 整段被 #if 掉，数据集无人引用 -> 包在门控内消除「defined but not used」告警。
 * TEST_DATASET_IMPL 必须位于 test_dataset_processed.h 之前（展开数组定义，避免被 include 守卫跳过导致 L6218E）。 */
#if defined(APP_ENABLE_INFERENCE) && APP_ENABLE_INFERENCE
#define TEST_DATASET_IMPL
#include "test_dataset_processed.h"  /* test_features_processed / test_labels / NUM_TEST_SAMPLES */
#endif
#include "logger.h"        /* LOG_I/E/W, logger_flush_to_flash */
#include "dbg_config.h"    /* DBG_LOG_DIAG */
#include "app_config.h"    /* APP_ENABLE_*（本文件函数常驻，不依赖其门控） */
#include "BSP_W25Q64.h"    /* W25QXX_BufferRead / W25QXX_SectorErase / W25QXX_Test */
#include "iwdg.h"          /* log_wdt_feed */
#include "watchdog_heartbeat.h" /* POST 收尾 watchdog_arm() + TIM7 运行期心跳喂狗 */

/* POST 自检输出级别规划（见 logger.h 工业语义）：
 *   - 系统级里程碑(start/done/[i/n]/OK)走 LOG_EMIT_DIRECT(LOG_LVL_INFO,"I",tag,...)（Channel B 保证通道，绕过运行级、同步直发 USART1，
 *     即便抽空任务/调度器异常也必落线）；
 *   - CRITICAL FAIL（紧随 NVIC_SystemReset 终止系统）走 LOG_F（FATAL 语义=导致系统终止/重启）；
 *   - non-critical FAIL 走 LOG_W（暗示潜在问题、系统继续）；
 *   - 分步诊断走 LOG_I（Channel A 异步，证明按预期工作）；
 *   - 内部执行路径/变量值探针走 LOG_EMIT_DIRECT(LOG_LVL_DEBUG,"D","POSTEST",...)（DEBUG，Channel B 同步，DBG_LOG_POSTEST 门控；
 *     默认 build 不编进；LOG_EMIT_DIRECT 仅受编译期 LOG_COMPILE_MAX_LEVEL 裁剪、绕过运行级，故定位卡死行时 DBG_LOG_POSTEST=1 且 LOG_COMPILE_MAX_LEVEL>=DEBUG 即出）。
 * 均不经 DBG_LOG_<TASK> 管控（POST 为一次性 boot 状态，无持续 DEBUG 通道）。 */


/* 跨 TU 共享对象（定义见 freertos.c） */
extern TestResults_t     g_Test_results;
extern osMutexId_t       InferenceDataMutexHandle;

/* 看门狗 arming 经 watchdog_heartbeat.h：POST 收尾调用 watchdog_arm()（置 armed + 新鲜戳）。
 * POST 期间 TIM7 不喂狗（watchdog_should_feed() 返回 0），由各 Xxx_Test 内的 log_wdt_feed()
 * 协作式喂狗（卡死→IWDG 复位抓到；慢测试如 ML 自检 ~130s 靠协作喂活过）。 */

/* 黑匣子保留扇区（与 BSP_W25Q64.c 一致：W25Q64=8MB 末 4KB 起始） */
#ifndef W25Q_CRASHLOG_ADDR
#define W25Q_CRASHLOG_ADDR  0x7FF000U
#endif

/* ML 自检通过阈值：accuracy 低于此值判失败（关键任务→触发复位） */
#define ML_TEST_ACC_THRESHOLD  0.85f

/* ===================== 外部 Flash 自检（最先跑；按 APP_ENABLE_FLASH 门控） ===================== */
#if defined(APP_ENABLE_FLASH) && APP_ENABLE_FLASH
static void flash_hexdump(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 16) {
        char line[64];
        int  n = 0;
        n += snprintf(line + n, sizeof(line) - (size_t)n, "%04X: ", (unsigned)i);
        for (uint32_t j = 0; j < 16; j++) {
            if (i + j < len) n += snprintf(line + n, sizeof(line) - (size_t)n, "%02X ", buf[i + j]);
            else             n += snprintf(line + n, sizeof(line) - (size_t)n, "   ");
        }
        n += snprintf(line + n, sizeof(line) - (size_t)n, " ");
        for (uint32_t j = 0; j < 16 && (i + j) < len; j++) {
            uint8_t c = buf[i + j];
            n += snprintf(line + n, sizeof(line) - (size_t)n, "%c", ((c >= 0x20 && c < 0x7F) ? c : '.'));
        }
        LOG_I("POSTEST", "%s", line);
    }
}

int Flash_Test(void)
{
    #define BLACKBOX_DUMP_BYTES  512U
    static uint8_t buf[BLACKBOX_DUMP_BYTES];   /* static → BSS，不占任务栈 */
    uint32_t t0 = HAL_GetTick();
    log_wdt_feed();   /* POST 运行在 Task_Test(HIGH)，喂狗点仅在最低优先级 Logger；每步主动喂+让出，防饿死 */

    /* 1. 读黑匣子扇区并打印（看上次崩溃日志） */
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Flash_Test step1 enter: read blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    #endif
    LOG_I("POSTEST", "[Flash] step1: read blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    W25QXX_BufferRead(buf, W25Q_CRASHLOG_ADDR, BLACKBOX_DUMP_BYTES);
    log_wdt_feed();
    /* 黑匣子扇区非全 0xFF(已擦除态) => 上次运行遗留崩溃日志，提示潜在稳定性问题（WARN 语义） */
    int bb_dirty = 0;
    for (uint32_t k = 0; k < BLACKBOX_DUMP_BYTES; k++) { if (buf[k] != 0xFFU) { bb_dirty = 1; break; } }
    if (bb_dirty) LOG_W("POSTEST", "Flash blackbox non-empty @0x%X: prior crash log suspected",
                        (unsigned)W25Q_CRASHLOG_ADDR);
    osDelay(1);   /* 让出 CPU 给 Logger 刷上面的日志 + 喂狗（仅阻塞本 HIGH 任务 1 tick） */
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Flash_Test step1 done: elapsed=%lu ms first_byte=0x%02X",
              (unsigned long)(HAL_GetTick() - t0), (unsigned)buf[0]);
    #endif
    LOG_I("POSTEST", "Flash blackbox first %u bytes:", BLACKBOX_DUMP_BYTES);
    flash_hexdump(buf, BLACKBOX_DUMP_BYTES);

    /* 2. 清空（给本次运行留干净黑匣子） */
    uint32_t t2 = HAL_GetTick();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Flash_Test step2 enter: erase blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    #endif
    LOG_I("POSTEST", "[Flash] step2: erase blackbox @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);
    W25QXX_SectorErase(W25Q_CRASHLOG_ADDR);   /* 内部 WaitForWriteEnd 已加 1.5s 超时，不会死等 */
    log_wdt_feed();
    osDelay(1);
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Flash_Test step2 done: elapsed=%lu ms", (unsigned long)(HAL_GetTick() - t2));
    #endif
    LOG_I("POSTEST", "Flash blackbox erased @0x%X", (unsigned)W25Q_CRASHLOG_ADDR);

    /* 3. 芯片完整性自检（JEDEC + 擦/写/读回环）；JEDEC 由 W25QXX_Test 内部 printf 直发 USART1 */
    uint32_t t3 = HAL_GetTick();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Flash_Test step3 enter: W25QXX_Test");
    #endif
    LOG_I("POSTEST", "[Flash] step3: W25QXX_Test (JEDEC + R/W loopback)");
    int rc = W25QXX_Test();
    log_wdt_feed();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Flash_Test step3 done: rc=%d elapsed=%lu ms",
              rc, (unsigned long)(HAL_GetTick() - t3));
    #endif
    if (rc != 0) {
        LOG_E("POSTEST", "Flash chip self-test FAIL rc=%d "
              "(查 W25Q 接线: CS/CLK/MOSI/DO(MISO)；DO 悬空->状态寄存器读0xFF 会误判 busy 致原代码死等)",
              rc);
    } else {
        LOG_I("POSTEST", "Flash chip self-test OK");
    }
    return rc;
}
#endif /* APP_ENABLE_FLASH */

/* ===================== 故障诊断 ML 自检（关键；按 APP_ENABLE_INFERENCE 门控） ===================== */
#if defined(APP_ENABLE_INFERENCE) && APP_ENABLE_INFERENCE
int FaultDiag_ML_Test(void)
{
    /* 1. 初始化（只运行一次） */
    float dummy_out[4];
    AI_Inference((float *)test_features_processed[0], dummy_out);
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "FaultDiag_ML_Test init inference done");
    #endif
#if DBG_LOG_DIAG
    LOG_I("DIAG", "AI_deploy ML self-test: Beginning Batch Inference...");
#endif

    int tp[4] = {0}, fp[4] = {0}, fn[4] = {0};
    int correct_total = 0;
    uint32_t start_time = HAL_GetTick();

    /* 2. 推理循环（长段 ≈1605×81ms≈130s，远超 IWDG 窗口≈4.1s，必须自喂狗） */
    for (int i = 0; i < NUM_TEST_SAMPLES; i++) {
        float test_output[4] = {0};
        int pred   = AI_Inference((float *)test_features_processed[i], test_output);
        int actual = test_labels[i];

        if (pred == actual) { correct_total++; tp[actual]++; }
        else                 { fp[pred]++;     fn[actual]++; }

        if (i % 10 == 0) osDelay(1);   /* 让出 CPU，避免饿死低优先级任务 */
#if DBG_LOG_POSTEST
        {   /* 每样本精度/中间 rc 探针（Channel B 同步直发，POST 期间实时可见） */
            float running_acc = (float)correct_total / (i + 1);
            LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "ML sample[%d/%d] pred=%d actual=%d running_acc=%.4f",
                  i, NUM_TEST_SAMPLES, pred, actual, running_acc);
            if (i == NUM_TEST_SAMPLES / 4 || i == NUM_TEST_SAMPLES / 2 ||
                i == (NUM_TEST_SAMPLES * 3) / 4)
                LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "ML milestone 25/50/75%% reached @sample %d", i);
        }
#endif
        log_wdt_feed();                /* 协同喂狗：长段自踢，防最低优先级 Logger 被饿死误复位 */
    }

    /* 3. 指标计算 */
    uint32_t end_time = HAL_GetTick();
    float total_time_ms = (float)(end_time - start_time);
    float macro_precision = 0.0f, macro_recall = 0.0f, macro_f1 = 0.0f;
    for (int i = 0; i < 4; i++) {
        float p = 0.0f, r = 0.0f;
        if ((tp[i] + fp[i]) > 0) p = (float)tp[i] / (tp[i] + fp[i]);
        if ((tp[i] + fn[i]) > 0) r = (float)tp[i] / (tp[i] + fn[i]);
        macro_precision += p; macro_recall += r;
        if ((p + r) > 0) macro_f1 += (2.0f * p * r) / (p + r);
    }
    macro_precision /= 4.0f; macro_recall /= 4.0f; macro_f1 /= 4.0f;
    float overall_accuracy = (float)correct_total / NUM_TEST_SAMPLES;

    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "FaultDiag_ML_Test metrics: acc=%.4f prec=%.4f recall=%.4f f1=%.4f total=%.2fms",
              overall_accuracy, macro_precision, macro_recall, macro_f1,
              (float)(end_time - start_time));
    #endif
#if DBG_LOG_DIAG
    LOG_I("DIAG", "ML Summary: samples=%d acc=%.4f prec=%.4f recall=%.4f f1=%.4f total=%.2fms per=%.4fms",
          NUM_TEST_SAMPLES, overall_accuracy, macro_precision, macro_recall, macro_f1,
          total_time_ms, total_time_ms / NUM_TEST_SAMPLES);
#endif

    /* 4. 写共享结构（生产逻辑：供 NetworkTask 读取） */
    if (osMutexAcquire(InferenceDataMutexHandle, osWaitForever) == osOK) {
        g_Test_results.num_test_samples  = NUM_TEST_SAMPLES;
        g_Test_results.overall_accuracy  = overall_accuracy;
        g_Test_results.macro_precision   = macro_precision;
        g_Test_results.macro_recall      = macro_recall;
        g_Test_results.macro_f1          = macro_f1;
        g_Test_results.total_time_ms     = total_time_ms;
        g_Test_results.data_is_ready     = 1;
        osMutexRelease(InferenceDataMutexHandle);
    }

    /* 5. 判定：accuracy 低于阈值即失败 */
    if (overall_accuracy < ML_TEST_ACC_THRESHOLD) {
        LOG_E("POSTEST", "ML self-test FAIL: acc=%.4f < %.2f", overall_accuracy, ML_TEST_ACC_THRESHOLD);
        return -1;
    }
    return 0;
}
#endif /* APP_ENABLE_INFERENCE */

/* ===================== 各模块自检（桩，待补全；按 APP_ENABLE_X 门控，未使能则不编译/不进表） ===================== */
#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
int Sensor_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Sensor_Test enter");
#endif
    LOG_I("POSTEST", "Sensor_Test: skeleton (TODO) -> OK");
    return 0;
}
#endif
#if defined(APP_ENABLE_ESP32S3) && APP_ENABLE_ESP32S3
int Esp32S3_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Esp32S3_Test enter");
#endif
    LOG_I("POSTEST", "Esp32S3_Test: skeleton (TODO) -> OK");
    return 0;
}
#endif
#if defined(APP_ENABLE_MOTOR) && APP_ENABLE_MOTOR
int Motor_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Motor_Test enter");
#endif
    LOG_I("POSTEST", "Motor_Test: skeleton (TODO) -> OK");
    return 0;
}
#endif
#if defined(APP_ENABLE_NETWORK) && APP_ENABLE_NETWORK
int Network_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Network_Test enter");
#endif
    LOG_I("POSTEST", "Network_Test: skeleton (TODO) -> OK");
    return 0;
}
#endif
#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
int Screen_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Screen_Test enter");
#endif
    LOG_I("POSTEST", "Screen_Test: skeleton (TODO) -> OK");
    return 0;
}
#endif

#if defined(APP_ENABLE_LOGGER) && APP_ENABLE_LOGGER
/* ===================== Logger 冒烟测试（测 logger 专用，无硬件依赖） ===================== */
int Logger_Test(void)
{
    /* 受控打印各优先级样例行，验证环形缓冲/级别门控/flush；
     * 低于 LOG_COMPILE_MAX_LEVEL 的行被编译期裁剪（运行期零开销），
     * 这正是 logger 机制正确性的一部分——可借此确认当前级别门控是否生效。
     * 级别规划（见 logger.h 工业语义）：
     *   - F/E/W/I 样例行：各走对应级别通道（F/E=Channel B 保证、W/I=Channel A），
     *     属 Logger 自测"证明机制按预期工作"，默认编译级即出（INFO 里程碑式）。
     *   - D/T 样例行：仅编译级>=DEBUG/TRACE 出现，验证高等级裁剪生效。
     *   - 内部执行路径探针(CKPT)属 DEBUG 语义，走 LOG_EMIT_DIRECT(LOG_LVL_DEBUG,"D","POSTEST",...)(Channel B 同步)；
 *     默认 build(未开 DBG_LOG_POSTEST)不编进；需定位 POST 卡死行时
 *     DBG_LOG_POSTEST=1 且 LOG_COMPILE_MAX_LEVEL>=DEBUG 即出（LOG_EMIT_DIRECT 绕过运行级；
 *     LOG_D/T 样例行另需 logger_set_level(DEBUG/TRACE)）。仍 Channel B 保证落线。
     * POST 期间 TIM7 未接管喂狗，本函数每步前主动喂狗防饿死 IWDG。 */
    log_wdt_feed();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test enter (runtime_lvl=%u compile_max=%u)",
              (unsigned)logger_get_level(), (unsigned)LOG_COMPILE_MAX_LEVEL);
    #endif

    log_wdt_feed(); LOG_F("LOGTEST", "logger smoke [FATAL] sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_F (Channel B 同步直发)");
    #endif
    log_wdt_feed(); LOG_E("LOGTEST", "logger smoke [ERROR] sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_E");
    #endif
    log_wdt_feed(); LOG_W("LOGTEST", "logger smoke [WARN]  sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_W (Channel A 入环)");
    #endif
    log_wdt_feed(); LOG_I("LOGTEST", "logger smoke [INFO]  sample");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_I");
    #endif

#if LOG_COMPILE_MAX_LEVEL >= LOG_LVL_DEBUG
    log_wdt_feed(); LOG_D("LOGTEST", "logger smoke [DEBUG] sample (仅编译级>=DEBUG 出现)");
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_D");
    #endif
#endif
#if LOG_COMPILE_MAX_LEVEL >= LOG_LVL_TRACE
    log_wdt_feed(); LOG_T("LOGTEST", "logger smoke [TRACE] sample (仅编译级>=TRACE 出现)");
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test after LOG_T");
    #endif
#endif
    log_wdt_feed();
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Logger_Test exit OK");
    #endif
    return 0;
}
#endif

/* ===================== 一站式调度 ===================== */
typedef struct {
    const char *name;
    int (*test)(void);
    int  critical;   /* 1=关键（失败→存档+复位）；0=非关键（失败→继续） */
} selftest_entry_t;

/* 自检表与 APP_ENABLE_X 同一真相源：未使能模块的函数不编译、也不进表，
 * 故 logger 等最小 profile 下数组只含 Logger 一项，无任何硬件/无模型自测噪声。 */
static const selftest_entry_t g_selftests[] = {
#if defined(APP_ENABLE_FLASH) && APP_ENABLE_FLASH
    {"Flash",     Flash_Test,          1},
#endif
#if defined(APP_ENABLE_INFERENCE) && APP_ENABLE_INFERENCE
    {"Inference", FaultDiag_ML_Test,   1},
#endif
#if defined(APP_ENABLE_SENSOR) && APP_ENABLE_SENSOR
    {"Sensor",    Sensor_Test,         1},
#endif
#if defined(APP_ENABLE_ESP32S3) && APP_ENABLE_ESP32S3
    {"Esp32S3",   Esp32S3_Test,        1},
#endif
#if defined(APP_ENABLE_MOTOR) && APP_ENABLE_MOTOR
    {"Motor",     Motor_Test,          0},
#endif
#if defined(APP_ENABLE_NETWORK) && APP_ENABLE_NETWORK
    {"Network",   Network_Test,        0},
#endif
#if defined(APP_ENABLE_SCREEN) && APP_ENABLE_SCREEN
    {"Screen",    Screen_Test,         0},
#endif
#if defined(APP_ENABLE_LOGGER) && APP_ENABLE_LOGGER
    {"Logger",    Logger_Test,         0},
#endif
};

int Selftest_RunAll(void)
{
    LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "=== Power-On Self-Test start (%u modules) ===",
          (unsigned)(sizeof(g_selftests) / sizeof(g_selftests[0])));
    uint32_t post_t0 = HAL_GetTick();   /* POST 总耗时基准（DEBUG 级汇总，见结尾） */
    /* 系统里程碑走 LOG_EMIT_DIRECT(LOG_LVL_INFO,"I",...)（Channel B）同步直发，不再依赖 StartLoggerTask 抽空；
     * POST 期间狗由本函数内 log_wdt_feed() 协作喂，关键进度必落线。 */

    for (size_t i = 0; i < sizeof(g_selftests) / sizeof(g_selftests[0]); i++) {
        const selftest_entry_t *e = &g_selftests[i];
        log_wdt_feed();   /* 每模块进测前协作喂狗：某测试短暂阻塞也不致 IWDG 误复位 */
                #if DBG_LOG_POSTEST
            LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Selftest_RunAll: entering [%u/%u] %s (critical=%d)",
                  (unsigned)(i + 1),
                  (unsigned)(sizeof(g_selftests) / sizeof(g_selftests[0])), e->name, e->critical);
        #endif
        LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "[%u/%u] %s ...", (unsigned)(i + 1),
              (unsigned)(sizeof(g_selftests) / sizeof(g_selftests[0])), e->name);

        int rc = e->test();
        if (rc != 0) {
            if (e->critical) {
                LOG_F("POSTEST", "CRITICAL %s FAIL rc=%d -> flush blackbox + NVIC_SystemReset",
                      e->name, rc);
                logger_flush_to_flash();   /* 崩溃前把最近日志落盘 */
                NVIC_SystemReset();        /* 看门狗式复位 + 错误已留痕 */
                /* 不会到达 */
            } else {
                LOG_W("POSTEST", "non-critical %s FAIL rc=%d (continue)", e->name, rc);
            }
        } else {
            LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "%s OK", e->name);
        }
    }

    LOG_EMIT_DIRECT(LOG_LVL_INFO, "I", "POSTEST", "=== Self-Test done ===");
    #if DBG_LOG_POSTEST
        LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Self-Test total elapsed=%lu ms (boot+POST window vs IWDG)",
              (unsigned long)(HAL_GetTick() - post_t0));
    #endif
    watchdog_arm();   /* POST 收尾：arming 看门狗，TIM7 此后接管、要求各被监视任务持续 kick 才喂 */
    return 0;
}
