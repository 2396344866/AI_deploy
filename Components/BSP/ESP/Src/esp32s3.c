/**
  * @file    esp32s3.c
  * @brief   ESP32-S3 CAM image co-processor comms (STM32 H743 RX side)
  * @note    Protocol: see Pinout Sec.12.6; debug uniformly via LOG_I("ESP32", ...) over UART1 to PC.
  *          Code only references CubeMX-generated huart6 / main.h macros, no hard-coded pins.
  *          This file is a self-built Components/BSP module; CubeMX regeneration will not overwrite it.
  */
#include "main.h"          /* huart6 + HAL */
#include "esp32s3.h"
#include "logger.h"        /* LOG_I / LOG_W (task context) */
#include "dbg_config.h"    /* DBG_LOG_ESP32S3 switch (unified debug scheme with Components/Debug) */
#include "watchdog_heartbeat.h" /* 运行期存活探针：每收到一帧踢一次（TIM7 据此判 IWDG 是否喂） */

/* ====================== Static state ====================== */
static uint8_t s_rx_buf[ESP32S3_RX_BUF_SIZE];      /* ReceiveToIdle_IT RX buffer */

/* Latest frame snapshot (written by parse task, read by others) */
static esp32s3_result_t s_latest;
static uint32_t s_frame_cnt = 0;          /* Global frame seq, ISR increments */
static volatile uint32_t s_frames_ok = 0; /* Successfully parsed frame count (ISR writes) */
static volatile uint32_t s_crc_err   = 0; /* CRC verify fail count (ISR writes) */
static volatile uint32_t s_ovf      = 0;  /* Queue-full drop count (ISR writes) */
static uint32_t s_crc_err_logged = 0;     /* CRC err count already printed on task side */

#define ESP32S3_FLAG_RX  0x00000001U

/* ====================== CRC16/MODBUS ======================
 * Polynomial 0x8005 (reflected 0xA001), init 0xFFFF, input/output reflected, xorout=0x0000.
 * Calc range: CMD byte + payload bytes (all bytes covered by frame length L).
 * ESP32 firmware must use the exact same algorithm, otherwise CRC verification always fails. */
static uint16_t esp32s3_crc16(const uint8_t *d, uint16_t n)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < n; i++) {
        crc ^= (uint16_t)d[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001U) crc = (crc >> 1) ^ 0xA001U;
            else               crc =  crc >> 1;
        }
    }
    return crc;
}

/* ====================== Frame parse state machine ======================
 * Feed byte stream; complete frame with correct CRC -> enqueue (ISR-safe); bad frame -> count.
 * Design constraint: this function runs in USART6 ISR context, must NOT call LOG_I (print only in task context). */
typedef enum {
    ST_H1, ST_H2, ST_LEN, ST_CMD, ST_PAY, ST_CRC_LO, ST_CRC_HI
} fsm_state_t;

static fsm_state_t s_st       = ST_H1;
static uint8_t  s_len        = 0;     /* L = total bytes of CMD + payload */
static uint8_t  s_cmd        = 0;
static uint8_t  s_pay[ESP32S3_RX_BUF_SIZE];
static uint8_t  s_crcbuf[ESP32S3_RX_BUF_SIZE]; /* CRC calc region: CMD + payload */
static uint16_t s_crc_len    = 0;     /* crcbuf collected byte count */
static uint16_t s_pay_idx    = 0;     /* Payload write index */
static uint8_t  s_crc_recv_lo = 0;    /* CRC low byte temp (for ST_CRC_LO) */

static void esp32s3_emit(const uint8_t *pay, uint8_t pay_len)
{
    /* Only CMD=0x01 detect frame supported; other CMDs ignored for now (still counted in s_frames_ok) */
    if (s_cmd != ESP32S3_CMD_DETECT) return;
    if (pay_len < 1U) return;

    uint8_t n = pay[0];
    if (n > ESP32S3_MAX_OBJ) n = ESP32S3_MAX_OBJ;   /* Truncate if exceeded, avoid out-of-bounds */

    esp32s3_result_t res;
    res.count = n;
    res.frame_cnt = ++s_frame_cnt;
    for (uint8_t i = 0U; i < n; i++) {
        const uint8_t *p = &pay[1U + (uint16_t)i * 7U];
        res.obj[i].cls  = p[0];
        res.obj[i].cx   = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
        res.obj[i].cy   = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
        res.obj[i].conf = p[5];
    }

    if (osMessageQueuePut(g_ImgResultImg_qHandle, &res, 0U, 0U) != osOK) {
        s_ovf++;   /* Queue full: drop, count (non-blocking) */
    }
}

static void esp32s3_fsm_feed(uint8_t b)
{
    switch (s_st) {
    case ST_H1:
        if (b == ESP32S3_SYNC_H1) s_st = ST_H2;
        break;
    case ST_H2:
        if (b == ESP32S3_SYNC_H2) { s_st = ST_LEN; s_crc_len = 0U; s_pay_idx = 0U; }
        else if (b == ESP32S3_SYNC_H1) { s_st = ST_H2; }   /* AA AA 55... tolerance */
        else s_st = ST_H1;
        break;
    case ST_LEN:
        s_len = b;
        s_st = ST_CMD;
        break;
    case ST_CMD:
        s_cmd = b;
        s_crcbuf[0] = b; s_crc_len = 1U;   /* CRC region first byte = CMD */
        s_pay_idx = 0U;
        s_st = ST_PAY;
        break;
    case ST_PAY:
        if (s_crc_len < (uint16_t)sizeof(s_crcbuf)) s_crcbuf[s_crc_len] = b;
        if (s_pay_idx < (uint16_t)sizeof(s_pay))    s_pay[s_pay_idx]    = b;
        s_crc_len++; s_pay_idx++;
        if (s_crc_len >= s_len) s_st = ST_CRC_LO;  /* CMD+payload fully received */
        break;
    case ST_CRC_LO:
        s_crc_recv_lo = b;
        s_st = ST_CRC_HI;
        break;
    case ST_CRC_HI: {
        uint16_t crc_recv = (uint16_t)s_crc_recv_lo | ((uint16_t)b << 8);
        uint16_t crc_calc = esp32s3_crc16(s_crcbuf, s_crc_len);
        if (crc_calc == crc_recv) {
            s_frames_ok++;
            esp32s3_emit(s_pay, (uint8_t)(s_len - 1U));   /* payload = L-1 */
        } else {
            s_crc_err++;
        }
        s_st = ST_H1;
        break;
    }
    default:
        s_st = ST_H1;
        break;
    }
}

/* ====================== HAL callback (weak symbol override) ======================
 * USART6_IRQHandler is generated and calls HAL_UART_IRQHandler(&huart6),
 * IDLE/RX-complete lands in this callback. ISR only does: frame RX + enqueue + set flag + restart RX. */
void ESP32S3_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart6) return;

    for (uint16_t i = 0U; i < size; i++) {
        esp32s3_fsm_feed(s_rx_buf[i]);
    }
    /* Restart RX (after ReceiveToIdle_IT finishes, UART returns to READY and must be re-enabled) */
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, s_rx_buf, (uint16_t)sizeof(s_rx_buf));
    /* Wake parse task (ISR-safe API); Task_Esp32S3Handle is CubeMX-generated */
    (void)osThreadFlagsSet(Task_Esp32S3Handle, ESP32S3_FLAG_RX);
}

/* ====================== Parse task (sole queue consumer) ====================== */
void ESP32S3_Task_Run(void *argument)
{
    (void)argument;
    esp32s3_result_t res;

    /* Start RX only after kernel starts, avoid ISR touching OS primitives before scheduler is up */
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart6, s_rx_buf, (uint16_t)sizeof(s_rx_buf)) != HAL_OK) {
        LOG_E("ESP32", "RX start failed");
        return;
    }

    for (;;) {
        /* 用 1s 超时而非 osWaitForever：保证即便 ESP32-S3 暂时无帧，任务仍周期性唤醒并踢心跳，
         * 使看门狗只反映"任务是否还被调度"，而非"是否收到帧"（收帧率由别处健康检查，不耦合复位）。 */
        (void)osThreadFlagsWait(ESP32S3_FLAG_RX, osFlagsWaitAny, 1000U);
        task_heartbeat_kick(HB_ESP32S3);   /* 存活探针：每 1s(或收到帧)踢一次；任务真冻结(不再被调度)才超时触发 IWDG 复位 */
        /* Drain queue, write latest snapshot and emit debug log */
        while (osMessageQueueGet(g_ImgResultImg_qHandle, &res, NULL, 0U) == osOK) {
            s_latest = res;   /* Task-context write, single writer, safe for other tasks to read */
#if DBG_LOG_ESP32S3 == 1
            LOG_I("ESP32", "DET n=%u f=%lu cls0=%u cx=%u cy=%u conf=%u",
                  res.count, (unsigned long)res.frame_cnt,
                  res.count ? res.obj[0].cls : 0U,
                  res.count ? res.obj[0].cx  : 0U,
                  res.count ? res.obj[0].cy  : 0U,
                  res.count ? res.obj[0].conf: 0U);
#endif
        }
#if DBG_LOG_ESP32S3 == 1
        if (s_crc_err != s_crc_err_logged) {
            LOG_W("ESP32", "CRC err total=%lu", (unsigned long)s_crc_err);
            s_crc_err_logged = s_crc_err;
        }
#endif
    }
}

/* ====================== Public API ====================== */
int ESP32S3_BSP_Init(void)
{
    /* Force baud 921600 (design value, Pinout Sec.3.8). .ioc may generate 115200;
       this override lives in self-built module, not lost on CubeMX regen, but sync .ioc as source of truth is recommended. */
    huart6.Init.BaudRate = 921600U;
    if (HAL_UART_Init(&huart6) != HAL_OK) return -1;

    /* Task (Task_Esp32S3 / StartEsp32S3Task) and image queue (g_ImgResultImg_qHandle,
       Size=8/ItemSize=72) are created by CubeMX in MX_FREERTOS_Init; this module only
       uses the handles. NOTE: ESP32S3_BSP_Init runs BEFORE the queue is osMessageQueueNew'd
       (call order in MX_FREERTOS_Init), so do NOT NULL-check the handle here. */
    LOG_I("ESP32", "BSP init ok (USART6 921600, IDLE)");
    return 0;
}

int ESP32S3_GetLatest(esp32s3_result_t *out)
{
    if (out == NULL) return -1;
    *out = s_latest;
    return 0;
}

void ESP32S3_PrintStats(void)
{
    LOG_I("ESP32", "rx f_ok=%lu crc_err=%lu ovf=%lu last_n=%u",
          (unsigned long)s_frames_ok, (unsigned long)s_crc_err,
          (unsigned long)s_ovf, s_latest.count);
}
