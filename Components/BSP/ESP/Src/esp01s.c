/**
  * @file    esp01s.c
  * @brief   ESP-01S (ESP8266 AT) WiFi / Aliyun MQTT / OTA communication (STM32 H743 side, USART2)
  * @note    Based on bsp_esp8266.c logic from STM32_robot_version_2 (AT+DMA/IDLE dual queue),
  *          but this project's USART2 has no DMA configured (usart.c only has hdma_usart1_rx), so we use
  *          HAL_UARTEx_ReceiveToIdle_IT (interrupt-only, same decision as ESP32-S3).
  *          ISR rule (error.md E2): ESP01S_UART_RxCallback only collects bytes into the ring buffer
  *          + sets the task flag; never LOG_I / blocking send. All logs are printed in task context.
  *          Debug: LOG_I("NET", ...) over UART1, gated by DBG_LOG_NET in dbg_config.h.
  */
#include "main.h"          /* huart2 + HAL */
#include "esp01s.h"
#include "logger.h"        /* LOG_I / LOG_E / LOG_W */
#include "dbg_config.h"    /* DBG_LOG_NET switch */

#include <string.h>
#include <stdio.h>

/* ====================== Static state ====================== */
static uint8_t s_rx_tmp[ESP01S_RX_BUF_SIZE];   /* ReceiveToIdle_IT RX buffer */
static uint8_t s_rx_ring[ESP01S_RX_BUF_SIZE];  /* Ring buffer: ISR writes, task reads */
static volatile uint16_t s_rx_wi = 0;          /* Write index (incremented by ISR) */
static volatile uint16_t s_rx_ri = 0;          /* Read index (incremented by task) */
static uint16_t s_rx_size = 0;                 /* Byte count from last IDLE (set by ISR) */
static osThreadId_t s_net_task = NULL;
static uint8_t  s_transparent = 0;             /* 1=transparent mode (raw TCP) entered, 0=AT mode */

#define ESP01S_FLAG_RX  0x00000001U

/* ====================== Ring buffer ====================== */
static void ring_put(uint8_t b)
{
    s_rx_ring[s_rx_wi] = b;
    s_rx_wi = (uint16_t)((s_rx_wi + 1U) % ESP01S_RX_BUF_SIZE);
    if (s_rx_wi == s_rx_ri) {                  /* Full: drop oldest */
        s_rx_ri = (uint16_t)((s_rx_ri + 1U) % ESP01S_RX_BUF_SIZE);
    }
}
static uint16_t ring_avail(void)
{
    return (uint16_t)((s_rx_wi - s_rx_ri + ESP01S_RX_BUF_SIZE) % ESP01S_RX_BUF_SIZE);
}
/* Copy available bytes to dst without consuming (handles wrap), return count */
static uint16_t ring_peek(uint8_t *dst, uint16_t max)
{
    uint16_t n = ring_avail();
    if (n > max) n = max;
    for (uint16_t i = 0U; i < n; i++) {
        dst[i] = s_rx_ring[(s_rx_ri + i) % ESP01S_RX_BUF_SIZE];
    }
    return n;
}
/* Consume all available bytes (clear after each AT cmd to avoid response crosstalk) */
static void ring_drain(void)
{
    s_rx_ri = s_rx_wi;
}
/* Find fixed-length binary pattern in ring buffer */
static int ring_find_bin(const uint8_t *pat, uint8_t patlen)
{
    uint16_t n = ring_avail();
    if (n < patlen) return 0;
    for (uint16_t i = 0U; i <= (uint16_t)(n - patlen); i++) {
        int ok = 1;
        for (uint8_t j = 0U; j < patlen; j++) {
            if (s_rx_ring[(s_rx_ri + i + j) % ESP01S_RX_BUF_SIZE] != pat[j]) { ok = 0; break; }
        }
        if (ok) return 1;
    }
    return 0;
}

/* ====================== USART2 RX callback (forwarded by uart_rx_dispatcher.c) ====================== */
/* ISR context: only collect bytes into ring buffer + restart RX + set task flag; never LOG/block. */
void ESP01S_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    s_rx_size = size;
    for (uint16_t i = 0U; i < s_rx_size; i++) {
        ring_put(s_rx_tmp[i]);
    }
    /* Restart RX (after ReceiveToIdle_IT finishes, UART returns to READY and must be re-enabled) */
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rx_tmp, (uint16_t)sizeof(s_rx_tmp));
    if (s_net_task != NULL) {
        (void)osThreadFlagsSet(s_net_task, ESP01S_FLAG_RX);
    }
}

/* ====================== AT TX/RX ====================== */
/* Wait for token (AT-mode text), return 0 on timeout */
static int wait_token(const char *tok, uint32_t timeout_ms)
{
    uint32_t t0 = osKernelGetTickCount();
    uint16_t toklen = (uint16_t)strlen(tok);
    for (;;) {
        if (ring_avail() >= toklen) {
            uint8_t tmp[ESP01S_RX_BUF_SIZE];
            uint16_t n = ring_peek(tmp, (uint16_t)sizeof(tmp));
            tmp[n] = '\0';
            if (strstr((char *)tmp, tok) != NULL) return 1;
        }
        if ((uint32_t)(osKernelGetTickCount() - t0) >= timeout_ms) return 0;
        (void)osThreadFlagsWait(ESP01S_FLAG_RX, osFlagsWaitAny, 50U);
    }
}

int ESP01S_SendAT(const char *cmd, char *resp, uint32_t timeout_ms)
{
    if (cmd == NULL) return ESP01S_ERR_PARAM;
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    if (n < 0 || n >= (int)sizeof(buf)) return ESP01S_ERR_PARAM;

    ring_drain();   /* Clear history responses first to avoid false match */
    if (HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 1000U) != HAL_OK)
        return ESP01S_ERR_LINK;

    int rc = ESP01S_ERR_TIMEOUT;
    if (wait_token("OK", timeout_ms)) {
        rc = ESP01S_OK;
    } else if (ring_find_bin((const uint8_t *)"ERROR", 5) || wait_token("ERROR", 100U)) {
        rc = ESP01S_ERR_NORESP;
    }

    if (resp != NULL) {
        uint8_t tmp[ESP01S_RX_BUF_SIZE];
        uint16_t m = ring_peek(tmp, (uint16_t)(sizeof(tmp) - 1U));
        tmp[m] = '\0';
        strncpy(resp, (char *)tmp, 127);
        resp[127] = '\0';
    }
    ring_drain();
    return rc;
}

int ESP01S_SendRaw(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) return ESP01S_ERR_PARAM;
    if (HAL_UART_Transmit(&huart2, data, len, 1000U) != HAL_OK) return ESP01S_ERR_LINK;
    return ESP01S_OK;
}

/* ====================== Network / link setup ====================== */
int ESP01S_Init(void)
{
    s_net_task = osThreadGetId();
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rx_tmp, (uint16_t)sizeof(s_rx_tmp)) != HAL_OK) {
        LOG_E("NET", "USART2 RX start failed");
        return ESP01S_ERR_LINK;
    }
    osDelay(500U);
    ring_drain();

    if (ESP01S_SendAT("AT", NULL, 1000U) != ESP01S_OK) {
        LOG_E("NET", "ESP-01S no AT echo (check wiring/baud/power)");
        return ESP01S_ERR_NORESP;
    }
    ESP01S_SendAT("AT+CWMODE=1", NULL, 2000U);   /* STA mode */
    if (ESP01S_SendAT("ATE0", NULL, 1000U) != ESP01S_OK) { /* Disable echo to reduce noise */
        /* Non-fatal, continue */
    }
    char buf[96];
    snprintf(buf, sizeof(buf), "AT+CWJAP=\"%s\",\"%s\"", ESP01S_WIFI_SSID, ESP01S_WIFI_PWD);
    if (ESP01S_SendAT(buf, NULL, ESP01S_NET_TIMEOUT_MS) != ESP01S_OK) {
        LOG_E("NET", "WiFi join failed (SSID/password?)");
        return ESP01S_ERR_LINK;
    }
    LOG_I("NET", "WiFi connected");
    return ESP01S_OK;
}

int ESP01S_ConnectTCP(const char *host, uint16_t port)
{
    if (host == NULL) return ESP01S_ERR_PARAM;
    char buf[128];
    snprintf(buf, sizeof(buf), "AT+CIPSTART=\"TCP\",\"%s\",%u", host, port);
    if (ESP01S_SendAT(buf, NULL, ESP01S_NET_TIMEOUT_MS) != ESP01S_OK) {
        LOG_E("NET", "TCP connect failed");
        return ESP01S_ERR_LINK;
    }
    /* Transparent mode + enter send */
    if (ESP01S_SendAT("AT+CIPMODE=1", NULL, 2000U) != ESP01S_OK) {
        LOG_E("NET", "Set transparent mode failed");
        return ESP01S_ERR_LINK;
    }
    if (ESP01S_SendAT("AT+CIPSEND", NULL, 2000U) != ESP01S_OK) {
        LOG_E("NET", "Enter transparent mode failed");
        return ESP01S_ERR_LINK;
    }
    s_transparent = 1U;
    LOG_I("NET", "TCP transparent link established");
    return ESP01S_OK;
}

/* ====================== Minimal MQTT client (raw packets, QoS0) ====================== */
/* Remaining-length variable-length encode, return byte count */
static uint8_t mqtt_varint(uint32_t v, uint8_t *b)
{
    uint8_t i = 0U;
    do {
        uint8_t byte = (uint8_t)(v & 0x7FU);
        v >>= 7;
        if (v) byte |= 0x80U;
        b[i++] = byte;
    } while (v);
    return i;
}

/* Build CONNECT packet -> out, *outlen; return 0 on success */
static int mqtt_build_connect(uint8_t *out, uint16_t max, uint16_t *outlen)
{
    const char *cid = ESP01S_MQTT_CLIENTID;
    const char *usr = ESP01S_MQTT_USERNAME;
    const char *pwd = ESP01S_MQTT_PASSWORD;
    uint16_t cl = (uint16_t)strlen(cid);
    uint16_t ul = (uint16_t)strlen(usr);
    uint16_t pl = (uint16_t)strlen(pwd);

    uint32_t paylen = 2U + cl + 2U + ul + 2U + pl;   /* Three segments: 2-byte len + string */
    uint32_t rem    = 9U + paylen;                    /* Variable header 9 bytes + payload */
    if ((uint32_t)(9U + paylen) > 0xFFFFU) return -1;

    uint8_t hdr[16];
    uint8_t hl = 0U;
    hdr[hl++] = 0x10;                                /* CONNECT */
    hl += mqtt_varint(rem, &hdr[hl]);
    hdr[hl++] = 0x00; hdr[hl++] = 0x04;              /* Protocol name length 4 */
    hdr[hl++] = 'M'; hdr[hl++] = 'Q'; hdr[hl++] = 'T'; hdr[hl++] = 'T';
    hdr[hl++] = 0x04;                                /* Protocol level 4 */
    hdr[hl++] = 0xC2;                                /* clean=1, username=1, pw=1 */
    hdr[hl++] = 0x00; hdr[hl++] = 0x3C;              /* keepalive 60s */

    if ((uint16_t)(hl + paylen) > max) return -1;
    uint16_t o = 0U;
    memcpy(&out[o], hdr, hl); o += hl;
    out[o++] = (uint8_t)(cl >> 8); out[o++] = (uint8_t)(cl & 0xFF); memcpy(&out[o], cid, cl); o += cl;
    out[o++] = (uint8_t)(ul >> 8); out[o++] = (uint8_t)(ul & 0xFF); memcpy(&out[o], usr, ul); o += ul;
    out[o++] = (uint8_t)(pl >> 8); out[o++] = (uint8_t)(pl & 0xFF); memcpy(&out[o], pwd, pl); o += pl;
    *outlen = o;
    return 0;
}

int ESP01S_MQTT_Connect(void)
{
    if (!s_transparent) return ESP01S_ERR_LINK;      /* Call ESP01S_ConnectTCP first */
    uint8_t pkt[160];
    uint16_t len = 0U;
    if (mqtt_build_connect(pkt, (uint16_t)sizeof(pkt), &len) != 0) return ESP01S_ERR_PARAM;
    if (ESP01S_SendRaw(pkt, len) != ESP01S_OK) return ESP01S_ERR_LINK;

    /* Wait CONNACK: fixed header 0x20 0x02 */
    uint8_t connack[2] = { 0x20, 0x02 };
    uint32_t t0 = osKernelGetTickCount();
    for (;;) {
        if (ring_find_bin(connack, 2U)) { LOG_I("NET", "MQTT CONNACK OK"); return ESP01S_OK; }
        if ((uint32_t)(osKernelGetTickCount() - t0) >= ESP01S_NET_TIMEOUT_MS) {
            LOG_E("NET", "MQTT CONNACK timeout");
            return ESP01S_ERR_TIMEOUT;
        }
        (void)osThreadFlagsWait(ESP01S_FLAG_RX, osFlagsWaitAny, 50U);
    }
}

int ESP01S_MQTT_Pub(const char *topic, const char *json)
{
    if (topic == NULL || json == NULL) return ESP01S_ERR_PARAM;
    if (!s_transparent) return ESP01S_ERR_LINK;

    uint16_t tlen = (uint16_t)strlen(topic);
    uint16_t jlen = (uint16_t)strlen(json);
    uint32_t rem = 2U + tlen + jlen;                 /* Topic (2+len) + payload */
    if (rem > 0xFFFFU) return ESP01S_ERR_PARAM;

    uint8_t pkt[320];
    uint16_t o = 0U;
    pkt[o++] = 0x30;                                 /* PUBLISH QoS0, no dup/retain */
    o += mqtt_varint(rem, &pkt[o]);
    pkt[o++] = (uint8_t)(tlen >> 8); pkt[o++] = (uint8_t)(tlen & 0xFF);
    memcpy(&pkt[o], topic, tlen); o += tlen;
    memcpy(&pkt[o], json, jlen);  o += jlen;

    if (ESP01S_SendRaw(pkt, o) != ESP01S_OK) return ESP01S_ERR_LINK;
    return ESP01S_OK;
}
