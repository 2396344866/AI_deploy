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
#include "app_config.h"    /* APP_ENABLE_NETWORK：物模型属性段整段门控 */

/* 物模型属性路由要驱动 LED / 电机 / 姿态（见 esp01s.h 的耦合说明） */
#include "led.h"
#include "motor.h"
#include "attitude.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>        /* strtof：JSON 值解析 */
#include "iwdg.h"         /* log_wdt_feed：POST 协作喂狗 */

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
/* Consume exactly n bytes (partial read: keep the rest for the next packet) */
static void ring_skip(uint16_t n)
{
    uint16_t avail = ring_avail();
    if (n > avail) n = avail;
    s_rx_ri = (uint16_t)((s_rx_ri + n) % ESP01S_RX_BUF_SIZE);
}
/* Read the k-th byte from the current read index without consuming (0-based).
 * Lets us parse a packet in place instead of peeking it into a large stack buffer. */
static uint8_t ring_at(uint16_t k)
{
    return s_rx_ring[(s_rx_ri + k) % ESP01S_RX_BUF_SIZE];
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

/* Locate CONNACK (0x20 0x02) in ring; return its return-code byte (4th), 0xFF if absent.
 * 注意：rc 必须参与连接成败判定 —— 只看 "0x20 0x02" 前两字节会把
 * rc=4(用户名密码错)/rc=5(未授权) 的**拒绝包**也误判成连接成功。 */
static uint8_t esp01s_connack_rc(void)
{
    uint16_t n = ring_avail();
    for (uint16_t i = 0U; i + 3U < n; i++) {
        uint16_t a = (uint16_t)((s_rx_ri + i)      % ESP01S_RX_BUF_SIZE);
        uint16_t b = (uint16_t)((s_rx_ri + i + 1U) % ESP01S_RX_BUF_SIZE);
        uint16_t d = (uint16_t)((s_rx_ri + i + 3U) % ESP01S_RX_BUF_SIZE);
        if (s_rx_ring[a] == 0x20U && s_rx_ring[b] == 0x02U) {
            return s_rx_ring[d];
        }
    }
    return 0xFFU;
}

/* CONNACK return-code -> human readable. 阿里云连不上的绝大多数原因是 4 / 5（凭据问题）。 */
static const char *esp01s_connack_str(uint8_t rc)
{
    switch (rc) {
        case 0U:  return "accepted";
        case 1U:  return "unacceptable protocol version";
        case 2U:  return "identifier rejected (clientId/format)";
        case 3U:  return "server unavailable";
        case 4U:  return "bad username or password (check signmethod/timestamp match!)";
        case 5U:  return "not authorized (check device triple / region)";
        default:  return "unknown";
    }
}

/* 把一段二进制按 16B/行 打印 hex+ASCII（TRACE 级，DEBUG 下不打），用于把 CONNECT 报文、
 * 透传期收到的原始字节原样呈现，便于逐字节核对凭据边界与 broker 回包（CONNACK/CLOSED/ERROR）。 */
static void esp01s_hexdump(const char *tag, const uint8_t *p, uint16_t len)
{
#if DBG_LOG_NET
    if (p == NULL || len == 0U) return;
    for (uint16_t base = 0U; base < len; base += 16U) {
        char line[96];
        int o = snprintf(line, sizeof(line), "[%s] %04X:", tag, (unsigned)base);
        uint16_t m = (uint16_t)((len - base > 16U) ? 16U : (len - base));
        for (uint16_t i = 0U; i < m; i++)
            o += snprintf(&line[o], (size_t)(sizeof(line) - (size_t)o), " %02X", (unsigned)p[base + i]);
        for (uint16_t i = m; i < 16U; i++)
            o += snprintf(&line[o], (size_t)(sizeof(line) - (size_t)o), "   ");
        o += snprintf(&line[o], (size_t)(sizeof(line) - (size_t)o), "  ");
        for (uint16_t i = 0U; i < m; i++) {
            uint8_t c = p[base + i];
            o += snprintf(&line[o], (size_t)(sizeof(line) - (size_t)o), "%c",
                          (char)((c >= 0x20U && c < 0x7FU) ? c : '.'));
        }
        line[o] = '\0';
        LOG_T("NET", "%s", line);
    }
#else
    (void)tag; (void)p; (void)len;
#endif
}

/* ====================== USART2 RX callback (forwarded by uart_rx_dispatcher.c) ====================== */
/* ISR context: only collect bytes into ring buffer + restart RX + set task flag; never LOG/block. */
/* Re-arm USART2 RX. Used by RxCallback and by uart_rx_dispatcher error recovery.
 * Returns HAL status; ISR-safe. */
HAL_StatusTypeDef ESP01S_UART_RxStart(void)
{
    return HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rx_tmp, (uint16_t)sizeof(s_rx_tmp));
}

/* 线程上下文专用：先清掉残留的 BUSY_RX 再重新 arm。
 *
 * 为什么必须有这一步：ReceiveToIdle 一旦 arm，只有收到 IDLE 或满长度才会回到 READY。
 * 若上一条 AT 超时（ESP-01S 没回应），huart2.RxState 会一直停在 BUSY_RX，
 * 此后每次调 ReceiveToIdle_IT 都返回 HAL_BUSY —— 表现为每轮重连都打印
 * "USART2 RX start failed" 然后直接 return，连 AT 都发不出去，
 * 网络永久死亡，即使后来把模块接好也永远起不来（只能复位 MCU）。
 *
 * 注意：AbortReceive 是阻塞式，故本函数只能在任务上下文调用（禁止 ISR）。
 * ISR 路径仍用轻量的 ESP01S_UART_RxStart()。 */
static HAL_StatusTypeDef esp01s_rx_restart_clean(void)
{
    if (huart2.RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(&huart2);
    }
    return ESP01S_UART_RxStart();
}

void ESP01S_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    s_rx_size = size;
    for (uint16_t i = 0U; i < s_rx_size; i++) {
        ring_put(s_rx_tmp[i]);
    }
    /* Restart RX (after ReceiveToIdle_IT finishes, UART returns to READY and must be re-enabled) */
    (void)ESP01S_UART_RxStart();
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
#if DBG_LOG_NET
    LOG_D("NET", "AT> %s", cmd);
#endif
    ring_drain();   /* Clear history responses first to avoid false match */
    if (HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 1000U) != HAL_OK)
        return ESP01S_ERR_LINK;

    int rc = ESP01S_ERR_TIMEOUT;
    if (wait_token("OK", timeout_ms)) {
        rc = ESP01S_OK;
    } else if (ring_find_bin((const uint8_t *)"ERROR", 5) || wait_token("ERROR", 100U)) {
        rc = ESP01S_ERR_NORESP;
    }

    /* 无论是否要 resp，都把这次 AT 回包原样打印（DEBUG 级），
     * 让你逐条确认「哪条指令返回、返回了什么」（含回显/OK/ERROR/超时）。 */
    uint8_t rxb[128];
    uint16_t rm = ring_peek(rxb, (uint16_t)(sizeof(rxb) - 1U));
    rxb[rm] = '\0';
#if DBG_LOG_NET
    LOG_D("NET", "AT< [%s] %s", (rc == ESP01S_OK) ? "OK" : "FAIL", (char *)rxb);
#endif
    if (resp != NULL) {
        strncpy(resp, (char *)rxb, 127);
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
    /* 走 esp01s_rx_restart_clean()（先 Abort 上一轮残留的 BUSY_RX）。
       直接用 ReceiveToIdle_IT 会让「一次 AT 超时」变成永久故障：
       此后每轮重连都卡在 RX start failed 直接 return，连 AT 都发不出去。 */
    HAL_StatusTypeDef rxst = esp01s_rx_restart_clean();
    if (rxst != HAL_OK) {
        /* 打印 HAL 状态码与 RxState，便于区分 HAL_BUSY(=2, 上一轮没清干净) 与 HAL_ERROR(=1, 硬件未就绪) */
        LOG_E("NET", "USART2 RX start failed (st=%d RxState=%d)", (int)rxst, (int)huart2.RxState);
        return ESP01S_ERR_LINK;
    }

    /* 防御性诊断：CubeMX 若没勾选 "USART2 global interrupt"，NVIC 层就没开中断，
     * 此时 ReceiveToIdle_IT 仍返回 HAL_OK（看起来成功），但 ISR 永不触发、永远收不到字节。
     * 这是最容易误判成"接线/波特率/模块坏了"的坑 —— 实测踩过，见 Error/net_error.md E32。 */
    if ((NVIC->ISER[(uint32_t)(USART2_IRQn) >> 5UL] &
         (1UL << ((uint32_t)(USART2_IRQn) & 0x1FUL))) == 0UL) {
        LOG_E("NET", "USART2 NVIC NOT enabled -> RX ISR never fires (fix: .ioc enable USART2 global interrupt)");
    }

    /* 先探一次 AT（300ms）：模块若在 AT 模式会立刻回 OK，零额外开销。
     * 没回应 → 极可能仍停在**透传模式**（上次连通过 CIPSEND，而重连/复位时模块并没断电）。
     * 透传下模块把 "AT\r\n" 当 TCP payload 转发、**不解析也不回 OK**，
     * 症状是握手恒超时 + rx 0 bytes，与"接线断/没供电"一模一样，极易误判回硬件排查。
     *
     * ⚠️ 不能只在"上电首次 Init"做：CONNACK 超时等失败路径会带着透传状态回到这里，
     * 每轮重连都可能遇上 —— 所以改成"每次先探测、静默才退出"。
     *
     * 退出序列：单独一帧 "+++"（不带 CRLF），且前后各需 >1s 线路静默，三条缺一不可。 */
    if (ESP01S_SendAT("AT", NULL, 300U) != ESP01S_OK) {
        static const uint8_t exit_tp[3] = { '+', '+', '+' };
        osDelay(1100U);                                        /* "+++" 前的静默间隔 */
        (void)HAL_UART_Transmit(&huart2, exit_tp, 3U, 100U);
        osDelay(1200U);                                        /* 等模块处理完并回到 AT 模式 */
        /* 清理可能残留的 TCP 连接：透传退出后链路往往还活着，
           不清掉的话后面 AT+CIPSTART 会报 ALREADY CONNECTED。失败可忽略（无连接时会超时） */
        (void)ESP01S_SendAT("AT+CIPCLOSE", NULL, 500U);
#if DBG_LOG_NET
        LOG_D("NET", "AT probe silent -> sent transparent-exit (+++) + CIPCLOSE");
#endif
    }

    osDelay(500U);
    ring_drain();   /* 丢掉上轮残留/上电噪声，避免被误判成 AT 回响 */

#if DBG_LOG_NET
    LOG_D("NET", "AT handshake");
#endif
    if (ESP01S_SendAT("AT", NULL, 1000U) != ESP01S_OK) {
        /* 字节级诊断：区分「模块完全沉默」与「回了但乱码」——两者排查方向完全不同。
         *   0 字节   -> TX/RX 断线、模块没供电、EN/GPIO0 电平不对（模块根本没跑 AT 固件）
         *   有字节但非 "OK" -> 波特率不匹配（乱码）或帧格式不对
         * 少了这一步就只能盲猜硬件，实测这里能一次定位到根因。 */
        uint16_t n = ring_avail();
        if (n == 0U) {
            LOG_E("NET", "no AT echo: rx 0 bytes -> module SILENT (wiring/power/EN/GPIO0)");
        } else {
            char hex[64];
            int  o  = 0;
            uint16_t m = (n > 16U) ? 16U : n;
            for (uint16_t i = 0U; i < m; i++) {
                int w = snprintf(&hex[o], (size_t)((int)sizeof(hex) - o), "%02X ", (unsigned)ring_at(i));
                if (w <= 0) break;
                o += w;
                if (o >= ((int)sizeof(hex) - 4)) break;
            }
            hex[o] = '\0';
            LOG_E("NET", "no AT echo: rx %u bytes [%s]-> garbage=baud mismatch", (unsigned)n, hex);
        }
        return ESP01S_ERR_NORESP;
    }
#if DBG_LOG_NET
    LOG_D("NET", "CWMODE=1");
#endif
    ESP01S_SendAT("AT+CWMODE=1", NULL, 2000U);   /* STA mode */
#if DBG_LOG_NET
    LOG_D("NET", "ATE0");
#endif
    if (ESP01S_SendAT("ATE0", NULL, 1000U) != ESP01S_OK) { /* Disable echo to reduce noise */
        /* Non-fatal, continue */
    }
    char buf[96];
    char resp[128];
    snprintf(buf, sizeof(buf), "AT+CWJAP=\"%s\",\"%s\"", ESP01S_WIFI_SSID, ESP01S_WIFI_PWD);
#if DBG_LOG_NET
    LOG_D("NET", "CWJAP ssid=%s", ESP01S_WIFI_SSID);
#endif
    if (ESP01S_SendAT(buf, resp, ESP01S_NET_TIMEOUT_MS) != ESP01S_OK) {
        LOG_E("NET", "WiFi join failed (SSID/password?)");
#if DBG_LOG_NET
        LOG_D("NET", "CWJAP resp='%s'", resp);
#endif
        return ESP01S_ERR_LINK;
    }
    LOG_I("NET", "WiFi connected");
    return ESP01S_OK;
}

int ESP01S_ConnectTCP(const char *host, uint16_t port)
{
    if (host == NULL) return ESP01S_ERR_PARAM;
    char buf[128];
    char resp[128];
    snprintf(buf, sizeof(buf), "AT+CIPSTART=\"TCP\",\"%s\",%u", host, port);
#if DBG_LOG_NET
    LOG_D("NET", "CIPSTART %s:%u", host, (unsigned)port);
#endif
    if (ESP01S_SendAT(buf, resp, ESP01S_NET_TIMEOUT_MS) != ESP01S_OK) {
        LOG_E("NET", "TCP connect failed");
#if DBG_LOG_NET
        LOG_D("NET", "CIPSTART resp='%s'", resp);
#endif
        return ESP01S_ERR_LINK;
    }
    /* Transparent mode + enter send */
#if DBG_LOG_NET
    LOG_D("NET", "CIPMODE=1");
#endif
    if (ESP01S_SendAT("AT+CIPMODE=1", resp, 2000U) != ESP01S_OK) {
        LOG_E("NET", "Set transparent mode failed");
#if DBG_LOG_NET
        LOG_D("NET", "CIPMODE resp='%s'", resp);
#endif
        return ESP01S_ERR_LINK;
    }
#if DBG_LOG_NET
    LOG_D("NET", "CIPSEND");
#endif
    if (ESP01S_SendAT("AT+CIPSEND", resp, 2000U) != ESP01S_OK) {
        LOG_E("NET", "Enter transparent mode failed");
#if DBG_LOG_NET
        LOG_D("NET", "CIPSEND resp='%s'", resp);
#endif
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

    uint32_t paylen = 2U + cl + 2U + ul + 2U + pl;   /* 负载三段：2字节长度前缀 + 字符串 */
    /* 剩余长度 = 可变头(10B) + 负载(paylen)。
     * 可变头 = 协议名(2B长度前缀+"MQTT"=6B) + 协议级别(1B) + 连接标志(1B) + 保活(2B) = 10B。
     * ⚠️ 旧代码写成 9U+payload → 剩余长度少 1 → broker 解析到缺最后 1 字节(密码末字符)→ 静默断链、无 CONNACK（见 E37）。 */
    uint32_t rem    = 10U + paylen;
    if ((uint32_t)(10U + paylen) > 0xFFFFU) return -1;

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
    /* 阿里云的 clientId 很长（含 securemode/signmethod/timestamp），password 是 96 字符 hex，
     * 实测 CONNECT 报文约 217 B —— 超过旧的 160 B 缓冲，build 直接返回失败。
     * 用 static 而非栈上：Task_Network 栈仅 2048 B，这里再压 256 B 有溢出风险。 */
    static uint8_t pkt[256];
    uint16_t len = 0U;
    if (mqtt_build_connect(pkt, (uint16_t)sizeof(pkt), &len) != 0) {
        uint16_t cl = (uint16_t)strlen(ESP01S_MQTT_CLIENTID);
        uint16_t ul = (uint16_t)strlen(ESP01S_MQTT_USERNAME);
        uint16_t pl = (uint16_t)strlen(ESP01S_MQTT_PASSWORD);
        LOG_E("NET", "MQTT CONNECT too big: cid=%u usr=%u pwd=%u -> need~%u B > buf=%u",
              (unsigned)cl, (unsigned)ul, (unsigned)pl,
              (unsigned)(13U + 6U + cl + ul + pl), (unsigned)sizeof(pkt));
        return ESP01S_ERR_PARAM;
    }
    LOG_D("NET", "MQTT CONNECT send (wait CONNACK)");   /* DEBUG 里程碑：已发 CONNECT，进入等待 */
#if DBG_LOG_NET
    /* 以下为报文细节，仅 TRACE 级打印（DEBUG 下不打，避免刷屏） */
    LOG_T("NET", "CONNECT build: cid=%u usr=%u pwd=%u total=%u B (buf=%u)",
          (unsigned)strlen(ESP01S_MQTT_CLIENTID), (unsigned)strlen(ESP01S_MQTT_USERNAME),
          (unsigned)strlen(ESP01S_MQTT_PASSWORD), (unsigned)len, (unsigned)sizeof(pkt));
    esp01s_hexdump("CONN", pkt, len);   /* TRACE 级：逐字节核对凭据边界/剩余长度字段 */
#endif
    if (ESP01S_SendRaw(pkt, len) != ESP01S_OK) {
#if DBG_LOG_NET
        LOG_D("NET", "MQTT sendraw failed");
#endif
        return ESP01S_ERR_LINK;
    }

    /* Wait CONNACK: fixed header 0x20 0x02, return-code at byte[3].
     * 关键：rc 必须判 0，否则 rc=4(用户名密码错) / rc=5(未授权) 也会被当成连上，
     * 之后发布全部静默失败，极难定位。 */
    uint8_t connack[2] = { 0x20, 0x02 };
    uint32_t t0 = osKernelGetTickCount();
    uint16_t last_avail = 0U;          /* 只打印"新到"的字节，避免每轮重复打整段 ring */
    for (;;) {
        if (ring_find_bin(connack, 2U)) {
            uint8_t rc = esp01s_connack_rc();
            if (rc != 0U) {
                LOG_E("NET", "CONNACK rejected: rc=%u %s", (unsigned)rc, esp01s_connack_str(rc));
                return ESP01S_ERR_NORESP;
            }
            LOG_I("NET", "MQTT CONNACK OK (rc=0)");
            return ESP01S_OK;
        }
        /* 透传下 broker 的回包（CONNACK / CLOSED / ERROR）经 ESP-01S 原样转发回串口。
         * 这里把"新到达"的字节实时 hex 打印，让你直接看到 broker 到底回了什么（或什么都没回）。 */
        uint16_t na = ring_avail();
        if (na > last_avail) {
            uint16_t newn = (uint16_t)(na - last_avail);
            uint8_t tmp[48];
            uint16_t c = (newn > 48U) ? 48U : newn;
            for (uint16_t i = 0U; i < c; i++) tmp[i] = ring_at((uint16_t)(last_avail + i));
#if DBG_LOG_NET
            /* 新到字节的实时嗅探：仅 TRACE 级（DEBUG 下不打，避免每轮刷屏） */
            LOG_T("NET", "RX +%u B (total %u):", (unsigned)newn, (unsigned)na);
            esp01s_hexdump("RX", tmp, c);
#endif
            last_avail = na;
        }
        if ((uint32_t)(osKernelGetTickCount() - t0) >= ESP01S_NET_TIMEOUT_MS) {
            /* 超时时把 ring 里收到的东西打出来：区分"broker 完全没回"与"回了但格式不对"
             * （透传下 TCP 被 broker 断开通常会看到 CLOSED 字样）。 */
            uint16_t n = ring_avail();
            if (n == 0U) {
                LOG_E("NET", "CONNACK timeout: rx 0 bytes (broker silent / CONNECT not delivered?)");
            } else {
                char asc[64];
                uint16_t m = (n > 56U) ? 56U : n;
                for (uint16_t i = 0U; i < m; i++) {
                    uint8_t c = ring_at(i);
                    asc[i] = (char)((c >= 0x20U && c < 0x7FU) ? c : '.');
                }
                asc[m] = '\0';
                LOG_E("NET", "CONNACK timeout: rx %u bytes ASCII:'%s'", (unsigned)n, asc);
#if DBG_LOG_NET
                uint8_t tmp2[128];
                uint16_t c2 = (n > 128U) ? 128U : n;
                for (uint16_t i = 0U; i < c2; i++) tmp2[i] = ring_at(i);
                esp01s_hexdump("RX", tmp2, c2);
#endif
            }
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

    /* 整包 = 1(type) + varint(1~4 B) + rem，按上界 4 B 估。
     * 旧代码 pkt[320] 且**不做任何长度检查** —— 物模型属性上报 JSON 可达 300+ B，
     * 加上 52 B 的 topic 会越过 320 B 直接越界写栈（HardFault / 栈破坏），属 P0 隐患。
     * 这里放大缓冲 + 显式校验：装不下就丢弃并报错，绝不越界写。
     * 用 static 而非栈上：Task_Network 栈仅 2048 B。 */
    static uint8_t pkt[512];
    uint32_t need = 1U + 4U + rem;
    if (need > (uint32_t)sizeof(pkt)) {
        LOG_E("NET", "MQTT pub too big: rem=%u need=%u > buf=%u (dropped)",
              (unsigned)rem, (unsigned)need, (unsigned)sizeof(pkt));
        return ESP01S_ERR_PARAM;
    }

    uint16_t o = 0U;
    pkt[o++] = 0x30;                                 /* PUBLISH QoS0, no dup/retain */
    o += mqtt_varint(rem, &pkt[o]);
    pkt[o++] = (uint8_t)(tlen >> 8); pkt[o++] = (uint8_t)(tlen & 0xFF);
    memcpy(&pkt[o], topic, tlen); o += tlen;
    memcpy(&pkt[o], json, jlen);  o += jlen;

    if (ESP01S_SendRaw(pkt, o) != ESP01S_OK) return ESP01S_ERR_LINK;
    return ESP01S_OK;
}

/* ====================== Downlink: SUBSCRIBE + PUBLISH parsing ====================== */
int ESP01S_MQTT_Ping(void)
{
    static const uint8_t ping[2] = { 0xC0, 0x00 };   /* PINGREQ */
    if (!s_transparent) return ESP01S_ERR_LINK;
    return ESP01S_SendRaw(ping, 2U);
}

/* Build SUBSCRIBE packet (QoS0, one topic) -> out, *outlen; return 0 on success */
static int mqtt_build_subscribe(uint8_t *out, uint16_t max, uint16_t *outlen,
                                const char *topic, uint16_t pktid)
{
    uint16_t tlen = (uint16_t)strlen(topic);
    uint32_t rem  = 2U /*packet id*/ + 2U + (uint32_t)tlen + 1U /*qos*/;

    uint8_t hdr[8];
    uint8_t hl = 0U;
    hdr[hl++] = 0x82;                                   /* SUBSCRIBE */
    hl += mqtt_varint(rem, &hdr[hl]);
    if ((uint32_t)(hl + rem) > (uint32_t)max) return -1;

    uint16_t o = 0U;
    memcpy(&out[o], hdr, hl); o += hl;
    out[o++] = (uint8_t)(pktid >> 8); out[o++] = (uint8_t)(pktid & 0xFFU);
    out[o++] = (uint8_t)(tlen >> 8);  out[o++] = (uint8_t)(tlen & 0xFFU);
    memcpy(&out[o], topic, tlen); o += tlen;
    out[o++] = 0x00U;                                   /* QoS0 */
    *outlen = o;
    return 0;
}

/* Scan the ring for SUBACK (0x90 0x03 .. rc) and consume everything up to it.
 * Returns 1=found (*rc set), 0=not yet (some bytes may have been dropped to keep scanning).
 * Uses a small window so it never needs a 512-byte stack buffer. */
static int ring_take_suback(uint8_t *rc)
{
    uint16_t avail = ring_avail();
    if (avail < 5U) return 0;

    uint8_t  buf[64];
    uint16_t n = ring_peek(buf, (uint16_t)sizeof(buf));
    if (n < 5U) return 0;

    uint16_t limit = (uint16_t)(n - 4U);                /* last valid start index for a 5-byte match */
    for (uint16_t i = 0U; i < limit; i++) {
        if (buf[i] == 0x90U && buf[i + 1U] == 0x03U) {
            *rc = buf[i + 4U];
            ring_skip((uint16_t)(i + 5U));
            return 1;
        }
    }
    /* Not in this window: drop all but the trailing 4 bytes (they may start a match) */
    ring_skip((uint16_t)(n - 4U));
    return 0;
}

int ESP01S_MQTT_Sub(const char *topic)
{
    if (topic == NULL) return ESP01S_ERR_PARAM;
    if (!s_transparent) return ESP01S_ERR_LINK;

    uint8_t  pkt[192];
    uint16_t len = 0U;
    if (mqtt_build_subscribe(pkt, (uint16_t)sizeof(pkt), &len, topic, 1U) != 0) {
#if DBG_LOG_NET
        LOG_D("NET", "MQTT build SUBSCRIBE failed (topic too long?)");
#endif
        return ESP01S_ERR_PARAM;
    }
    ring_drain();                                       /* Drop pre-SUBACK noise (e.g. stale PUBLISH) */
#if DBG_LOG_NET
    LOG_D("NET", "MQTT SUBSCRIBE send: %s", topic);
#endif
    if (ESP01S_SendRaw(pkt, len) != ESP01S_OK) return ESP01S_ERR_LINK;

    uint32_t t0 = osKernelGetTickCount();
    for (;;) {
        uint8_t rc = 0xFFU;
        if (ring_take_suback(&rc)) {
            if (rc <= 2U) {                             /* 0/1/2 = granted QoS */
#if DBG_LOG_NET
                LOG_D("NET", "SUBACK rc=%u", (unsigned)rc);
#endif
                LOG_I("NET", "subscribed: %s", topic);
                return ESP01S_OK;
            }
            LOG_E("NET", "SUBACK rejected (rc=%u)", (unsigned)rc);
            return ESP01S_ERR_NORESP;
        }
        if ((uint32_t)(osKernelGetTickCount() - t0) >= ESP01S_NET_TIMEOUT_MS) {
            LOG_E("NET", "SUBACK timeout");
            return ESP01S_ERR_TIMEOUT;
        }
        (void)osThreadFlagsWait(ESP01S_FLAG_RX, osFlagsWaitAny, 50U);
    }
}

int ESP01S_MQTT_PollPublish(char *topic, uint16_t topic_max,
                            char *payload, uint16_t payload_max,
                            uint16_t *topic_len, uint16_t *payload_len)
{
    if (topic == NULL || payload == NULL || topic_max == 0U || payload_max == 0U) {
        return ESP01S_ERR_PARAM;
    }

    uint16_t avail = ring_avail();
    if (avail < 2U) return ESP01S_ERR_NORESP;           /* Nothing decodable yet */

    /* 1) Fixed header + varint (at most 5 bytes), read in place */
    uint8_t hdr[5];
    uint8_t hn = (avail < 5U) ? (uint8_t)avail : 5U;
    for (uint8_t i = 0U; i < hn; i++) hdr[i] = ring_at(i);

    uint32_t rem  = 0U;
    uint32_t mult = 1U;
    uint8_t  nb   = 0U;
    uint8_t  byte = 0U;
    do {
        if (nb >= 4U) { ring_skip(1U); return ESP01S_ERR_NORESP; }   /* Malformed varint: resync */
        if ((uint16_t)(1U + nb) >= avail) return ESP01S_ERR_NORESP;  /* Need more bytes */
        byte = hdr[1U + nb];
        rem += (uint32_t)(byte & 0x7FU) * mult;
        mult *= 128U;
        nb++;
    } while (byte & 0x80U);

    uint32_t total = 1U + (uint32_t)nb + rem;

    /* Ring 装不下的包永远等不到完整帧，必须整段丢弃，否则 ring 会永久卡在这个头部上。
       （ring 是 drop-oldest 环形，新字节会覆盖旧字节，不加此保护会一直返回 NORESP。） */
    if (total > (uint32_t)ESP01S_RX_BUF_SIZE) {
        ring_drain();
        LOG_W("NET", "downlink packet %u B > ring %u B, flushed",
              (unsigned)total, (unsigned)ESP01S_RX_BUF_SIZE);
        return ESP01S_ERR_NORESP;
    }

    /* 2) Non-PUBLISH packet: consume it so the ring never stalls on an unknown header.
     *    PINGRESP is always exactly 2 bytes and arrives periodically on a live link. */
    if (hdr[0] != 0x30U) {                              /* Only QoS0 PUBLISH (0x30) is handled */
        if (avail >= total)       ring_skip((uint16_t)total);
        else if (hdr[0] == 0xD0U) ring_skip(2U);        /* PINGRESP */
        return ESP01S_ERR_NORESP;
    }

    if (avail < total) return ESP01S_ERR_NORESP;        /* Incomplete PUBLISH: wait for the rest */

    /* 3) Topic: 2-byte length + bytes */
    uint16_t idx  = (uint16_t)(1U + nb);
    uint16_t tlen = (uint16_t)(((uint16_t)ring_at(idx) << 8) | (uint16_t)ring_at(idx + 1U));
    idx += 2U;
    if (rem < (2U + (uint32_t)tlen)) {                  /* Malformed: remaining length too small */
        ring_skip((uint16_t)total);
        return ESP01S_ERR_NORESP;
    }
    if ((uint32_t)tlen >= (uint32_t)topic_max) {
        ring_skip((uint16_t)total);
        LOG_W("NET", "downlink topic too long (%u >= %u), dropped", (unsigned)tlen, (unsigned)topic_max);
        return ESP01S_ERR_NORESP;
    }
    for (uint16_t i = 0U; i < tlen; i++) topic[i] = (char)ring_at((uint16_t)(idx + i));
    topic[tlen] = '\0';
    idx += tlen;

    /* 4) Payload = remaining length - 2 - topic length */
    uint32_t plen = rem - 2U - (uint32_t)tlen;
    if (plen >= (uint32_t)payload_max) {
        ring_skip((uint16_t)total);
        LOG_W("NET", "downlink payload too long (%u >= %u), dropped", (unsigned)plen, (unsigned)payload_max);
        return ESP01S_ERR_NORESP;
    }
    for (uint32_t i = 0U; i < plen; i++) payload[i] = (char)ring_at((uint16_t)(idx + i));
    payload[plen] = '\0';

    ring_skip((uint16_t)total);                         /* Consume the whole PUBLISH */

    if (topic_len)   *topic_len   = tlen;
    if (payload_len) *payload_len = (uint16_t)plen;
    return ESP01S_OK;
}

/* ====================== 阿里云物模型「属性」路由 ======================
 * 映射表与方向约定见 esp01s.h 对应段落；网络未使能时整段不编译（零体积）。
 * 全部静态名加 prop_ 前缀，避免与上方链路层状态重名。
 * ==================================================================== */
#if APP_ENABLE_NETWORK

/* ---------- 属性本地状态 ---------- */
static uint8_t s_prop_led     = 0;      /* LED_1（led.h 只有 setter，这里镜像一份） */
static uint8_t s_prop_move    = 0;      /* 0=停, 1=前进, 2=后退, 3=原地左转, 4=原地右转 */
static uint8_t s_prop_stopped = 1;      /* 1=停机/刹车（上电默认安全态） */
static float   s_prop_temp_c  = 0.0f;   /* MPU6050 芯片温度(℃)，由 Task_Sensor 喂入 */

/* ---------- 电机动作 ---------- */
/* 方向依据实测「中心对称」（motor_debug.md）：A50=左轮前进，B50=右轮后退。
 *   左轮前进 = A(+S)   左轮后退 = A(-S)
 *   右轮前进 = B(-S)   右轮后退 = B(+S)
 * 故：前进 A+S/B-S，后退 A-S/B+S，原地左转(左退右进) A-S/B-S，原地右转(左进右退) A+S/B+S。 */
static void prop_motion_apply(void)
{
    const int32_t S = (int32_t)ESP01S_ALIIOT_MOTION_SPEED;
    switch (s_prop_move) {
        case 1:  Motor_SetSpeed(MOTOR_A,  S); Motor_SetSpeed(MOTOR_B, -S); break;
        case 2:  Motor_SetSpeed(MOTOR_A, -S); Motor_SetSpeed(MOTOR_B,  S); break;
        case 3:  Motor_SetSpeed(MOTOR_A, -S); Motor_SetSpeed(MOTOR_B, -S); break;
        case 4:  Motor_SetSpeed(MOTOR_A,  S); Motor_SetSpeed(MOTOR_B,  S); break;
        default: Motor_SetSpeed(MOTOR_A, 0);  Motor_SetSpeed(MOTOR_B, 0);  break;
    }
}

/* 置某个运动态；v=1 生效（并自动解除停机），v=0 归零 */
static void prop_motion_set(uint8_t state, uint8_t v)
{
    /* 未平衡（本地自治未使能平衡）时拒绝运动指令，防摔倒 */
    if (!Attitude_GetEnable()) {
        LOG_W("NET", "motion rejected: balance NOT enabled");
        return;
    }
    s_prop_move = v ? state : 0U;
    if (v && s_prop_stopped) {        /* 下发运动指令视为解除刹车，无需先发 move_stop=0 */
        s_prop_stopped = 0U;
        Motor_Resume();
    }
    if (s_prop_stopped) return;       /* 处于停机态时不偷偷转轮 */
    prop_motion_apply();
}

/* ---------- 各属性 setter / getter ---------- */
static void     prop_set_led(uint8_t v)        { s_prop_led = v ? 1U : 0U; LED_R(s_prop_led); }
static uint8_t  prop_get_led(void)             { return s_prop_led; }

/* 云端 balance_enable 表达的是“意图”，交由 FSM 仲裁，不直接使能 */
static void prop_set_balance_enable(uint8_t v)
{
    if (v) { Attitude_SetCloudStand(1U); Attitude_SetCloudSit(0U); }
    else   { Attitude_SetCloudSit(1U);   Attitude_SetCloudStand(0U); }
}
static uint8_t prop_get_balance_enable(void)
{
    return Attitude_GetEnable() ? 1U : 0U;   /* 反馈当前实际平衡使能态 */
}

static float    prop_get_pitch(void)           { return Attitude_GetPitch(); }
static float    prop_get_roll(void)            { return Attitude_GetRoll(); }
static float    prop_get_yaw(void)             { return Attitude_GetYaw(); }

static void     prop_set_move_stop(uint8_t v)
{
    s_prop_stopped = v ? 1U : 0U;
    if (s_prop_stopped) {
        s_prop_move = 0U;
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        Motor_EmergencyStop();        /* 刹车并置 running=0 */
    } else {
        Motor_Resume();
        prop_motion_apply();
    }
}
static uint8_t  prop_get_move_stop(void)       { return s_prop_stopped; }

static void     prop_set_move_on(uint8_t v)    { prop_motion_set(1U, v); }
static uint8_t  prop_get_move_on(void)         { return (s_prop_move == 1U) ? 1U : 0U; }

static void     prop_set_move_back(uint8_t v)  { prop_motion_set(2U, v); }
static uint8_t  prop_get_move_back(void)       { return (s_prop_move == 2U) ? 1U : 0U; }

static void     prop_set_move_left(uint8_t v)  { prop_motion_set(3U, v); }
static uint8_t  prop_get_move_left(void)       { return (s_prop_move == 3U) ? 1U : 0U; }

static void     prop_set_move_right(uint8_t v) { prop_motion_set(4U, v); }
static uint8_t  prop_get_move_right(void)      { return (s_prop_move == 4U) ? 1U : 0U; }

static float    prop_get_temp(void)            { return s_prop_temp_c; }

/* ---------- 属性表（顺序必须与 esp01s_prop_t 一致） ---------- */
typedef struct {
    const char *ident;
    uint8_t     writable;      /* 1=云端可写（有 setter） */
    uint8_t     is_float;      /* 1=float 只读；0=bool */
    void      (*set_bool)(uint8_t v);
    uint8_t   (*get_bool)(void);
    float     (*get_float)(void);
} esp01s_prop_desc_t;

static const esp01s_prop_desc_t s_prop_desc[ESP01S_PROP_COUNT] = {
    { "LED_1",             1, 0, prop_set_led,        prop_get_led,        NULL            },  /* ESP01S_PROP_LED */
    { "balance_enable",    1, 0, prop_set_balance_enable, prop_get_balance_enable, NULL   },  /* ESP01S_PROP_EULER_OPEN */
    { "Euler_angle_Pitch", 0, 1, NULL,                NULL,                prop_get_pitch },  /* ESP01S_PROP_PITCH */
    { "Euler_angle_Roll",  0, 1, NULL,                NULL,                prop_get_roll  },  /* ESP01S_PROP_ROLL */
    { "Euler_angle_Yaw",   0, 1, NULL,                NULL,                prop_get_yaw   },  /* ESP01S_PROP_YAW */
    { "move_stop",         1, 0, prop_set_move_stop,  prop_get_move_stop,  NULL            },  /* ESP01S_PROP_MOVE_STOP */
    { "move_on",           1, 0, prop_set_move_on,    prop_get_move_on,    NULL            },  /* ESP01S_PROP_MOVE_ON */
    { "move_back",         1, 0, prop_set_move_back,  prop_get_move_back,  NULL            },  /* ESP01S_PROP_MOVE_BACK */
    { "move_left_rotate",  1, 0, prop_set_move_left,  prop_get_move_left,  NULL            },  /* ESP01S_PROP_MOVE_LEFT */
    { "move_right_rotate", 1, 0, prop_set_move_right, prop_get_move_right, NULL            },  /* ESP01S_PROP_MOVE_RIGHT */
    { "temp",              0, 1, NULL,                NULL,                prop_get_temp  },  /* ESP01S_PROP_TEMP */
};

/* ---------- 极简 JSON 取值（不引 cJSON） ---------- */
/* 在 s[0..slen) 中找 "key" : value，成功返回 value 起始指针并置 *vlen，失败返回 NULL。
   只做「顶层 key 名精确匹配」，不建树——阿里云 set 帧的 params 是扁平键值对，够用。
   局限：key 名必须在整个 payload 中唯一（本表 11 个标识符均满足）。 */
static const char *prop_json_get(const char *s, int slen, const char *key, int *vlen)
{
    int klen = (int)strlen(key);
    for (int i = 0; (i + klen + 2) < slen; i++) {
        if (s[i] != '"') continue;
        if (strncmp(&s[i + 1], key, (size_t)klen) != 0) continue;
        if (s[i + 1 + klen] != '"') continue;

        int j = i + 1 + klen + 1;                       /* 跳过 key 后的引号 */
        while (j < slen && (s[j] == ' ' || s[j] == '\t' || s[j] == '\n' || s[j] == '\r')) j++;
        if (j >= slen || s[j] != ':') continue;
        j++;
        while (j < slen && (s[j] == ' ' || s[j] == '\t' || s[j] == '\n' || s[j] == '\r')) j++;
        if (j >= slen) return NULL;

        int e = j;                                      /* 值终止符：, } ] 或换行 */
        while (e < slen && s[e] != ',' && s[e] != '}' && s[e] != ']' && s[e] != '\n' && s[e] != '\r') e++;
        while (e > j && (s[e - 1] == ' ' || s[e - 1] == '\t')) e--;
        *vlen = e - j;
        return &s[j];
    }
    return NULL;
}

/* 拷贝值到定长缓冲并 NUL 结尾（防 strtof 越界） */
static void prop_json_copy_val(const char *v, int vlen, char *dst, int dst_max)
{
    int n = (vlen < (dst_max - 1)) ? vlen : (dst_max - 1);
    if (n < 0) n = 0;
    memcpy(dst, v, (size_t)n);
    dst[n] = '\0';
}

/* 值 → bool：兼容 0/1 与 true/false（阿里云物模型 bool 下发多为 0/1 数字） */
static int prop_json_parse_bool(const char *v, int vlen, uint8_t *out)
{
    char t[24];
    prop_json_copy_val(v, vlen, t, (int)sizeof(t));
    if (strncmp(t, "true",  4) == 0) { *out = 1U; return 0; }
    if (strncmp(t, "false", 5) == 0) { *out = 0U; return 0; }
    *out = (strtof(t, NULL) != 0.0f) ? 1U : 0U;
    return 0;
}

/* ---------- 对外 API ---------- */
void ESP01S_AliIot_Init(void)
{
    s_prop_move    = 0U;
    s_prop_stopped = 1U;
    s_prop_led     = 0U;
    s_prop_temp_c  = 0.0f;
    LED_R(0);
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

int ESP01S_AliIot_HandleSet(const char *payload, char *reply, uint16_t reply_max)
{
    if (payload == NULL || reply == NULL || reply_max < 32U) return 0;

    int slen = (int)strlen(payload);
    int hits = 0;

    /* 1) 取 id（用于 reply 回带；缺失时用 "0"，云端仍能匹配默认会话） */
    char id[32] = "0";
    int  vlen = 0;
    const char *v = prop_json_get(payload, slen, "id", &vlen);
    if (v != NULL && vlen > 0) {
        char tmp[32];
        prop_json_copy_val(v, vlen, tmp, (int)sizeof(tmp));
        char *p = tmp;                                  /* 去掉可能存在的引号 */
        if (p[0] == '"') p++;
        int n = (int)strlen(p);
        if (n > 0 && p[n - 1] == '"') p[n - 1] = '\0';
        if (p[0] != '\0') strncpy(id, p, sizeof(id) - 1U);
        id[sizeof(id) - 1U] = '\0';
    }

    /* 2) 遍历属性表，命中且可写则执行 setter */
    for (int i = 0; i < (int)ESP01S_PROP_COUNT; i++) {
        const esp01s_prop_desc_t *d = &s_prop_desc[i];
        if (!d->writable || d->set_bool == NULL) continue;   /* 只读属性不接受下发 */

        v = prop_json_get(payload, slen, d->ident, &vlen);
        if (v == NULL || vlen <= 0) continue;

        uint8_t bval = 0U;
        if (prop_json_parse_bool(v, vlen, &bval) != 0) {
#if DBG_LOG_NET
            LOG_W("NET", "aliiot: %s bad value", d->ident);
#endif
            continue;
        }
        d->set_bool(bval);
        hits++;
#if DBG_LOG_NET
        LOG_D("NET", "aliiot: set %s = %u", d->ident, (unsigned)bval);
#endif
    }

    /* 3) 组装 set_reply：{"id":"..","code":200,"data":{}} */
    int n = snprintf(reply, reply_max, "{\"id\":\"%s\",\"code\":200,\"data\":{}}", id);
    if (n < 0 || n >= (int)reply_max) {
        /* 极端情况（id 被截到超长）退回无 id 的最小合法 reply */
        (void)snprintf(reply, reply_max, "{\"id\":\"0\",\"code\":200,\"data\":{}}");
    }
#if DBG_LOG_NET
    LOG_D("NET", "aliiot: set hits=%d reply=%s", hits, reply);
#endif
    return hits;
}

int ESP01S_AliIot_BuildReport(char *buf, uint16_t max)
{
    if (buf == NULL || max < 64U) return -1;

    static uint32_t s_prop_seq = 1U;    /* 上报自增 id，便于云端日志对齐 */

    int o = snprintf(buf, max,
                     "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{",
                     (unsigned long)s_prop_seq);
    if (o < 0 || o >= (int)max) return -1;

    /* 逗号由「是否已写过一项」决定，而非索引 i（有属性被跳过也不会漏/多逗号） */
    int first = 1;
    for (int i = 0; i < (int)ESP01S_PROP_COUNT; i++) {
        const esp01s_prop_desc_t *d = &s_prop_desc[i];
        int n;
        if (d->is_float && d->get_float != NULL) {
            n = snprintf(&buf[o], (size_t)((uint16_t)max - (uint16_t)o),
                         "%s\"%s\":%.2f", (first ? "" : ","), d->ident, (double)d->get_float());
        } else if (d->get_bool != NULL) {
            n = snprintf(&buf[o], (size_t)((uint16_t)max - (uint16_t)o),
                         "%s\"%s\":%u", (first ? "" : ","), d->ident, (unsigned)d->get_bool());
        } else {
            continue;
        }
        if (n < 0 || (o + n) >= (int)max) return -1;   /* 缓冲不足：整帧丢弃，不截断上报 */
        o += n;
        first = 0;
    }

    int n = snprintf(&buf[o], (size_t)((uint16_t)max - (uint16_t)o),
                     "},\"method\":\"thing.event.property.post\"}");
    if (n < 0 || (o + n) >= (int)max) return -1;
    o += n;

    s_prop_seq++;
    return o;
}

void ESP01S_AliIot_UpdateTempRaw(int16_t mpu_temp_raw)
{
    s_prop_temp_c = ((float)mpu_temp_raw / 340.0f) + 36.53f;   /* MPU6050 datasheet */
}

/* ===================== [迁移] 网络自检：从 selftest.c 下沉到本组件（按 APP_ENABLE_NETWORK 门控） ===================== */
#if defined(APP_ENABLE_NETWORK) && APP_ENABLE_NETWORK
int Network_Test(void)
{
#if DBG_LOG_POSTEST
    LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", "Network_Test enter");
#endif
    log_wdt_feed();
    /* POST 期绝不碰 USART2 线缆：ESP01S 完整握手（ESP01S_Init → AT/CWMODE/CWJAP/MQTT）
       由 StartNetworkTask 在 POST 之后异步完成，那才是真正的链路校验。
       POST 此处只做「非线缆」检查：确认 CubeMX 已初始化 USART2 句柄
       （MX_USART2_UART_Init 在 main.c 跑过、huart2.Instance 非空、未处于错误态）。
       为何不能发 AT 探活：POST 时 ESP01S_Init 尚未运行 → USART2 RX ISR 未武装
       （ReceiveToIdle_IT 只在 ESP01S_Init 内 esp01s_rx_restart_clean 调用）；
       此时发 "AT" 后 ESP 回的 "OK" 无处可读 → 硬件 ORE 溢出 →
       uart_err_monitor 的致命计数器（跨整机上电单调累加、永不复位）被污染，
       会把后续真实握手的容错余量吃掉，可能误触 3 次阈值 → 黑匣子+复位。 */
    if (huart2.Instance == NULL) {
        LOG_W("POSTEST", "Network USART2 handle not init -> SKIP (NetworkTask will init)");
        return -1;   /* 非关键 critical=0 */
    }
    if (huart2.gState == HAL_UART_STATE_ERROR) {
        LOG_W("POSTEST", "Network USART2 in ERROR state -> SKIP (NetworkTask recovers)");
        return -1;
    }
    LOG_I("POSTEST", "Network USART2 handle OK (deferred: full handshake in StartNetworkTask)");
    return 0;
}
#endif

#endif /* APP_ENABLE_NETWORK */
