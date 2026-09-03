# ESP-01S MQTT over TLS 移植方案 — STM32(FreeRTOS) 落地

> 整理：2026-09-02。基于阿里云官方接入文档 + 乐鑫 ESP-AT 指令集，面向现有 `Components/BSP/ESP01S` 的 9 步 AT 序列做最小改动升级。

## 0. 结论先行（架构判断）
**TLS 应在 ESP-01S 模组侧做，不在 STM32 做。**
- 你当前架构：STM32 只发 AT 指令、ESP-01S 跑 TCP/IP+MQTT 栈。TLS 是 TCP 之上的加密层，天然由模组固件承担。
- 因此"在 STM32 移植 mbed TLS / WolfSSL"不是你的方案（那是把 ESP 切透传、STM32 自己跑 TLS 栈，工作量翻几倍，仅在你放弃 AT、改透传时才考虑）。
- **STM32 代码改动极小**：只改 AT 指令序列的几个参数 + 增加 SNTP 时间同步 + 证书入模组。认证（HMAC 签名 password）逻辑完全复用。

## 1. 两条路径对比
| 路径 | 做法 | TLS 执行方 | 改动量 | 适用 |
|---|---|---|---|---|
| **A（推荐）** | 烧 SSL AT 固件，AT scheme=3 + 8883 | ESP-01S | 极小（改 AT 序列） | 你的现状 |
| B | ESP 透传，STM32 跑 mbedTLS+MQTT | STM32H743 | 大（TLS bio/UART、栈、证书管理） | 模组无 SSL 或要证书存 MCU |

> STM32H743 有 1MB RAM，路径 B 技术上可行，但**对你无收益、徒增风险**，不推荐。

## 2. 你的 9 步序列 → TLS 升级 diff
原（明文 1883）：
```
CWJAP → MQTTUSERCFG(scheme=1) → MQTTCLIENTID → MQTTCONN(1883) → MQTTSUB → ...
```
改（TLS 8883）：
```
CWJAP
→ CIPSNTPCFG=1,8,"ntp1.aliyun.com"        // 新增：证书校验前必须有时间
→ MQTTUSERCFG(0,3,"<id|securemode=2,...|>","<user>","<pwd>",0,0,"")   // scheme 1→3；clientId 内 securemode=3→2
→ MQTTCLIENTID(0,"<id|securemode=2,...|>")   // 与 USERCFG 中 clientId 一致
→ MQTTCONN(0,"<pk>.iot-as-mqtt.<region>.aliyuncs.com",8883,1)   // 端口 1883→8883
→ MQTTSUB / MQTTUNSUB / MQTTPUB 不变
```
要点：
- **scheme=3**（AT 层 TLS 校验 server 证书） ↔ 阿里云 **securemode=2**（CONNECT 层 TLS 模式） ↔ **端口 8883**。
- password 仍用 DeviceSecret 的 hmacsha1 派生，**不变**。
- AT 总长 ≤256 字节：`clientId` 长就改用 `AT+MQTTLONGCLIENTID`。

## 3. 完整 TLS AT 序列模板（可直接落 BSP）
```
AT+CWMODE=1
AT+CWJAP="<SSID>","<PWD>"
AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com","ntp2.aliyun.com"
AT+MQTTUSERCFG=0,3,"<clientId>|securemode=2,signmethod=hmacsha1|","<deviceName>&<productKey>","<hmacsha1_password>",0,0,""
AT+MQTTCONN=0,"<productKey>.iot-as-mqtt.<region>.aliyuncs.com",8883,1
AT+MQTTSUB=0,"/sys/<productKey>/<deviceName>/thing/service/property/set",1
```
> 证书：用"阿里云专有/内置 CA"固件则 scheme=3 自动校验；否则 `AT+SSLROOTCERT` 写入阿里云自签 CA（MD5 c7a6afb466713832af778a7bcb6d1aef，8883 用）。

## 4. RTOS（FreeRTOS / CMSIS-RTOS2）落地要点
- **任务划分**：现有 `StartNetTask` / `uart_rx_dispatcher` 按 huart 分发不变；TLS 握手是阻塞操作，必须在 ESP 专属任务内、用 `osDelay` 让出 CPU，禁止在中断/HardFault 路径调 AT。
- **状态机**：建议把连接状态从"已连 TCP"细化为 `WIFI_OK → NTP_OK → MQTT_TLS_CONNECTED → SUB_OK`，SNTP 失败即不进 CONNECT，避免无谓 TLS 重试。
- **证书存储**：CA 烧在模组固件（推荐）；若要 OTA 轮换证书，走 `AT+SSLROOTCERT` 动态写（官方要求设备具备更新 CA 能力，因旧 Global Sign R1 2028 到期）。
- **时间同步**：上电先 `CIPSNTPCFG`+等 SNTP 拿到时间，再 `MQTTCONN`；时间误差 >15min 证书校验必失败（已知坑）。
- **重连**：`AT+MQTTCONN ...reconnect=1` 自动重连吃内存；建议 STM32 侧状态机检测 `+MQTTDISCONNECTED` 后主动 `MQTTCLEAN`+重配+重连，便于看门狗心跳管理与遥测。
- **与日志系统**：TLS 握手失败走 `LOG_E`（Channel B 保证通道，永静音）；不要用遥测静音逻辑压 ERROR。

## 5. 易错清单（踩坑点）
1. **固件不对**：ESP-01S 出厂通用 AT 固件常无 MQTT+SSL；必须烧"阿里云/安信可 MQTT 固件"或乐鑫 esp-at（含 cert 分区）。
2. **忘记 SNTP**：scheme=3 不先同步时间 → 证书有效期判断失败、握手失败（ERR 0x6009）。
3. **端口/证书不匹配**：8883 必须配自签 CA（2053 到期）；用旧 Global Sign R1 只能 1883 且 2028 到期。
4. **securemode 写反**：TLS 用 `2`，明文用 `3`；AT scheme 用 `3` 校验证书。
5. **AT 超 256 字节**：clientId 带扩展参数易超长 → 改 `AT+MQTTLONGCLIENTID`。
6. **DNS/域名**：`iot-as-mqtt.cn-shanghai.aliyuncs.com`（公共实例）或企业版终端节点。
7. **一机一密**：同一三元组多设备共用 → 互相挤下线。

## 6. 建议下一步
- 确认你手头 ESP-01S 当前固件是否支持 scheme=3（发 `AT+GMR` 看版本 / 试 `AT+MQTTUSERCFG=0,3,...` 响应）。
- 若支持：直接按 §3 模板改 `Components/BSP/ESP01S` 的 AT 序列，复用现有签名代码。
- 若不支持：先烧固件（安信可 MQTT 固件 V1.6+ / 乐鑫 esp-at），再改序列。
