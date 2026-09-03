# ESP-AT MQTT 指令集 — TLS 加密接入参考

> 来源：乐鑫官方 ESP-AT 用户指南《MQTT AT 命令集》
> https://docs.espressif.com/projects/esp-at/zh_CN/release-v3.0.0.0/esp32c2/AT_Command_Set/MQTT_AT_Commands.html
> 整理日期：2026-09-02（ESP-01S / ESP8266 适用，指令集一致）

## 1. AT+MQTTUSERCFG — 核心配置
```
AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">
```
### `<scheme>` 取值（重点）
| scheme | 含义 |
|---|---|
| 1 | MQTT over TCP（明文，对应阿里云 1883 / securemode=3） |
| 2 | MQTT over TLS（**不校验证书**） |
| **3** | **MQTT over TLS（校验 server 证书）← 对接阿里云用这个** |
| 4 | TLS + 提供 client 证书（双向认证） |
| 5 | TLS 校验 server + 提供 client 证书 |
| 6~10 | WebSocket / WebSocket Secure 变体 |

- `<cert_key_ID>` / `<CA_ID>`：当前 ESP-AT 仅支持一套，固定填 `0`。
- 单条 AT 总长 ≤ 256 字节（超长用 AT+MQTTLONGCLIENTID / LONGPASSWORD）。

## 2. TLS 完整连接示例（官方）
```
AT+CWMODE=1
AT+CWJAP="ssid","password"
AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com","ntp2.aliyun.com"   // 校验证书有效期必须先有正确时间
AT+MQTTUSERCFG=0,3,"ESP32-C2","espressif","1234567890",0,0,""
AT+MQTTCONN=0,"192.168.200.2",8883,1
```

## 3. 证书校验的时间要求（易错）
- scheme=3/5/8/10（校验 server 证书）时，**AT+MQTTCONN 前必须已通过 SNTP 拿到当前时间**，否则证书有效期判断失败、握手失败。
- 配时：`AT+CIPSNTPCFG=1,8,"ntp1.aliyun.com"`；查时：`AT+CIPSNTPTIME?`。

## 4. CA 根证书加载
- 若烧录的是"阿里云专有/内置 CA"固件（如安信可 MQTT 固件、乐鑫 esp-at 带 cert 分区），scheme=3 直接用内置 CA 校验，无需额外指令。
- 动态加载：`AT+SSLROOTCERT=<index>,"<PEM 或 HEX 证书>"`（分片 ≤1024 字节写入阿里云根证书）。
- ESP-01S 1MB Flash：务必选**含 MQTT+TLS+CA 的固件**，否则无 scheme=3 能力。

## 5. 关键错误码（TLS 相关）
| 错误码 | 含义 |
|---|---|
| 0x6009 | TLS 配置错误 |
| 0x6052 | 主机名校验失败（证书/域名不匹配） |
| 0x6051 | 处于断开态 |
| 0x6005 | 内存分配失败（TLS 较吃 RAM） |

## 6. 连接状态/事件
- 建立：`+MQTTCONNECTED:<LinkID>,<scheme>,<host>,<port>,...`
- 断开：`+MQTTDISCONNECTED:<LinkID>`
- 订阅收到：`+MQTTSUBRECV:<LinkID>,<topic>,<len>,<data>`
