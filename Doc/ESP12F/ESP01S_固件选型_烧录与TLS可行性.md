# ESP-01S 固件选型 · TLS 可行性 · 烧录指南

> 适用：STM32 + ESP-01S（AT 指令架构）对接阿里云物联网平台，需从 1883 明文升级到 8883 TLS。
> 配套文档：《ESP01S_MQTT_TLS_移植方案与RTOS落地.md》（AT 序列改造）、《阿里云MQTT_TLS接入_官方文档整理.md》、《ESP-AT_MQTT_TLS指令参考.md》。

## 1. 你当前的 1471 固件身份
- **名称**：安信可 ESP8266 AT MQTT 常规固件 **V2.2.0**，固件号 **1471**，1MB 版 `(1471)ESP8266-AT_MQTT-1M.bin`。
- **适配**：专为 **1MB(8Mbit) Flash** 模组（ESP-01S / ESP-01M）。烧录地址 `0x0`，SPI MODE=DOUT，FLASH SIZE=8Mbit。
- **能力**：带 MQTT 指令集（`AT+MQTTUSERCFG` / `AT+MQTTCONN` / `AT+MQTTPUB` / `AT+MQTTSUB`），**不含 SSL/TLS** → 所以当前是 1883 明文。这不是配置错，是固件本身无 TLS。
- **验证**：`AT+GMR` 回显应含 `AT version:1.7.x...` + `Bin version:1471...`。发我回显可精确定位子版本。

## 2. 最新版本下载地址
| 来源 | 地址 | 说明 |
|---|---|---|
| 安信可官网固件汇总（权威入口） | https://docs.ai-thinker.com/esp8266 → 各类AT固件 | 含全部固件号与指令集文档 |
| 1471 直链（1MB MQTT 明文） | https://docs.ai-thinker.com/_media/1471_esp8266-at_mqtt-1m.zip | 你当前固件；**1MB 版最新即 V2.2.0，无更高 1MB 版** |
| 1112 直链（4MB MQTT） | 官网 1112 号 ESP8266 AT MQTT常规固件 V2.3.0 | 4MB 版最新；需换 4MB Flash 模组 |
| 乐鑫官方 esp-at | https://github.com/espressif/esp-at （release v2.2.0.0_esp8266） | 可自编译开启 `AT_MQTT_COMMAND_SUPPORT` + `AT_SSL_COMMAND_SUPPORT`，含 cert 分区 |
| 开源 esp_atmod（TLS） | https://github.com/jiribilek/esp_atmod | BearSSL，作者明确"fits into 1024KB，可跑 ESP-01 8Mbit"；**仅 TCP+TLS，MQTT 需自组**，证书靠 LittleFS |

⚠️ **安信可没有"1471 的 TLS 升级版"**。要 TLS 必须换固件或换模组。

## 3. ESP-01S 能烧 TLS 固件吗（核心结论）
- 1471 能烧 ESP-01S ✓（专门适配），但它不带 TLS。
- **1MB Flash 是硬约束**：
  - 安信可官方"支持TLS连接"的 MQTT 固件（③/④号，2020）年代旧、AT 口定义特殊（③=IO13/IO15，④=UART0），且未确认有 1MB 版。
  - 乐鑫 esp-at 对 ESP8266 的 1MB 支持需自定义分区，工程化成本高。
  - 开源 esp_atmod 证明 **1MB 能跑 TLS**（BearSSL + 证书指纹校验），但 AT 指令不全（仅 TCP+TLS，无 MQTT 指令；MQTT 报文需 STM32 自己组）+ 证书管理靠 LittleFS 自编译。
- **结论**：ESP-01S(1MB) 上"模组自带 MQTT 指令 + TLS"的组合，官方无现成固件；硬上 TLS 只有开源坑多路线，**不划算**。

## 4. 四方案决策表
| 方案 | 做法 | STM32 改动 | 安全性 | 工程成本 | 结论 |
|---|---|---|---|---|---|
| **A 换模组（推荐）** | ESP-01S → ESP-12F/12S（4MB，pin 兼容器）烧 1112 / 乐鑫 esp-at（MQTT+TLS） | 几乎不动（改 AT 序列 scheme=3 / 8883 / 加 SNTP） | 传输层 TLS ✓ | 低（换料+烧录） | **工业级稳妥，符合 AT 架构不动初衷** |
| B 开源 TLS 固件 | ESP-01S 烧 esp_atmod（1MB TLS） | 中（自组 MQTT 报文 over TLS 管道） | TLS ✓ 但无模组 MQTT 指令 | 高（自编译+证书） | 能跑但不推荐 |
| C 应用层加密 | 保持 1471 明文，STM32 对 payload 做 AES（DeviceSecret 派生） | 中（加解密层） | 仅 payload 加密，三元组仍明文过公网 | 中 | 仅测试/局域，**生产不推荐** |
| D MCU 跑 TLS 透传 | ESP-01S 切 TCP 透传，H743 跑 mbedTLS+MQTT | 大（完整 TLS+MQTT 栈） | TLS ✓ | 极高 | 违背"TLS 由模组做"初衷 |

## 5. 推荐落地：方案 A
1. **换料**：ESP-01S → **ESP-12F / ESP-12S**（4MB Flash，硬件基本 pin2pin；注意 IO0/EN 启动脚一致，供电用 3.3V LDO 勿用 USB-TTL 3.3V）。
2. **烧录**：乐鑫 esp-at（release v2.2.0.0_esp8266，4MB 版，开启 MQTT+SSL 命令）或安信可 1112（V2.3.0）。
3. **AT 序列改造**（详见《ESP01S_MQTT_TLS_移植方案与RTOS落地.md》§3）：
   ```
   CIPSNTPCFG(先校时) → MQTTUSERCFG(0,3,"...|securemode=2,...|") → MQTTCONN(8883)
   ```
4. `password` **复用**现有 `DeviceSecret → hmacsha1` 派生，无需重写。

## 6. 烧录要点（1471 同款，换模组同理）
- 工具：ESP_DOWNLOAD_TOOL（安信可/乐鑫官网）。
- 进入下载模式：IO0=GND，EN=3V3，上电；烧完 IO0 拉高重新上电。
- 参数：CrystalFreq=26M（禁改）、SPI SPEED=40M、SPI MODE=DOUT、FLASH SIZE 按模组（8Mbit/32Mbit）。
- 固件起始 `0x0`，勾选、取消 DoNotChgBin；波特率 115200。
- 校核：`AT+GMR` 看版本；`AT+MQTTUSERCFG=0,3,...` 试响应（ERROR=不支持 TLS，OK=支持）。

## 7. ⚠️ 给你的决断
- **坚持不换 ESP-01S 硬件** → 只能走 B（esp_atmod，坑多）或 C（应用层加密，不安全）。TLS 传输层在 1MB 官方路线上无解。
- **可换料** → A 最优：STM32 代码改动最小、TLS 由模组承担、符合"AT 架构不动"初衷。
