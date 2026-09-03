# ESP-12F / ESP-12S 裸模组：买回来后要做哪些操作

> 适用：原用 ESP-01S（1MB）想换 12F/12S（4MB）上 MQTT+TLS，但只买到裸 SMD 模组。
> 配套：《ESP01S_固件选型_烧录与TLS可行性.md》（为何换、四方案）、《ESP01S_MQTT_TLS_移植方案与RTOS落地.md》（AT 序列改造）。

## 0. 先认清差异（为什么 12F/12S 比 01S 多事）
| 项 | ESP-01S | ESP-12F / ESP-12S |
|---|---|---|
| 形态 | 8-pin DIP 2.54mm，自带焊盘+基本外围 | **SMD 邮票孔/半孔 2.0mm，裸模组** |
| Flash | 1MB(8Mbit) | **4MB(32Mbit) 内置** ✓（核心收益） |
| 天线 | 板载 | 板载 PCB 天线（无需外接） |
| 外围 | EN 内部已上拉，基本即插即用 | **需自接 EN/GPIO15/GPIO0/GPIO2 上拉下拉** |
| 物理兼容 | 直接插 8-pin 座 | **与 01S 2.54mm 座不兼容**，需转接板/改板 |

→ 12F/12S 不是即插即用模块，是**裸 SMD 模组**，需载体 + 最小外围。换它的理由就是 **4MB Flash 装得下 MQTT+TLS 固件**。

## 1. 两种落地路径（二选一）
- **路径 A（推荐·最快验证）**：买"ESP-12F/12S 转接板/最小系统板"（淘宝 1~3 元，已集成 EN 上拉/GPIO15 下拉/LDO/排针/天线净空）→ 焊上裸模组 = 功能更强的 ESP-01S → 按 ESP-01S 接法接 STM32。
- **路径 B（产品级）**：STM32 主板上画 12F/12S 封装焊盘 + 最小外围，替换原 ESP-01S 座位置。

## 2. 最小外围电路（路径 B 必备；路径 A 已替你焊好）
| 信号 | 接法 | 作用 |
|---|---|---|
| EN(CH_PD) | 10kΩ → 3.3V | 使能，必须高 |
| GPIO15 | 10kΩ → GND（或硬接地） | 启动 strapping，必须低，否则不启动 |
| GPIO0 | 10kΩ → 3.3V；烧录时临时接 GND | 高=运行 / 低=下载 |
| GPIO2 | 10kΩ → 3.3V | 启动必须高 |
| RST | 10kΩ → 3.3V + 按键→GND（可选） | 手动复位 |
| VCC | 3.3V LDO（≥500mA，如 AMS1117-3.3） | **勿用 USB-TTL 3.3V**（Wi-Fi 峰值 300mA 不够） |
| 去耦 | 100nF + 10μF 靠近 VCC-GND | 稳压，防突发掉电 |
| GND | 多 GND 脚都接（散热+RF） | — |
| 天线净空 | 板载天线一侧 PCB 不铺铜/不走线 | 保 RF 性能 |

⚠️ **启动电平（上电瞬间采样）**：EN=1, GPIO15=0, GPIO0=1, GPIO2=1 → 正常运行；GPIO0=0 → 下载模式。

## 3. 与 STM32 接线（沿用 ESP-01S 的 UART0）
- ESP-12F/12S UART0 = **GPIO1(TX) / GPIO3(RX)**，与 ESP-01S 一致。
- **3.3V 直连**（STM32 也是 3.3V，无需电平转换；ESP8266 不耐 5V）。
- 物理：12F 是 2.0mm SMD，**不能直接插原 ESP-01S 2.54mm 座** → 用转接板或改板引 2.54mm 排针再接 STM32。

## 4. 烧录（4MB 固件）
- 进下载：GPIO0=GND + EN 拉高上电（路径 A 转接板有 FLASH 键/跳线；路径 B 自己接按键）。
- 工具：ESP_DOWNLOAD_TOOL → Developer Mode → ESP8266 DownloadTool。
- 参数：CrystalFreq=26M，SPI SPEED=40M，SPI MODE=DOUT，**FLASH SIZE=32Mbit**，固件起始 0x0，取消 DoNotChgBin，波特率 115200。
- 固件：4MB 版——**1112 号 V2.3.0**（安信可）或 **乐鑫 esp-at release v2.2.0.0_esp8266 4MB 版**（开启 `AT_MQTT_COMMAND_SUPPORT`+`AT_SSL_COMMAND_SUPPORT`）。
- 校核：`AT+GMR` 看版本；`AT+MQTTUSERCFG=0,3,...` 试响应（OK=支持 TLS，ERROR=不支持）。

## 5. 采购注意
- Flash ≥ 4MB（12F/12S 均为 32Mbit=4MB，确认别买到旧 12E 或其他特殊版）。
- 板载天线版（默认），无需外接天线。
- 12S 有"阿里飞燕认证"版（天线匹配已做）；12F 通用。
- 买"裸模组 + 转接板"套装最省事（避免自己画最小系统）。

## 6. 后续
烧好 4MB MQTT+TLS 固件后，按《ESP01S_MQTT_TLS_移植方案与RTOS落地.md》§3 改 STM32 侧 AT 序列：
`CIPSNTPCFG` 校时 → `MQTTUSERCFG(scheme=3)` → `MQTTCONN(8883)`；`password` 复用 DeviceSecret→hmacsha1 派生。
