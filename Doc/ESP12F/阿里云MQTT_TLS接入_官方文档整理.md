# 阿里云物联网平台 MQTT(S) 原生协议接入 — 官方文档整理

> 来源：阿里云帮助中心《MQTT(S)原生协议接入》https://help.aliyun.com/zh/document_detail/2860281.html
> 证书/端口对照另据《MQTT-TLS连接通信》https://help.aliyun.com/document_detail/73742.html
> 整理日期：2026-09-02（面向 STM32 + ESP-01S AT 对接场景）

## 1. 官方安全立场
- **强烈建议 TLS**：设备使用 TCP（非加密）接入安全风险非常高，新建企业版实例默认关闭 TCP 接入。
- 无 TLS 的 1883 明文连接仅建议测试验证，不推荐生产环境。
- 自行开发设备端（即不用 Link SDK）须用**根证书**完成对平台的鉴权。

## 2. 端口 / 根证书对照表
| 端口 | 加密方式 | 根证书 | 有效期 | 备注 |
|---|---|---|---|---|
| 8883 | TLS（推荐） | 阿里云物联网平台自签名 CA | 至 **2053-07-04** | MD5 `c7a6afb466713832af778a7bcb6d1aef`；端口与证书版本必须匹配 |
| 443 | TLS | 同上自签 CA / 兼容 | 长期 | 可走 443 规避网络限制 |
| 1883 | TLS（旧） | Global Sign R1 | 至 **2028-01-28** | 旧版证书；到期后依赖它的设备无法接入 |
| 1883 | 明文（无 TLS） | 无 | — | 不安全，仅测试 |

> 关键：端口必须与所用根证书版本匹配，否则连接失败。自签 CA（8883）长期有效，优先采用。

## 3. MQTT CONNECT 报文参数（一机一密）
```
mqttClientId : clientId + "|securemode=<2|3>,signmethod=<hmacsha1|hmacsha256|hmacmd5>[,timestamp=xxx]|"
mqttUsername : deviceName + "&" + productKey
mqttPassword : sign_hmac(deviceSecret, content)   // content=按参数名字典序拼接
```
- **securemode 取值**：
  - `2` = TLS 直连（配合 8883 + 根证书）
  - `3` = TCP 直连（配合 1883 明文）
- **signmethod**：hmacsha1 / hmacsha256 / hmacmd5（与平台一致即可）
- **clientId**：自定义，≤64 字符，建议用 MAC/SN 便于区分设备
- **timestamp**：当前毫秒值，可选；若填须与 clientId 内一致

### 签名计算（示例）
参数：clientId=12345, deviceName=device, productKey=pk, timestamp=789, signmethod=hmacsha1, deviceSecret=secret
```
content = "clientId12345deviceNamedeviceproductKeypktimestamp789"   // 参数名首字母字典序拼接值
mqttPassword = hmacsha1(secret, content)  → 二进制转十六进制字符串
```
> 你的工程已用 DeviceSecret 运行时派生 password（hmacsha1），逻辑不变；TLS 只是"加密传输"，认证方式（签名）完全沿用。

## 4. 保活（Keep Alive）
- 取值 30s~1200s，建议 ≥300s。
- 平台每 30s 检测一次心跳；超时 = keepalive*1.5 + 等待时间 未收到报文则断开。

## 5. 官方风险提示
- 同一设备证书/ClientID 多设备共用 → 频繁上下线。
- TCP 明文接入：业务 payload 公网裸传，仅身份靠签名保护。
