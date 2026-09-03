# NET 任务运行期故障归档（ESP-01S WiFi / MQTT / 阿里云）

> 本文件是 **NET 任务**（StartNetworkTask + ESP-01S 无线模组 / MQTT 上云）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/Error/Error_Readme_idx.md` 一致）。
> 联调笔记见 `Components/Debug/debug/net_debug.md`（待建）；测试方案见 `Components/Debug/Test/net_test.md`。
> 编译期故障归 `Components/Debug/Error/Error_Readme_idx.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

> 原则：未实测到、未确认根因的事件**一律不编造条目**。
> 新增运行期故障时，直接在本文件追加「事件 E N」（编号接 `Components/Debug/Error/Error_Readme_idx.md` 全局最大编号，不重置），
> 每个事件含「现象 / 调试器定位 / 根因链 / 修复方案 / 验证步骤 / 状态」六段（格式参考 `Error/motor_error.md`）。

---

## 事件登记位
| 事件 | 现象摘要 | 状态 |
|------|----------|------|
| **E31** | 一次 AT 超时后网络永久死亡：每轮重连都卡 `USART2 RX start failed`，再也回不到 AT 阶段 | ✅ 已修复并验证（重连已能反复发生） |
| **E32** | USART2 的 NVIC 从未使能 → 永远收不到任何字节（回环自测 rx 0 bytes 暴露） | ✅ 已修复并验证（WiFi 已连通） |
| **E33** | MQTT CONNECT 报文 217 B > 缓冲 160 B；顺带发现 PUBLISH 缓冲无长度检查（P0 栈溢出） | ✅ 已修复并验证（CONNECT 已能组包发出） |
| **E34** | MCU 复位但模块没断电 → 卡在透传模式，AT 全部失效（症状同"模块没电"，极易误判） | ✅ 已验证（日志出现 transparent-exit，握手全程通过） |
| **E35** | CONNACK 返回码未参与判定（rc=4/5 被当成功）+ 三元组 signmethod/timestamp 不一致 | ✅ 代码已修；凭据待对齐（见 E36） |
| **E36** | 配置密码带 `signmethod=/timestamp=` 前缀，broker 期望 RAW 64-hex 签名 → 签名失败被静默丢弃 | ✅ 已修（密码前缀）；但非主因，见 E37 |
| **E37** | `mqtt_build_connect` 剩余长度少算 1B（可变头 9→应 10）→ CONNECT 畸形，broker 静默断链、无 CONNACK | ✅ 已修复并验证（CONNACK rc=0，已上线并订阅 property/set） |

---

## 事件 E31：一次 AT 超时后网络永久死亡（2026-08-31）

### 现象（用户实测串口日志）
```
[3118][D][NET] (esp01s.c:200) AT handshake
[4218][E][NET] (esp01s.c:203) ESP-01S no AT echo (check wiring/baud/power)
[4218][W][NET] (freertos.c:638) connect failed (1/6), retry in 10000 ms
[3670][E][NET] (esp01s.c:193) USART2 RX start failed
[3670][W][NET] (freertos.c:638) connect failed (2/6), retry in 20000 ms
[5777][E][NET] (esp01s.c:193) USART2 RX start failed
...（每轮都是 RX start failed，再没出现 AT handshake）
[4744][I][NET] (freertos.c:631) entering OFFLINE (reconnect every 60 s)
```

**特征**：第一条是 `no AT echo`，**之后全部变成 `RX start failed`，永远回不到 AT 阶段**，最终进 OFFLINE 每 60s 空转。
即：**瞬时的一次模块无响应，演变成"网络永久死亡"**。

### 根因链
1. `ESP01S_Init()` 调 `HAL_UARTEx_ReceiveToIdle_IT()` arm USART2 接收；
2. **ReceiveToIdle 只有收到 IDLE 中断或达到指定长度，才会回到 READY**；
3. 第一次发 `AT` 后 ESP-01S 无响应 → 1s 超时 → `ESP01S_Init` 返回 `ERR_NORESP`，
   但 **RX 一直停在 `BUSY_RX`**（还在等那个永远不会来的 IDLE）；
4. 下一轮重连再调 `ReceiveToIdle_IT` → UART 忙 → 返回 **`HAL_BUSY`** →
   打印 `USART2 RX start failed` → `return ESP01S_ERR_LINK`；
5. **连 AT 都发不出去了** → 每轮卡在同一处 → 网络永久死亡。
   即使后来把模块插好、线接对，也**永远起不来，只能复位 MCU**。

> 与离线重连的设计目标相悖：状态机本应"退避后重试"，实际是"退避后重复同一个必败操作"。

### 修复方案（代码，已落地）
- `esp01s.c` 新增 `esp01s_rx_restart_clean()`：arm 前先判 `huart2.RxState != HAL_UART_STATE_READY`，
  成立则 `HAL_UART_AbortReceive()` 把 UART 拉回 READY，再调 `ESP01S_UART_RxStart()`；
- `ESP01S_Init()` 的裸 `ReceiveToIdle_IT` 改调 `esp01s_rx_restart_clean()`；
- 错误日志加状态码：`USART2 RX start failed (st=%d RxState=%d)` —— 便于区分
  `HAL_BUSY(=2)`（上一轮没清干净）与 `HAL_ERROR(=1)`（硬件未就绪）；
- **为什么不改 `ESP01S_UART_RxStart()` 本体**：它被 ISR 回调（`ESP01S_UART_RxCallback`）
  与错误恢复路径调用，而 `AbortReceive` 是**阻塞式**，ISR 中调用有挂死风险。
  故拆成「轻量版（ISR 用）+ 清洁版（任务上下文用）」两个；
- 顺带修正 `esp01s.h` 的 `ESP01S_UART_BAUD`：921600 → **115200**
  （与 `usart.c:136` `huart2.Init.BaudRate = 115200` 实测一致；旧值是陈旧值，会误导波特率排查）。

### 验证步骤（用户侧）
1. Keil Rebuild 0E/0W、烧录；
2. **故意不接 ESP-01S** 上电：仍会打印 `no AT echo`，但**后续每轮都必须重新出现 `AT handshake`**
   —— 若又变成 `RX start failed` 死循环，说明 Abort 没生效（看 `st=` 与 `RxState=`）；
3. 接好模块上电：应出现 `WiFi connected` → `MQTT CONNACK OK` → `subscribed: .../property/set`；
4. 若每轮仍无 AT 回响但**能反复重试**，说明 E31 已修复，剩下的纯属硬件问题（见下）。

### 未决：首次 AT 无回响的硬件排查（本事件的触发源，尚未定位）

#### 决定性判据 A：字节级诊断（代码已加，2026-08-31）
握手失败时打印实际收到的字节数与前 16 字节 hex，据此二选一：

| 日志 | 含义 | 下一步 |
|---|---|---|
| `no AT echo: rx 0 bytes -> module SILENT` | 模块**一个字节都没回** | 查供电 / EN / GPIO0 / TX-RX 接线（下表 1–6） |
| `no AT echo: rx N bytes [XX XX ...] -> garbage=baud mismatch` | 模块**回了但解析不出 OK** | 波特率不匹配（双方须同为 115200），或线接触不良产生噪声 |

> 注：此日志走 `LOG_E`（ERROR 级），**不受 `DBG_LOG_NET` 门控**，关了调试开关也能看到。
> 它同时是 E31 是否修好的判据：若反复出现的是 `USART2 RX start failed` 而非本条，说明 Abort 没生效。

#### 决定性判据 B：PA2/PA3 回环自测（强烈建议先做）
1. **断开 ESP-01S**，用杜邦线短接 **PA2 ↔ PA3**；
2. 上电，看握手失败时的日志：
   - `rx 4 bytes [41 54 0D 0A]`（即 `AT\r\n`）→ **STM32 侧 TX/RX/USART2 全部正常**，
     问题 100% 在模块侧（供电 / EN / 波特率），不必再怀疑 MCU 引脚配置；
   - 仍 `rx 0 bytes` → STM32 侧问题（引脚 AF 配置 / TX 没发出 / NVIC），先修这个再谈模块。

#### 硬件排查 7 项
| 序号 | 检查项 | 正确状态 | 说明 |
|---:|---|---|---|
| 1 | 接线交叉 | PA2(TX)↔ESP **RX**，PA3(RX)↔ESP **TX** | 最常见的接反；TX 接 TX 永远收不到 |
| 2 | 共地 | STM32 GND ↔ ESP GND | 不共地电平无参考 |
| 3 | 供电 | 3.3V、**≥250mA** | 电流不足会反复重启；**不要用 MCU 的 3.3V 引脚直供** |
| 4 | EN / CH_PD | 拉**高**（10k 上拉） | 拉低=芯片关断，不响应 AT |
| 5 | GPIO0 | 拉**高**或悬空 | 拉低=下载模式，不跑 AT 固件 |
| 6 | GPIO15 | 拉**低** | 拉高会进 SDIO 启动模式 |
| 7 | 波特率 | 双方均 **115200** | STM32 侧已确认 115200；若模块被改过（如 921600）需先 `AT+UART_DEF=115200,8,1,0,0` 改回 |

### 状态
- [x] 根因定位（ReceiveToIdle 超时后 RxState 卡 BUSY_RX，放大成永久故障）
- [x] `esp01s.c`：`esp01s_rx_restart_clean()` + `ESP01S_Init()` 改调 + 日志加状态码
- [x] `esp01s.h`：`ESP01S_UART_BAUD` 921600 → 115200（与实际一致）
- [x] 握手失败加字节级诊断：`rx 0 bytes -> module SILENT` / `rx N bytes [hex] -> garbage=baud mismatch`（`LOG_E`，不受 DBG_LOG_NET 门控）
- [ ] 用户 Rebuild + 复现验证（每轮应出现上述诊断日志，而不是 `RX start failed` 死循环）
- [x] 用户回环自测完成：**rx 0 bytes** → 问题锁定在 MCU 侧（真根因见 E32）
- [x] E31 修复验证通过：日志中 `AT handshake` 每轮重现，不再有 `RX start failed` 死循环
- [ ] 用户在 `.ioc` 使能 USART2 NVIC 后复测（E32 的修复动作）

---

## 事件 E32：USART2 的 NVIC 中断从未使能 → 永远收不到任何字节（2026-08-31）

> 这是 **E31 里「未决：首次 AT 无回响」的真根因**。E31 修好后重连能反复发生，
> 但每次握手依然 `rx 0 bytes`，靠 PA2/PA3 回环自测把问题锁定到了 MCU 侧。

### 现象
- ESP-01S 接与不接，握手都失败：`no AT echo: rx 0 bytes -> module SILENT`；
- **断开模块、短接 PA2↔PA3 做回环自测，仍是 `rx 0 bytes`** —— 自己发自己收都收不到；
- 引脚 / 波特率 / 供电 / EN / GPIO0 逐项查过，全部正常。

### 根因（配置缺失，非代码 bug）
CubeMX **没有勾选 USART2 的 "global interrupt"**：

| 串口 | `HAL_NVIC_SetPriority` + `EnableIRQ` | `IRQHandler` |
|---|:--:|:--:|
| UART4（屏） | ✅ `SetPriority(6,0)` | ✅ |
| USART1（LOG） | ✅ `SetPriority(8,0)` | ✅ |
| USART6（ESP32-S3） | ✅ `SetPriority(5,0)` | ✅ |
| **USART2（ESP-01S）** | **❌ 完全没有** | **❌ `stm32h7xx_it.c` 里不存在** |

于是 `HAL_UARTEx_ReceiveToIdle_IT()` **照常返回 `HAL_OK`**（它只置位外设的 RXNE/IDLE 中断使能位），
但 **NVIC 层没开 → CPU 从不响应 USART2 中断 → ISR 永不触发 → 一个字节都收不到**。

> **为什么极易误判**：`ReceiveToIdle_IT` 返回成功、引脚配置正确（`PA2=TX / PA3=RX / AF7`）、
> `Mode = UART_MODE_TX_RX`、波特率 115200 全对 —— 表面上"一切都对"，
> 于是很容易一路怀疑到接线、供电、模块损坏上去。
> **判据就是回环自测**：TX 是轮询发送（不依赖中断），RX 靠中断 ——
> 短接后仍 0 字节，说明断的是中断路径，不是物理链路。

### 修复（必须走 CubeMX，符合「配置单一源 = .ioc」）
1. 打开 `AI_deploy.ioc` → Pinout & Configuration → Connectivity → **USART2**；
2. **NVIC Settings** 标签 → 勾选 **Enabled**（USART2 global interrupt）；
3. 设 **Preemption Priority = 7**（原因见下方 ⚠️）；
4. Generate Code → 重新编译烧录。

生成后应能看到两处新增：
- `Core/Src/usart.c`：`HAL_NVIC_SetPriority(USART2_IRQn, 7, 0); HAL_NVIC_EnableIRQ(USART2_IRQn);`
- `Core/Src/stm32h7xx_it.c`：出现 `USART2_IRQHandler()` 且内部调用 `HAL_UART_IRQHandler(&huart2);`

### ⚠️ 优先级的坑（FreeRTOS）
`ESP01S_UART_RxCallback()` 里调用了 **`osThreadFlagsSet()`（FromISR API）**，
故 USART2 的抢占优先级**数值必须 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`**（本工程为 **5**）。
若设成 0~4，运行时会触发 FreeRTOS `configASSERT` 失败（整机卡死）。
本工程现状：USART6=5、UART4=6、USART1=8 —— **选 7 安全**。

### 防御性诊断（代码已加）
`esp01s.c` `ESP01S_Init()` 在 arm 成功后检查 NVIC 使能位，未使能则报：
```
[E][NET] USART2 NVIC NOT enabled -> RX ISR never fires (fix: .ioc enable USART2 global interrupt)
```
即使以后 CubeMX 重新生成又把这项丢了，也能**一条日志定位**，不用再走一遍回环自测。

### 验证步骤
1. 勾选后 Generate Code，**确认 `usart.c` / `stm32h7xx_it.c` 出现上述两处新增**；
2. **先不接模块**、短接 PA2↔PA3：应看到 `rx 4 bytes [41 54 0D 0A]`（即自己发的 `AT\r\n`）—— 回环通了；
3. 拆掉短接线、接回 ESP-01S：应看到 `WiFi connected` → `MQTT CONNACK OK` → `subscribed: .../property/set`。

### 状态
- [x] 根因定位（USART2 NVIC 未使能 + 无 IRQHandler）
- [x] 加防御性诊断日志（`NVIC->ISER` 检查）
- [x] 用户在 `.ioc` 勾选并重新生成 —— **已验证**：WiFi 握手全程通过（`CWMODE=1`→`ATE0`→`CWJAP`→`WiFi connected`），
      TCP 透传也建立成功（`CIPSTART`→`CIPMODE=1`→`CIPSEND`→`TCP transparent link established`）
- [x] 接模块实测：进展到 MQTT 层，卡在 E33

---

## 事件 E33：MQTT CONNECT 报文缓冲不足 + PUBLISH 缓冲无长度检查（2026-08-31）

### 现象（E32 修好后暴露的下一层）
```
[6911][I][NET] WiFi connected
[7411][I][NET] TCP transparent link established
[7411][D][NET] MQTT build CONNECT failed      ← 卡在这
[7411][W][NET] connect failed (2/6), retry in 20000 ms
```

### 根因 1：CONNECT 报文 217 B > 缓冲 160 B
`mqtt_build_connect()` 末尾有 `if ((hl + paylen) > max) return -1;`，
而调用点给的是 `uint8_t pkt[160]` —— **阿里云凭据很长**，实际需要的长度：

| 字段 | 长度 |
|---|---:|
| clientId `k1tjfOebU45.STM32_dev\|securemode=2,signmethod=hmacsha256,timestamp=...\|` | 81 |
| username `STM32_dev&k1tjfOebU45` | 21 |
| password `signmethod=hmacsha1,timestamp=1,<64 hex>` | 96 |
| payload = 2+81+2+21+2+96 | **204** |
| 变长头 = 1(type)+2(varint)+2+4+1+1+2 | **13** |
| **整包** | **217** |

217 > 160 → build 直接失败。**凭据越长越容易踩**；换用短 clientId 的产品不会暴露。

### 根因 2（顺带发现的 P0）：`ESP01S_MQTT_Pub` 无长度检查 → 会越界写栈
旧代码 `uint8_t pkt[320]`，随后直接 `memcpy` topic 与 payload，**全程没有长度校验**：
物模型属性上报 JSON 可达 300+ B，加 52 B 的 topic 就会**越过 320 B 往栈上写** → HardFault 或栈破坏。
E33 修好、链路一通，这个隐患会在**第一次属性上报（上线 0 秒后）立刻触发**。

### 修复（已落地）
- `ESP01S_MQTT_Connect()`：`pkt[160]` → **`static uint8_t pkt[256]`**；
  失败时打印 `cid/ usr/ pwd` 长度与估算需求，凭据再变长也能一眼看出；
- `ESP01S_MQTT_Pub()`：`pkt[320]` → **`static uint8_t pkt[512]`**，并**补长度校验**
  （`need = 1 + 4 + rem`，超限则丢弃并 `LOG_E`，绝不越界写）；
- 两者都用 **static 而非栈上**：`Task_Network` 栈仅 `512*4 = 2048 B`，不能压几百字节的帧缓冲。

### 余量核对
属性上报最坏情况：JSON 448（`ESP01S_JSON_MAX`）+ topic 52 + 2 + varint 2 + 1 = **505 < 512** ✅

### 状态
- [x] 根因定位（CONNECT 217 B > 160 B；Pub 无长度检查 P0）
- [x] 两处缓冲改为 static 并加校验/诊断日志
- [ ] 用户 Rebuild + 实测：期望 `MQTT CONNACK OK` → `subscribed: .../property/set`
- [ ] 长连观察：属性上报（5s 一帧）是否正常，不再 HardFault

---

## 事件 E34：STM32 复位但 ESP-01S 没断电 → 模块卡在透传模式，AT 全部失效（2026-08-31）

### 现象（会"时好时坏"，极易误判）
- 上一轮实测已跑通：`WiFi connected` → `TCP transparent link established`；
- 重新 Rebuild 烧录后再上电，**又变回 `no AT echo: rx 0 bytes`**；
- 但**没有** `USART2 NVIC NOT enabled` 警告 —— 说明 E32 的中断修复仍然有效，RX 通路是好的。

即：**硬件没动、配置没动，只是 MCU 复位了一次，模块就"哑"了。**

### 根因（推断，待用户断电验证）
上次成功连接时，代码执行了 `AT+CIPMODE=1` + `AT+CIPSEND`，**模块已进入透传模式**。
此后 STM32 复位 / 重烧，但 **ESP-01S 一直没断电** —— 模块**仍停在透传模式**。

透传模式下，模块把串口收到的**一切**当作 TCP payload 原样转发，**不再解析 AT 命令**。
于是发 `AT\r\n` 时它一声不吭、直接转发出去 —— 表现为：

- 握手恒超时、`rx 0 bytes`；
- 症状与"接线断开 / 模块没供电 / EN 没拉高"**完全一致**，
  但接线供电其实都是好的 —— **极易把人带回 E31 那 7 项硬件排查，白查一遍**。

> 判据：**给 ESP-01S 单独断电再上电**（不是复位 STM32）。若立刻恢复，
> 即证实是透传残留；若仍 0 字节，才回到硬件排查。

### 修复（已落地，`ESP01S_Init()`）
上电后**首次** Init 先发退出透传序列，再走正常 AT 流程：

```c
if (s_first_init) {
    static const uint8_t exit_tp[3] = { '+', '+', '+' };
    s_first_init = 0U;
    osDelay(1100U);                                  /* "+++" 前需 >1s 线路静默 */
    (void)HAL_UART_Transmit(&huart2, exit_tp, 3U, 100U);
    osDelay(1200U);                                  /* 等模块回到 AT 模式 */
    (void)ESP01S_SendAT("AT+CIPCLOSE", NULL, 500U);  /* 清残留 TCP，失败可忽略 */
}
```

要点：
- `+++` 必须**单独一帧**、**不带 CRLF**、**前后各 >1s 静默** —— 三条缺一不可，否则模块仍当数据处理；
- 只做**一次**（`s_first_init` 标志）：之后模块已在 AT 模式，重连时不必每次多等 2.3 s；
- 紧跟 `AT+CIPCLOSE`：透传退出后 TCP 链路往往还活着，不清掉后面 `AT+CIPSTART` 会报 `ALREADY CONNECTED`；
- 若模块本就在 AT 模式，`+++` 不是合法命令，模块至多回个 ERROR，随后被 `ring_drain()` 丢弃，**无副作用**。

### 验证步骤
1. Rebuild 烧录（**不要**给模块断电，保留透传残留状态）→ 应直接握手成功，日志出现
   `sent transparent-exit (+++) + CIPCLOSE on first init`；
2. 对照实验：手动断开模块 VCC 再接回 → 也应正常（此时 `+++` 是多余但无害的）；
3. 连上后观察 `MQTT CONNACK OK` → `subscribed: .../property/set`。

### 状态
- [x] **已验证生效**：日志出现 `sent transparent-exit (+++) + CIPCLOSE`，
      随后 `AT handshake`→`CWMODE=1`→`ATE0`→`CWJAP`→`WiFi connected` 全部通过 —— 证实为透传残留
- [x] 修复升级为「每次 Init 先探测 AT（300ms）、静默才发 `+++`」：
      只做一次是不够的 —— CONNACK 超时等失败路径会带着透传状态回到 Init，每轮重连都可能遇上
- [ ] 连上后观察属性上报稳定性（进展到 E35）

---

## 事件 E35：CONNACK 返回码未参与判定 + 阿里云三元组三处不一致（2026-08-31）

### 现象
```
[6787][I][NET] TCP transparent link established
[6787][D][NET] MQTT CONNECT send (wait CONNACK)
[5871][E][NET] MQTT CONNACK timeout        ← broker 什么都没回
```
CONNECT 已发出（E33 修复生效），但 8 s 内**既没有 CONNACK、也没有拒绝码**。

### 根因 1（代码 bug，会让凭据错误伪装成成功）
`ESP01S_MQTT_Connect()` 只用 `ring_find_bin({0x20,0x02}, 2)` 判断"收到 CONNACK"，
**从未检查第 4 字节的返回码**：

```c
uint8_t rc = esp01s_connack_rc();      /* 只在 DBG_LOG_NET 里打了个日志 */
LOG_I("NET", "MQTT CONNACK OK");       /* 然后无条件 return ESP01S_OK —— rc=4/5 也算成功！ */
```

后果：broker 回 `rc=4（用户名密码错）` 或 `rc=5（未授权）` 时，代码照样报"MQTT CONNACK OK"，
之后所有 PUBLISH 静默失败。**凭据问题会被彻底掩盖**，表现为"连上了但云端没数据"。

修复：`rc != 0` 直接判失败，并按码给出人话解释：

| rc | 含义 |
|:--:|---|
| 0 | accepted |
| 1 | unacceptable protocol version |
| 2 | identifier rejected（clientId 格式） |
| 3 | server unavailable |
| **4** | **bad username or password（signmethod / timestamp 不匹配）** |
| **5** | **not authorized（三元组或地域不对）** |

另补：CONNACK 超时时把 ring 内容按 ASCII 打出来，区分
「broker 完全没回」与「回了但格式不对」（TCP 被断通常能看到 `CLOSED`）。

### 根因 2（配置问题，当前 CONNACK timeout 的真凶）
`esp01s_config_local.h` 的三元组**三处不一致**（只查元数据，未触碰密钥内容）：

| 项 | clientId | password | 问题 |
|---|---|---|---|
| `signmethod` | `hmacsha256` | `hmacsha1` | ❌ 两边必须一致 |
| `timestamp` | `1788096320569` | `1` | ❌ 必须同一个值，否则签名校验失败 |
| hex 长度 | — | **64** | 64 = SHA-256 输出（SHA-1 是 40）→ 签名实为 sha256，标签却写 sha1 |

**另有第三个坑**：clientId 写的是 **`securemode=2`**，而代码连的是 **1883（明文 TCP）**。
阿里云约定：`securemode=2` = TLS 直连（**8883**），`securemode=3` = TCP 直连（1883）。
ESP-01S 通用 AT 固件**不支持 TLS**，所以必须走 **1883 + `securemode=3`**。

以上任何一条都会让 broker 直接断开 TCP、不回任何 CONNACK —— 与本次超时现象完全吻合。

### 修复（需用户重新生成三元组）
正确格式（三处必须对齐）：
```
clientId : k1tjfOebU45.STM32_dev|securemode=3,signmethod=hmacsha256,timestamp=<T>|
username : STM32_dev&k1tjfOebU45
password : signmethod=hmacsha256,timestamp=<T>,<hmacsha256 签名的 64 hex>
端口     : 1883
```
`<T>` 为**同一个**毫秒时间戳；签名须由该 `<T>` 参与计算。
建议用阿里云官方 MQTT 签名工具生成，而不是手改其中一项。

### 关于旧版 AT 代码的对比（用户提供的可用参考）
旧版用的是 ESP8266 **带 MQTT 的 AT 固件**，靠
`AT+MQTTUSERCFG` / `AT+MQTTCLIENTID` / `AT+MQTTCONN` 由模块内部完成 MQTT；
本项目是**手写 MQTT 报文 + 透传**（通用 AT 固件即可）。
两者**对凭据内容的要求完全相同** —— 差别只在"谁来组 CONNECT 包"（模块 vs 你的代码）。
因此旧版能通不代表当前凭据格式可照抄，仍需按上面三处对齐重新生成。

### 状态
- [x] CONNACK rc 判定修复 + rc 人话映射 + 超时打印 ring 内容
- [x] Init 改为「探测式」透传退出（每轮都生效）
- [x] 用户已给控制台「MQTT连接参数」：`clientId`/`username` 与配置一致；**注意控制台给的就是 `securemode=2` + `port=1883`**（非本事件先前推测的 `securemode=3`）—— 以控制台为准，配置保持 `securemode=2`
- [ ] 真正残留阻点 = 配置密码带了 `signmethod=/timestamp=` 前缀，而 broker 期望 RAW 64-hex → 见 **事件 E36**

---

## 事件 E36：配置密码带 `signmethod=/timestamp=` 前缀，broker 期望 RAW 64-hex → CONNACK 静默超时（2026-08-31）

### 现象
```
[5291][I][NET] TCP transparent link established
[5291][D][NET] MQTT CONNECT send (wait CONNACK)
[4380][E][NET] CONNACK timeout: rx 0 bytes (broker silent / CONNECT not delivered?)
```
TCP 透传已建立、CONNECT 已发出，但 8 s 内 broker **一个字节都没回**（不是 rc=4/5 拒绝，是彻底静默）。

### 根因（配置字符串格式错配）
控制台「MQTT连接参数」给的密码是 **RAW 64-hex 签名**：
`af695fc08f44892781a7507068ef5ef1516706a743fbdf505ac5257ddf3ae2a5`
而 `esp01s_config_local.h` 曾写成带前缀：
`signmethod=hmacsha256,timestamp=1788138650959,af695fc08f…ae2a5`（**108 字符**）。

阿里云 broker 用 password 字段**直接做 HMAC 校验比对**，**期望 RAW 签名（64 字符）**。
带 `signmethod=…,timestamp=…,` 前缀后，broker 算出的期望签名（64）≠ 收到的字符串（108）→ 校验失败 →
**broker 不等 CONNACK、直接静默丢 TCP 连接** → 表现为 `rx 0 bytes` 超时。

> 与 E35 根因2 的关系：E35 怀疑的是 signmethod/timestamp/securemode 三处「取值」不一致；
> 实际落到配置上时，clientId/username 已与控制台一致，**唯一偏差是「密码被加了前缀」**。故单列 E36，避免与取值问题混淆。

### 修复（已落地，esp01s_config_local.h）
`ESP01S_MQTT_PASSWORD` 改为控制台给的 RAW 64-hex，去掉前缀；并加注释警示「勿加前缀」。

### 余量核对
RAW 密码 64 字符 → CONNECT payload = 2+81+2+21+2+64 = 172 B，整包 ≈ 186 B < 256 B 缓冲（E33 已扩）。✅

### 状态
- [x] 根因定位（密码前缀导致 broker 静默丢）
- [x] 配置改正为 RAW 64-hex + 注释警示
- [ ] 用户 Rebuild + 实测：期望 `MQTT CONNACK OK (rc=0)` → `subscribed: .../property/set`
- [ ] 长连观察属性上报（5 s 一帧）是否稳定，不再 HardFault（E33 P0）

### 备注：securemode=2 vs 3 的兜底判据
若改完密码仍 `rx 0 bytes` 超时，且 clientId 是 `securemode=2` + 连 1883，下一步试把 clientId 改为 `securemode=3`（仅改此一处，password 不动，签名与 securemode 无关）：
`k1tjfOebU45.STM32_dev|securemode=3,signmethod=hmacsha256,timestamp=1788138650959|`
ESP-01S 不支持 TLS，必须走 1883 明文；若 broker 对 `securemode=2` 强制 TLS 端口，则需此改。
（当前控制台给的就是 `securemode=2`，故先按控制台值实测，失败再试 3。）

---

## 事件 E37：`mqtt_build_connect` 剩余长度少算 1 字节 → CONNECT 畸形，broker 静默断链（2026-08-31）

### 现象（用户实测，与 E36 完全相同：TCP 透传建立后立即 `CONNACK timeout: rx 0 bytes`）
```
[4590][I][NET] WiFi connected
[4700][I][NET] TCP transparent link established
[4700][D][NET] MQTT CONNECT send (wait CONNACK)
[3781][E][NET] CONNACK timeout: rx 0 bytes (broker silent / CONNECT not delivered?)
```
用户坚持「配置完全正确」，且 E36 已排除前缀问题 → 说明问题**不在凭据内容，而在 CONNECT 报文本身**。

### 根因（代码 bug，长期伪装成「凭据/网络问题」）
`mqtt_build_connect()` 计算 MQTT 剩余长度（Remaining Length）时：

```c
uint32_t rem = 9U + paylen;   /* 注释写 "Variable header 9 bytes" —— 实际是 10 */
```

MQTT CONNECT 的**可变头固定 10 字节**：
`协议名长度前缀 2B` + `"MQTT" 4B` + `协议级别 1B` + `连接标志 1B` + `保活 2B` = **10B**。
代码写成了 **9**，于是：

- 报文里 `rem` 字段 = 213，但可变头+负载实际 = **214 字节**；
- broker 按 `rem=213` 去读，读到**密码最后一个字符**就认为包结束 → 把整包判为畸形；
- 结果：**broker 不发 CONNACK、直接静默断 TCP** → 表现为 `rx 0 bytes` 超时。

> ⚠️ **这个 bug 与密码对不对完全无关**：它从你开始手写 MQTT 报文起就一直在，
> 所以无论凭据怎么填都连不上。你之前「旧版成功」是 ESP8266 模块内部组 MQTT 包，根本没走这段代码。
> 它也解释了为什么 E31–E36 一路修到 MQTT 层，却始终卡在 `CONNACK timeout: rx 0 bytes`——前面每层修好都会暴露下一层，而这一层是**根**。

### 修复（已落地，esp01s.c）
- `rem = 9U + paylen` → **`rem = 10U + paylen`**；溢出检查同步改为 `10U + paylen`；
- 配套新增调试（用户明确要求的「AT↔阿里云交互详细化」）：
  1. `SendAT`：每条 AT 指令先打 `AT> <cmd>`，再把回包原样打 `AT< [OK|FAIL] <resp>`（DBG_LOG_NET）；
  2. `MQTT_Connect`：`CONNECT build: cid/ usr/ pwd/ total` + **整包 hex 转储**（`esp01s_hexdump`），可逐字节核对凭据边界与剩余长度字段；
  3. CONNACK 等待循环：把**新到达的字节实时 hex 打印**（`RX +N B`），超时则把 ring 全盘 hex 转储——让你直接看到 broker 到底回了 CONNACK / CLOSED / ERROR，还是真的什么都没回。

### 验证步骤
1. Rebuild 0E/0W + 烧录；
2. 期望日志顺序：
   ```
   AT> AT+CIPSTART=...        AT< [OK] ...
   AT> AT+CIPMODE=1           AT< [OK]
   AT> AT+CIPSEND             AT< [OK]
   [I] TCP transparent link established
   [D] CONNECT build: cid=81 usr=21 pwd=64 total=217 B (buf=256)
   [D] [CONN] 0000: 10 D6 01 00 04 4D 51 54 54 04 C2 00 3C ...  ← 第 2~3 字节应为 0xD6 01（rem=214）
   [I] MQTT CONNACK OK (rc=0)           ← 修复后应直接连上
   ```
   - hex 转储里第 2 字节应为 `0xD6`（214 的变长编码：`214 = 0xD6` 带延续位 = `D6 01`），而非旧的 `0xD5 01`（213）。
3. 若仍失败：
   - 出现 `CONNACK rejected: rc=4/5` → 凭据问题（此时密码 `ad15fc7e…` 可能与 clientId 的 `timestamp=1788141913707` 不对应，需按控制台**重新一键复制**整套三元组）；
   - 出现 `RX +4 B` 且含 `CLOSED` → TCP 被 broker 断开，查 securemode=2 vs 1883（E35/E36 备注的兜底）；
   - 仍 `rx 0 bytes` → 物理链路/透传 TX 问题，回到 E31 硬件排查。

### 状态
- [x] 根因定位（剩余长度少 1B → 畸形 CONNECT）
- [x] `rem = 10 + paylen` 修复 + 溢出检查同步
- [x] AT 请求/响应全量日志 + CONNECT 整包 hex 转储 + CONNACK 实时 RX 嗅探
- [ ] 用户 Rebuild + 实测：期望 `MQTT CONNACK OK (rc=0)` → `subscribed: .../property/set`
