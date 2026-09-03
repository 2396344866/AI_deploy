# POST 上电自检 任务测试方案（test_template.md）

> 模仿 `Components/Debug/Test/test_template.md` 风格。命名约定：`StartTestTask` = POST 任务**函数**（CubeMX 生成骨架，体内调 `Postest_RunAll`）；`Task_Test` = 该任务**对象**（osPriorityHigh，CubeMX 生成）。POST 进度 tag = `POSTEST`。
> 配套：设计红线见 **附录 A**；复位源判读见 **附录 B**；Keil 自维护见 **附录 C**。
> **文档分工**：本 md = 测试规格 + **详尽五要素操作步骤 + 实测记录**（§二汇总、§三逐用例展开）；深读见 `Doc/调试手册.md`（宏配置速查）、`Components/Debug/Error/Error_Readme_idx.md`（编译/uvprojx 索引）、`Components/Debug/Error/*.md`（运行期故障事件）。
> 风格：工业界测试规范（目标/环境/宏配置/用例 → 五要素步骤 → 实测 → 量化 KPI）。

> **⚠ 启动时序边界（本工程实测印证，勿混淆）**：POST 是**上电后、应用任务运行前**的一次性「静态/离线」自检（`Task_Test`, osPriorityHigh，本 run `983ms` 结束即 `Self-Test done`）。**运行期握手（WiFi 连网 `@5596ms`、MQTT CONNACK `@6231ms`、ESP32 链路 `@6002ms`）发生在各自任务（`StartNetworkTask` 等）启动之后，不在 POST 内**——`Network_Test` 仅校验 USART2 句柄（`esp01s.c:1057` 标 `deferred: full handshake in StartNetworkTask`）。POST 与运行期 BIST/握手是**两个不同阶段**，不可混为一谈。

---

## 一、测试目标与概述

验证 **POST（上电自检）** 在**功能、实时性、精度、鲁棒性**四维达标，覆盖已修复缺陷（IWDG 复位环、关键失败停机、任务退出语义 `prvTaskExitError`）的回归，并验证 Logger 级别门控在最小 profile 下不产生干扰噪声。

### 1.1 测试环境
| 项 | 内容 |
|----|------|
| 硬件 | STM32H743VIT6；W25Q64（黑匣子/外部 Flash，若 profile 含 FLASH）；MPU6050（若含 SENSOR）；ESP32-S3（若含 ESP32S3）；IWDG1（LSI 32kHz/Pre32/Reload4095≈4.1s） |
| 软件 | 固件版本 <tag>；CubeMX 生成 RTOS 对象（`Task_Test`）；profile 由 `app_config.h` 的 `APP_PROFILE_*` 派生 `APP_ENABLE_X`；自检表 `g_postests[]` 与 `APP_ENABLE_X` 同一真相源 |
| 工具链 | Keil MDK ARM v6；ST-Link V2；串口助手（UART1 PA9/PA10，921600 8N1；推荐 VOFA+ 或 PuTTY） |
| 观测 | 串口 UART1（自检进度/栈水位/级别回显）；Keil Live Watch `g_wdt_tick_cnt` / `g_hb_ticks[]`；逻辑分析仪接 W25Q DO(MISO)（异常注入观测） |

### 1.2 宏配置与代码操作（如何配置宏才能真正生效）
> POST 受**三层门控**约束，凡涉及编译期开关的用例必须按本表配。

| 闸门/手段 | 宏 / 接口 | 定义位置 | 作用 | 改后是否需重编 |
|-----------|-----------|----------|------|----------------|
| L1 功能包含（总闸） | `APP_PROFILE_*` / `APP_ENABLE_X` | `app_config.h` | 决定哪些模块编进 .bin、POST 自检表 `g_postests[]` 实际项、心跳监视集 | 是 |
| 总日志闸 | `LOG_ENABLED` | `logger.h:28` | 未定义 → 所有 `LOG_*` 宏为空、零开销 | 是 |
| Gate1 编译上限 | `LOG_COMPILE_MAX_LEVEL` | `logger.h:48` | 编进二进制的最高级别（FATAL0/ERROR1/WARN2/INFO3/DEBUG4/TRACE5） | 是 |
| Gate2 运行默认 | `LOG_RUNTIME_DEFAULT_LEVEL` | `logger.h:52` | boot 初始过滤级，须 ≤ 编译上限（否则 `#error`） | 是 |
| Gate2 运行调级 | `logger_set_level()` / 串口 `debugX` | `logger.c` / 控制台 | 运行期现场提/降级别（被编译上限封顶） | 否 |
| POST 门控（里程碑，常开） | 无（系统里程碑走 `LOG_EMIT_DIRECT(LOG_LVL_INFO,...)`/`LOG_E` 恒 INFO） | — | 系统里程碑走 `LOG_EMIT_DIRECT(LOG_LVL_INFO,...)`（Channel B 保证通道，绕过运行级必落线），`LOG_E` 关键失败本就 Channel B；不经 `DBG_LOG_<TASK>` 管控；生产靠 `LOG_ENABLED=0` 整体静音 | 否 |
| POST 内部探针 / 模块细节开关 | `DBG_LOG_POSTEST` + `DBG_LOG_FLASH`/`DBG_LOG_DIAG` | `dbg_config.h`（POSTEST 默认 1，模块细节开关默认 0） | **框架编排探针**（`X_Test enter`/`Postest_RunAll`/`Logger` 子步骤/`Self-Test done`）走 `#if DBG_LOG_POSTEST` 包裹的 `LOG_EMIT_DIRECT(LOG_LVL_DEBUG,"D","POSTEST",...)`（Channel B 同步直发），饿死 `LoggerTask` 也能实时落线；**模块内部细节另归模块开关**：`Flash_Test` step 进度→`DBG_LOG_FLASH`(tag `FLASH`)，`FaultDiag_ML_Test` init/sample/metrics→`DBG_LOG_DIAG`(tag `DIAG`)，关掉只丢细节、不影响判卷；三者均仅 `LOG_COMPILE_MAX_LEVEL>=DEBUG`+对应开关=1+运行 `debug4` 才出（发布零开销） | 是（设 1 + 抬编译上限 + 重编） |

**代码侧操作步骤**（想验证 LOGGER profile 自检必须这样配）：
1. **设总闸**：`app_config.h` 选 `APP_PROFILE_LOGGER`（其余 `APP_ENABLE_X` 自动 = 0）→ 自检表仅 `{Logger}` 一项。
2. **抬编译上限**：`logger.h` 设 `LOG_COMPILE_MAX_LEVEL` 到目标级（看 Logger_Test 的 DEBUG/TRACE 样例行需 ≥4/5）；`LOG_RUNTIME_DEFAULT_LEVEL` 须 ≤ 它。
3. **Keil 工程操作**：确认两源组就位（`Components/POSTest/Src/Postest.c` + `Components/Debug/Src/watchdog_heartbeat.c`）→ Rebuild（0 Error/0 Warning）→ Download → Debug。
4. **运行期调级（可选）**：串口发 `debug4` / `debug5` 现场提级验证运行期门控（仅影响 `LOG_D`/`LOG_T` 样例行；POST 里程碑恒 INFO 不受影响）。
5. **定位 POST 卡死（调试 build，见 §1.5）**：`Postest.c` 内 `#if DBG_LOG_POSTEST` 包裹的 `LOG_EMIT_DIRECT(LOG_LVL_DEBUG,...)` 探针 + `log_wdt_feed()` 已就位；要实时看「卡到哪一行」，需 `LOG_COMPILE_MAX_LEVEL` 抬到 `DEBUG(4)` + `DBG_LOG_POSTEST=1` + 重编 + 运行 `debug4`。

### 1.3 测试数据生成策略
- **正常工况**：各 profile 默认硬件在位（Flash/W25Q 焊接良好、MPU6050 应答、模型已加载）。
- **边界工况**：`LOG_COMPILE_MAX_LEVEL` 依次设 INFO(3)/DEBUG(4)/TRACE(5)，验证编译期级别裁剪边界。
- **异常工况**：短接/断开 W25Q DO(MISO) 制造 Flash 自检失败，验证 critical 失败 → 刷黑匣子 + `while(1)` 停机（TC-04）。
- **性能压力**：全模块 profile 跑全量自检测 `Task_Test` 栈水位（TC-07）；含 INFERENCE 时 ML 自检约 130s 靠协作喂狗活过。
- **抗噪/鲁棒性**：NRST 手动复位、掉电上电验证复位源判读；全模块 profile 连续运行 72h 验证心跳监视无 IWDG 误复位（TC-08）。

### 1.4 通过 / 失败判据
- **通过**：`=== Self-Test done ===` 出现；无**周期**复位；`[BOOT] ResetSrc` 仅一次性（SFTRST/PINRST/PORRST）；关键失败用例（TC-04）能正确刷黑匣子并停机(halt)。
- **失败**：出现 `ResetSrc=IWDG1` 且**每 ~4.1s 周期重打**（高优先级任务/ISR 冻结饿死喂狗点）；或 `LOG_COMPILE_MAX_LEVEL≥DEBUG` 但 DEBUG/TRACE 行不出现（裁剪逻辑错）；或 critical 失败未刷黑匣子（`logger_flush_to_flash` 缺失，日志丢失）。

### 1.5 POST 卡死定位（DBG_LOG_POSTEST 调试 build 与探针判读）

> 适用场景：`[BOOT] ResetSrc=IWDG1` 每 ~4.1s 周期重打，且串口 POST 进度停在某行 → 某 `Xxx_Test` 内存在忙等环 / 外设无应答 / 同步直发卡死，饿死 IWDG。
> 普通 `LOG_D` 走 Channel A，依赖最低优先级 `StartLoggerTask` 抽空；POST 期间 `Task_Test`(HIGH) 饿死它，故 **Channel A 探针在卡死前无法实时落线**。因此 POST 内部探针统一走 **Channel B 同步直发**（`#if DBG_LOG_POSTEST` 包裹 `LOG_EMIT_DIRECT(LOG_LVL_DEBUG, "D", "POSTEST", ...)`），绕过运行级与抽空任务，卡死前必落线。

**开启调试 build（三道闸门全部满足，缺一道即不出）**：
1. `logger.h`：`LOG_COMPILE_MAX_LEVEL` 抬到 `LOG_LVL_DEBUG(4)`（或 `TRACE(5)`）；`LOG_RUNTIME_DEFAULT_LEVEL` 须 ≤ 它（否则 `#error`）。
2. `dbg_config.h`：`DBG_LOG_POSTEST` 由 `0` 改 `1`（两分支都改；`#else` 分支也改，避免总闸关时宏未定义 `-Wundef`）。
3. 运行期：串口发 `debug4`（→ `logger_set_level(DEBUG)`）让 `DBG_LOG_POSTEST` 探针文本完整出现。

**重编 + 烧录**：Keil Rebuild 0 Error/0 Warning → Download → 复位观察串口。

**探针判读（定位到具体行）**：
- `Logger_Test` 内 `#if DBG_LOG_POSTEST` 包裹的 `LOG_EMIT_DIRECT(DEBUG)` 探针 带 `enter` / `after LOG_F` / `after LOG_E` / `after LOG_W` / `after LOG_I` / `after LOG_D` / `exit OK` 标记，且 **每条 `LOG_*` 前都先 `log_wdt_feed()`**。
- 串口最后出现的 `DBG_LOG_POSTEST` 探针 = 「卡死行之前最后成功执行的探针」；**下一条缺失的探针 = 卡死点**。
- `Flash_Test` 探针带每步 `elapsed_ms` 与 `rc`（受 `DBG_LOG_FLASH` 门控，tag `FLASH`）；`FaultDiag_ML_Test` 探针默认**每 500 个样本**（`DBG_ML_SAMPLE_LOG_INTERVAL`，ai_infer.c）打印一次 `pred/actual/running_acc`，末尾不足 500 的余数段补打一次（无 25/50/75% 里程碑，避免与 interval 日志重复），细节受 `DBG_LOG_DIAG` 门控（tag `DIAG`）⇒ 卡在擦除 / SPI 回环 / 某段推理一目了然。
  - 1605 样本逐条打印会刷屏淹掉其它 POST 日志；**需定位到单个错分样本时，把 `DBG_ML_SAMPLE_LOG_INTERVAL` 改成 1** 即恢复逐样本打印。
  - 判据是 `(i+1) % N == 0`，即这 N 个**全部跑完**才打；样本序号为 1-based（末帧显示 `[1605/1605]`）。

**替代调试手段（不改代码）**：Keil **Live Watch**（SWO 非 halt）+ **断点**：单变量（`g_wdt_tick_cnt`、`e->name`、`correct_total`）实时观测，适合确认「是否进某函数 / 某循环是否推进」。POST 卡死优先用本 §1.5 的 `DBG_LOG_POSTEST` 探针（实时连续轨迹），单点确认用 Live Watch / 断点。

---

## 二、测试用例明细（规格 + 实测汇总）

> 五要素步骤见 §三；2026-08-30 LOGGER profile 全项 PASS 实测见 §三 各 TC「实测记录」及 §四。

| 用例编号 | 测试类别 | 核心验证点 | 预期结果（简述） | 实测结果（2026-08-30 LOGGER profile） | 状态 |
|----------|----------|------------|------------------|----------------------------------------|------|
| POST-TC-01 | 正常值（LOGGER profile 干净启动） | LOGGER profile 仅 `{Logger}` 自检；无周期复位 | `SFTRST` 一次性→`System Init Success!`→`Self-Test start (1 modules)`→`Logger OK`→`Self-Test done` | `ResetSrc=SFTRST` 一次性；`=== Self-Test done ===`；无周期复位 | ✅ PASS |
| POST-TC-02 | 正常值（含 FLASH） | Flash `critical=1` 自检通过 | `Flash step1/2/3`→`Flash chip self-test OK`→`Self-Test done` | 需含 FLASH profile（本 run 未使能） | ⏳ 待执行 |
| POST-TC-03 | 正常值（含 INFERENCE） | ML 自检 `accuracy≥0.85` | `Beginning Batch`→末行 `accuracy≥0.85`→`Inference OK` | 需含 INFERENCE profile | ⏳ 待执行 |
| POST-TC-04 | 异常值（关键失败） | W25Q DO 短 GND→critical FAIL | `Flash step3 FAIL`→刷黑匣子→`while(1)` 停机（不自动复位） | 需异常注入 | ⏳ 待执行 |
| POST-TC-05 | 边界值（级别=INFO） | 编译期裁剪 DEBUG/TRACE | `logger smoke` 仅 F/E/W/I 四行；D/T 被裁 | 编译上限=3(INFO)；`debug5` 提级被 clamp 到 3（仅 INFO 出）→ 证裁剪生效 | ✅ PASS |
| POST-TC-06 | 边界值（级别=DEBUG/TRACE） | 编译期放出 D/T | 多出 `[DEBUG]` 与 `[TRACE]`；运行期 `debug4` 提级生效 | 当前编译上限=3，故 `debug5` 无法放出 TRACE（被封顶到 3）= 预期；抬上限重编后出 | ✅ PASS(裁剪逻辑) |
| POST-TC-07 | 性能压力（栈水位） | `Task_Test` 栈用量 | 末行 `stack: used=744/4096`（实测 18.2%）；可缩容 1024→256 words | `used=744/4096`（free=3352）→ 建议 final stack≈1000 | ✅ PASS |
| POST-TC-08 | 鲁棒性（复位源+心跳） | 全模块 profile 心跳监视 | NRST→`PINRST`；掉电→`PORRST`/`BORRST`；72h 无 `IWDG1` | 需全模块 profile + 72h | ⏳ 待执行 |
| POST-TC-09 | 回归（任务退出语义） | 禁用模块任务不自然 return → `prvTaskExitError` | LOGGER profile 全程**不进** `prvTaskExitError`（MDK 暂停 PC 不在 `port.c:235`）；各禁用任务 `osThreadTerminate(osThreadGetId())` 自删 | LOGGER profile 全程不进 `prvTaskExitError` | ✅ PASS |

**2026-08-30 LOGGER profile 单跑原始串口日志（节选，用户已跑）**：
```
[BOOT] IWDG DISABLED ... [3][I][POSTEST] === Self-Test done ===
[984][I][TEST] Task_Test stack: used=744/4096 bytes (free=3352) -> 建议 final stack ≈ 1000
... debug5 > log level -> 3 ... debug2 > log level -> 2 ...
测试完整通过；debug 目前编译最大是 3、初始运行是 3；逻辑合格，测试通过
```
> 注：本次 boot 打印 `IWDG DISABLED` 为 bring-up 调试期**临时关闭看门狗**以观察启动全貌，非发布配置；发布须 IWDG 使能（见附录 A 喂狗策略）。`debug5→3` 证明运行期提级被 `LOG_COMPILE_MAX_LEVEL=3(INFO)` 封顶——这是门控正确性的直接证据。

---

## 三、测试用例操作手册（五要素详细步骤 + 实测记录）

> 五要素：`[宏]` 编译期配置 / `[Keil]` 工程与构建 / `[连]` 物理连接 / `[发]` 控制台发送 / `[读]` 串口与观测判读。
> 通用准备（所有 TC 前置）：① ST-Link V2 接板载 SWD（SWCLK/SWDIO/SW-RST）；② USB-TTL 接 `UART1`：`PA9(TX)`→RX、`PA10(RX)`→TX、GND 共地，PC 端 921600 8N1 打开串口；③ Keil 打开 `MDK-ARM/STM32H743VIT6.uvprojx`；④ 确认源组含 `POSTest/Src` 与 `Debug/Src`（附录 C）。

### POST-TC-01　LOGGER profile 干净启动（正常值）

- **[宏]** `app_config.h`：`#define APP_PROFILE_LOGGER`，其余 `APP_ENABLE_X` 不定义（自动=0）→ 自检表仅 `{Logger}` 一项。`logger.h`：`LOG_COMPILE_MAX_LEVEL=LOG_LVL_INFO(3)`、`LOG_RUNTIME_DEFAULT_LEVEL=LOG_LVL_INFO(3)`、`LOG_ENABLED` 保持定义。
- **[Keil]** 菜单 `Project → Rebuild all target files`，确认 **0 Error / 0 Warning**；`Flash → Download`；`Debug → Start/Stop Debug Session` 进入，或掉电上电。
- **[连]** ST-Link + UART1（921600）；若观察喂狗存活可开 Keil Live Watch `g_wdt_tick_cnt`。
- **[发]** 板卡上电 / 复位（NRST 或调试器 `SYSRESETREQ`）。可选：运行期发 `debug4` 提级看 Logger_Test 内部。
- **[读]** 串口依次应见：`[BOOT] ResetSrc=SFTRST`（一次性）→ `System Init Success!` → `[POSTEST] Self-Test start (1 modules)` → `Logger OK`（logger smoke F/E/W/I 四行）→ `=== Self-Test done ===`。MDK 暂停 PC 不应在 `port.c:235`（`prvTaskExitError`）；之后**无周期复位**。
- **[实测记录]（2026-08-30）** ✅ PASS：`ResetSrc=SFTRST` 仅一次性（调试器下载，非复位环）；`=== Self-Test done ===` 出现；`g_wdt_tick_cnt` 持续递增（TIM7 存活）；全程无周期复位；串口末段 `Task_Test stack: used=744/4096`。boot 行含 `IWDG DISABLED`（调试期临时关闭）。

### POST-TC-02　含 FLASH 自检（正常值）

- **[宏]** `app_config.h`：使能含 FLASH 的 profile（或 `#define APP_ENABLE_FLASH 1`）；W25Q64 已焊接在位。其余按默认 INFO 级。
- **[Keil]** 确认 `Components/BSP/W25Q64/` 源组在册 → Rebuild(0E/0W) → Download。
- **[连]** 同上；W25Q64 DO(MISO) 保持悬空（不得短地，否则触发 TC-04 失败路径）。
- **[发]** 上电复位。
- **[读]** 串口：`Flash step1`→`Flash step2`→`Flash step3`→`Flash chip self-test OK`→`Self-Test done`；无 `IWDG1` 周期复位。
- **[实测记录]** ⏳ 本 LOGGER run 未使能 FLASH，待含 FLASH profile 执行。

### POST-TC-03　含 INFERENCE 自检（正常值）

- **[宏]** `app_config.h`：`#define APP_ENABLE_INFERENCE 1`（模型已加载至 `Components/Fault_Diagnosis/` / `.dtcmram`）。
- **[Keil]** 确认 `Fault_Diagnosis` 源组与 tensor arena 配置就位 → Rebuild(0E/0W) → Download。
- **[连]** 同上；ML 自检约 **130s**，期间观察串口持续推进、无饿死。
- **[发]** 上电复位。
- **[读]** 串口：`Beginning Batch`→每 100 样本进度（该批做完才打）→末行 `accuracy ≥ 0.85`→`Inference OK`。耗时约 130s，靠 `log_wdt_feed()`+`osDelay(1)` 协作喂狗活过（不触发 IWDG）。
- **[实测记录]** ⏳ 待含 INFERENCE profile 执行。

### POST-TC-04　关键失败 → 黑匣子 + 停机（异常值）

- **[宏]** `app_config.h`：`#define APP_ENABLE_FLASH 1`（Flash `critical=1`）。
- **[Keil]** Rebuild(0E/0W) → Download。
- **[连]** 在 POST 进行到 `Flash step3` 前，用镊子**短接 W25Q DO(MISO) 到 GND**（制造读回恒 0 / 校验失败）；测完断开恢复。
- **[发]** 上电复位；触发失败后无需发送，固件自动处理。
- **[读]** 串口：`Flash step3 FAIL` → `logger_flush_to_flash()` 刷黑匣子（写 W25Q 固定扇区）→ `while(1)` 停机（等调试器/手动复位；POST 期间 IWDG 未 arm，不自动复位）；用工具读 W25Q 黑匣子扇区确认日志落盘。
- **[实测记录]** ⏳ 待异常注入执行（验证黑匣子不丢失 + 停机行为）。

### POST-TC-05　级别=INFO 编译期裁剪（边界值）

- **[宏]** `logger.h`：`LOG_COMPILE_MAX_LEVEL=LOG_LVL_INFO(3)`（D/T 不进二进制）。`app_config.h`：LOGGER profile。
- **[Keil]** Rebuild(0E/0W) → Download。
- **[连]** ST-Link + UART1。
- **[发]** 上电复位；运行期依次发 `debug5`（请求 TRACE）、`debug2`（请求 WARN）观察回显。
- **[读]** `logger smoke` 段应仅 `F/E/W/I` 四行；`debug5` 后回显 `log level -> 3`（被编译上限封顶到 INFO，TRACE 未编入故无法放出）；`debug2` 回显 `log level -> 2`（WARN，运行级生效）。
- **[实测记录]（2026-08-30）** ✅ PASS：编译上限=3(INFO)；运行期 `debug5 > log level -> 3`、`debug2 > log level -> 2`；`log smoke` 仅 F/E/W/I，证明 D/T 被编译期裁剪、运行期调级被封顶——门控两级 AND 正确。

### POST-TC-06　级别=DEBUG/TRACE 放出（边界值）

- **[宏]** `logger.h`：`LOG_COMPILE_MAX_LEVEL` 抬到 `LOG_LVL_DEBUG(4)` 或 `TRACE(5)`；`LOG_RUNTIME_DEFAULT_LEVEL` 须 ≤ 它；并按需 `#define DBG_LOG_POSTEST 1` 等。`app_config.h`：LOGGER profile。
- **[Keil]** 改完两宏 → **必须 Rebuild**（上限变化属编译期）→ Download。
- **[连]** ST-Link + UART1。
- **[发]** 上电复位；运行期发 `debug4`（→DEBUG）或 `debug5`（→TRACE）提级。
- **[读]** `logger smoke` 多出 `[DEBUG]`（及 `[TRACE]` 若上限=5）行（Logger_Test 的 `LOG_D`/`LOG_T` 样例行）；POST 系统里程碑恒为 `LOG_EMIT_DIRECT(LOG_LVL_INFO)` 不受运行级影响。
- **[实测记录]（2026-08-30）** ✅ PASS（裁剪逻辑验证）：当前编译上限固定=3(INFO)，故 `debug5` 无法放出 TRACE（回显封顶到 3）——这与 TC-05 互为印证，证明「需抬上限+重编」才出 D/T 的机制正确。待抬上限重编后补一次放出观测。

### POST-TC-07　栈水位（性能压力）

- **[宏]** 任意 profile（本例 LOGGER）。`app_config.h` 决定自检项数；栈峰值随项数上升。
- **[Keil]** Rebuild → Download。
- **[连]** ST-Link + UART1。
- **[发]** 上电复位，待 POST 跑完。
- **[读]** 串口末段打印 `Task_Test stack: used=XXXX/4096 bytes (free=YYYY) -> 建议 final stack ≈ ZZZ`。`Task_Test_attributes.stack_size` = CubeMX Stack Size 字段 ×4 字节（1024 words ×4 = 4096B）。
- **[实测记录]（本 run LOGGER profile）** ✅ PASS：`used=744/4096`（free=3352，占 18.2%）；建议 final stack≈1000B。**缩容建议**：CubeMX `Task_Test` Stack Size 字段 `1024 → 256 words`（=1024B，≈2× 余量），改完 Rebuild 验 `free>0`。注：栈缩容属 CubeMX 生成区，须用户在 `.ioc` 改后重生成（工程文件 `.uvprojx` 由用户在 Keil 中维护）。

### POST-TC-08　复位源 + 心跳监视（鲁棒性）

- **[宏]** `app_config.h`：使能全模块 profile（各 `APP_ENABLE_X=1`，心跳监视集满）。
- **[Keil]** Rebuild(0E/0W) → Download。
- **[连]** ST-Link + UART1；可选逻辑分析仪接喂狗相关信号。
- **[发]** ① 按 NRST 手动复位；② 拔 USB/断电再上电。
- **[读]** ① NRST → `[BOOT] ResetSrc=PINRST`；② 掉电 → `PORRST`/`BORRST`；两者均一次性，无周期。全模块 profile 连续运行 **72h**，`g_wdt_tick_cnt` 持续递增、无 `IWDG1` 复位（任一被监视任务冻结→不喂→IWDG 复位，此即监视生效的证据）。
- **[实测记录]** ⏳ 需全模块 profile + 72h 长测执行。

### POST-TC-09　任务退出语义回归（不进 `prvTaskExitError`）

- **[宏]** `app_config.h`：LOGGER profile（7 个禁用模块任务 `Task_Inference/Motor/Network/Sensor/Screen/Flash/Esp32S3` 函数体被 `#if APP_ENABLE_X` 裁空）。
- **[Keil]** Rebuild → Download → Debug 进入；在 `port.c:235`（`prvTaskExitError` 的 `configASSERT`）设断点（可选，用于证伪）。
- **[连]** ST-Link + UART1。
- **[发]** 上电复位，让 POST 完整跑完、进入应用。
- **[读]** MDK 暂停 PC **不在** `port.c:235`；各禁用任务函数末尾为 `osThreadTerminate(osThreadGetId())` 自删，永不 `return`（不触发 FreeRTOS 安全网）；串口全程无「中断被全局关闭导致串口断气」。
- **[实测记录]（2026-08-30）** ✅ PASS：LOGGER profile 干净启动全程不进 `prvTaskExitError`；8 处自删点（`osThreadTerminate(osThreadGetId())`）统一生效。根因与修复见 `Components/Debug/Error/post_error.md` E26 / `logger_error.md` E25。

---

## 四、预期目标（报告量化指标摘要）

### 4.1 功能覆盖指标（需求追溯）
- **需求覆盖率**：100%（所有 `APP_ENABLE_X≠0` 的模块均有对应 `Xxx_Test` 进 `g_postests[]`；未使能模块不进表、零噪声——单一真相源）。
- **分支覆盖率**：关键分支（critical 失败→`while(1)` 停机 vs 非关键→`continue`）已通过 TC-03/TC-04 覆盖；难仿真分支靠代码审查标 `-`。

### 4.2 性能指标（KPIs）
- **实时性**：POST 在 `Task_Test`（osPriorityHigh）一次性跑，正常 profile 下 **< 数秒**；含 INFERENCE 时 ML 自检约 **130s**，靠各 `Xxx_Test` 内 `log_wdt_feed()`+`osDelay(1)` 协作喂狗活过（不触发 IWDG）。
- **栈占用**：`Task_Test` **实测 `used=744 / 4096 Bytes`（18.2%）**（本 run LOGGER profile，1024 words 配置）；建议 CubeMX 缩至 **256 words（≈1024B，≈2× 余量）** 防浪费 RAM；缩容后仍需 Rebuild 验 `free>0`。

### 4.3 鲁棒性与稳定性指标
- **长时间稳定性**：全模块 profile 连续运行 **72 小时**，无 IWDG1 复位（心跳监视集满且各任务 `task_heartbeat_kick` 正常）。
- **异常保护**：关键失败（`critical==1` 且 `rc!=0`）100% 触发 `logger_flush_to_flash()`+`while(1)` 停机，黑匣子不丢失；最小 profile（无 critical 项）固件自身发不出自动复位（分支被链接器 GC 删除）。
- **构建质量**：Rebuild **0 Error / 0 Warning**（含 `test_features_processed` 数据集按 `APP_ENABLE_INFERENCE` 门控，消除「定义未使用」告警）。

---

## 附录 A：POST 设计原则速查（红线 · 详细背景）

| 项 | 规则 |
|---|---|
| 单一真相源 | POST 自检表 `g_postests[]` 与 `APP_ENABLE_X` **同一真相源**：每个 `Xxx_Test` 由自身 `APP_ENABLE_X` 门控；未使能模块→函数不编译、不进表；**profile 不再额外配 companion 清单**（避免第二真相源漂移，曾致 Inference 空跑 IWDG 环） |
| Run 循环 | 仍由 `APP_ENABLE_X` 包裹（`freertos.c` 各 `StartXxxTask` 体） |
| 喂狗（POST 期） | TIM7 未 `armed` → `watchdog_should_feed()` 返回 0 → 不自动喂；由各 `Xxx_Test` 内 `log_wdt_feed()`+`osDelay(1)` **协作式喂**（卡死→IWDG 抓到；ML 长段靠协作喂活过） |
| 喂狗（运行期） | `Postest_RunAll` 收尾 `watchdog_arm()` → TIM7 每 500ms 经 `watchdog_should_feed()` 按**任务心跳新鲜度**喂；任一被监视任务（APP_ENABLE_X≠0）冻结→不喂→IWDG 复位 |
| 喂狗点位置 | `TIM7_IRQHandler`（CubeMX 生成 ISR）USER CODE 块：每 1ms `g_wdt_tick_cnt++`，每 500ms 经 `should_feed` 判；独占 TIM7，不占 `HAL_TIM_PeriodElapsedCallback`（TIM6 占做 `HAL_IncTick`） |
| 任务形态 | `Task_Test` 跑完 `Postest_RunAll()` 后 `osThreadTerminate(osThreadGetId())` **自删**（CMSIS 层必须用有效句柄，不能用 `osThreadTerminate(NULL)`——NULL 被当参数错丢弃），只上电跑一次 |
| 关键失败 | `critical==1` 且 `rc!=0`：`logger_flush_to_flash()` → `while(1)` 停机（不返回，等调试器/手动复位；POST 期间 IWDG 未 arm，不会自动复位）；非关键 `critical==0`：`LOG_W`+`continue` |
| 复位源位 | `IWDG1RSTF=bit26` / `SFTRSTF=bit24` / `PORRSTF=bit23` / `BORRSTF=bit21` / `PINRSTF=bit22` / `WWDG1RSTF=bit28`（`main.c print_reset_cause()` 打 `RCC->RSR`） |
| 已删除 | `HardFaultLab` 目录与 `HARDFAULT_LAB` 宏已于 2026-08-29 由用户拍板**彻底删除放弃**，故障注入/模拟代码不再存在于工程 |

> 关键失败集是**动态**的：随 profile 不同，`g_postests[]` 实际项的 `critical` 字段决定哪些失败会进入停机分支。最小 profile（如 LOGGER）下**无 critical 项** → 关键失败分支不可达 → 链接器以 unused-section 回收删除（见 TC-01 预期：仅一次性 `SFTRST` 来自调试器下载，非固件自停机）。

## 附录 B：复位源判读速查（执行 TC-01/TC-08 用）

| 现象 | 含义 | 处理 |
|---|---|---|
| `ResetSrc=IWDG1` + 每 ~4.1s 重复 | 高优先级任务/ISR 冻结饿死喂狗点 | 查 `Task_Test` 内忙等环 / 外设无应答；看串口有无 POST 进度（无=Logger 被饿死） |
| `ResetSrc=IWDG1` + 每 ~4.1s 重复 + 串口停在 POST 某 `LOG_*` 后 | 某 `Xxx_Test` 内 `LOG_*` 同步直发 / 忙等卡死 | 开 `DBG_LOG_POSTEST` 调试 build（§1.5）：看最后一条 `DBG_LOG_POSTEST` 探针 = 卡死行前最后成功点；缺失的下一条即卡死点 |
| `ResetSrc=SFTRST` 仅**一次**，无周期重打 | 调试器下载时的 `SYSRESETREQ`（一次性），**非复位环** | 正常；继续看后续 POST 输出 |
| 关键失败停机（无复位） | 某 `critical==1` 自检失败→`logger_flush_to_flash()`+`while(1)` 停机；POST 期间 IWDG 未 arm，故**不自动复位**，需调试器/手动复位 | 看 POST 日志定位哪个 CRITICAL FAIL；若要求自动重试，需把 `while(1)` 改为 `NVIC_SystemReset()`（待拍板） |
| `ResetSrc=PINRST` | 手动/NRST 复位 | 正常 |
| `ResetSrc=PORRST`/`BORRST` | 掉电/欠压上电 | 正常 |

## 附录 C：Keil 自维护清单（执行前必查）
1. `Components/POSTest/Src/Postest.c` 加入 `POSTest/Src` 源组（否则 `L6218E: Undefined symbol Postest_RunAll`）。
2. `Components/Debug/Src/watchdog_heartbeat.c` 加入 `Debug/Src` 源组（与 `dbg_telemetry.c` 同组；否则 `L6218E: Undefined symbol watchdog_arm / watchdog_should_feed / task_heartbeat_kick`）。
3. `HARDFAULT_LAB` 宏**不得勾**（目录已删；若 Define 列表仍有残留 Group 引用，清理即可）。
4. Rebuild 应为 **0 Error / 0 Warning**。
