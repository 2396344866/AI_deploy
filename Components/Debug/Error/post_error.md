# POST / 上电自检 运行期故障归档（跨任务 Task_Test 一次性自检 + boot/启动卡死）

> 本文件归 **POST / 上电自检** 运行期故障（跨任务：一次性 `Task_Test` 自检任务 + boot/启动卡死），无单一任务 owner，故独立成文件，命名与 `Error/<TASK>_error.md` 同构。
> 事件用全局连续编号 **E N**（与 `Components/Debug/Error/Error_Readme_idx.md` 一致）。
> 任务自身运行期故障看对应 `Error/<TASK>_error.md`；编译/链接/uvprojx 故障看 `Error/compile_link_error.md` / `Error/uvprojx_error.md`；启动死机取证归 `Components/Debug/Error/crash_error.md`。
> 原 `Components/Debug/Error/Error_Readme_idx.md` 问题 7 迁入此处（E26）。

---

## 事件 E26（启动卡死伪象）：上电 POST 卡在 entering Logger 后串口断气（prvTaskExitError 全局关中断）【运行期 / 启动卡死】

### 现象
- 关狗（`APP_ENABLE_WATCHDOG=0`）+ 干净全量重编后，串口到 `Selftest_RunAll: entering [1/1] Logger (critical=0)` 后无 `Self-Test done`、无 `*** HARD FAULT ***` 横幅；MDK 暂停 PC 停在 `prvTaskExitError`（`port.c:235` `configASSERT(uxCriticalNesting == ~0UL)`），循环复现稳定。

### 诊断关键
- 横幅缺失 ⇒ 非 HardFault（CM7 任何 fault 必升级 HardFault；`HardFault_Handler` 第一动作即打 banner，用与 `entering` 同一 uart 链路且 `entering` 刚成功）；排除 `bsp_uart1_emit` 自旋（已 `guard=8000` 有界 TXE 超时）；MDK 暂停 PC 命中 `prvTaskExitError` 收口。

### 根因
- LOGGER profile 仅 `APP_ENABLE_LOGGER=1`，但 `MX_FREERTOS_Init`（freertos.c:303–328）**未对 `osThreadNew` 做 `APP_ENABLE_X` 门控**，仍创建 Inference/Motor/Network/Sensor/Screen/Flash/Esp32S3 共 7 个 Task；这些 Task 入口函数体被 `#if APP_ENABLE_X` 裁空 → 空函数直接 `return`。FreeRTOS 任务禁止 return（无返回地址）→ 跳 `prvTaskExitError` → `portDISABLE_INTERRUPTS()` + `while(1)` **全局关中断冻结** → USART1/SysTick/TIM7 中断全停 → 串口后续输出（含 `Self-Test done`）全被吞，伪装成 POST 卡死。
- **同族第二表现（Task_Test 自身）**：`StartTestTask` 用 `osThreadTerminate(NULL)` 自删，CMSIS 当参数错 NO-OP → 函数 return → 同样落 `prvTaskExitError`（详见 `logger_error.md` **E25**）。

### 修复（详见 logger_error.md E25 / E24）
- 7 个门控任务函数末尾（`#endif` 后）加 `osThreadTerminate(osThreadGetId());`，禁用时自删、永不 return。
- `StartTestTask` 末行 `osThreadTerminate(NULL)` → `osThreadTerminate(osThreadGetId());`（CMSIS 自删必须传有效句柄）。
- 两者均走 USER CODE 块，CubeMX 重生成保留。

### 验证
- Rebuild All → Download（狗关）→ 串口完整跑通 `entering [1/1] Logger` → `Logger_Test` 6 级 smoke → `Logger OK` → `=== Self-Test done ===`（2026-08-29 确认）。
- **2026-08-30 终验 PASS**：`Task_Test stack: used=528/8192` 落线、`Task_Test` 真删、`Task_logger` 接管，不再进 `prvTaskExitError`；`debug<n>` 控制台命令回执正常（级别门控：编译上限=3 时 `debug5` 封顶到 3）。

### 状态
- [x] 根因坐实（读 cmsis_os2.c：NULL 分支 NO-OP）
- [x] 代码修复：8 处任务自删统一 `osThreadTerminate(osThreadGetId())`
- [x] 终验 PASS（2026-08-30 串口）
- [x] 原 Keil 文档问题 7 迁出至本文件（E26），Keil 文档退化为索引
