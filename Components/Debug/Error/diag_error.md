# DIAG 任务运行期故障归档（推理 / Inference）

> 本文件是 **DIAG 任务**（StartInferenceTask，离线故障诊断 INT8 模型测试）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/Error/Error_Readme_idx.md` 一致）。
> 编译期故障归 `Components/Debug/Error/Error_Readme_idx.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

## 事件 E18：debug 下 watch 读 g_Test_results.macro_precision=0，实为观测假象（推理已正常跑完，实测 0.91128）（2026-08-23）

### 现象（用户实测 · 已反转）
- 初判：Keil 调试时 watch 观察 `g_Test_results.macro_precision` 全程保持 0，以为 `StartInferenceTask` 没跑。
- **真相（串口/最终观测实证）**：`g_Test_results.macro_precision` 实际为 **0.91128…**（即 INT8_HYBRID 硬件实测 ~91%，与 fault_diagnosis_test.md 完全一致）。
- 系统未崩溃，`htim1.Instance->CNT` 持续变化，欧拉角姿态结算（Task_Sensor）正常。
- 结论：**推理任务完整执行并正确写回结果，系统行为完全正常**。"恒为 0" 是调试期的**观测假象**，非代码缺陷。

### 为什么 debug 下容易误读成 0（根因＝观测方式，不是推理逻辑）
1. **`StartInferenceTask` 只在启动后跑一次**（freertos.c:264-348）：热身 `AI_Inference` → 1605 次循环 → 写 `g_Test_results` → 之后 `for(;;) osDelay(1000)` 空转。
   结果在**上电后约 0.8 s（Release）内一次性写定**，之后 `macro_precision` 不再变化。
2. **watch 窗口的"全程为 0" = 在结果写定之前的那一瞬被观察**。用户在调试器里单步/暂停的时点若早于推理写回（尤其 -O0 下整批更慢、写回更晚），看到的就是初值 0；等到写回完成再看，就是 0.91128。
3. **`DBG_LOG_ENABLE=0`（dbg_config.h:28）** 时 `LOG_I("INFER","Summary…")` 整段被编译掉（freertos.c:328-332），串口**没有任何文本 Summary**，用户无法从日志确认"推理已完成"，只能依赖 watch —— 进一步放大"没跑"的错觉。
4. 串口那段 `€€€€\0…`（0x80/0x00 交替）是 `Dbg_Telemetry_Send` 的 **firewater 二进制遥测帧**（attitude 每秒在发，freertos.c:498），**不是推理 Summary**，与 `macro_precision` 无关，勿混淆。

### 已证伪的初判假设（诚实归档）
- ~~R1：-O0 下推理太慢没跑到写回~~ → 实测最终 macro=0.91128，证明**已跑到写回**，只是观察时点早于写回。
- ~~R2：优先级饥饿排不到~~ → 结果已写回，饥饿不成立。
- R3（int8 三方错配→~42%）/ R4（链接启动失败）→ 原本就排除，且 0.91128 与同批权重实测吻合，彻底坐实无错配。

### 正确的 debug 观测姿势（防复发）
- 不要靠 watch 的"瞬时 0"下结论；要么**多等几秒再读 watch**，要么开 `DBG_LOG_ENABLE=1` 让 `StartInferenceTask` 打印 `Summary: … prec=0.9113 …`（freertos.c:329），或在任务入口加 `LOG_I("INFER","enter")`。
- 想确认推理"何时写完"：watch 里盯 `g_Test_results.data_is_ready`（freertos.c:341 写 1），它由 0 变 1 即写回完成，比盯 `macro_precision` 更可靠（变 1 之前 macro 必为 0）。
- 串口二进制流 ≠ 推理日志：区分 `Dbg_Telemetry_Send`（firewater 帧，attitude 用）与 `LOG_I`（文本，需 DBG_LOG_ENABLE=1）。

### 状态
- [x] 现象反转：实测 macro_precision = 0.91128（INT8_HYBRID 正常）
- [x] 定位"watch 恒 0"＝结果写定前的观察假象，非推理未跑
- [x] 排除 R1/R2/R3/R4（全部不成立）
- [x] 给出正确 debug 观测姿势（盯 data_is_ready / 开 DBG_LOG_ENABLE / 区分二进制遥测与文本日志）
