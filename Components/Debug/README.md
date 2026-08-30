# Debug 总导航（Components/Debug/）

> 本目录是 STM32H743VIT6 固件的**调试 / 日志 / 遥测 / 故障 / 测试**全套资料的归属地。
> 按「**per-task 三件套 + 全局 Ref**」组织，新增内容照此归类，勿在根散落。

## 一、目录结构

| 路径 | 用途 | 入口 |
|------|------|------|
| `Error.md` | **运行期故障总索引**：事件 E N → 各 `Error/<TASK>_Error.md` 的映射表 | 故障查档第一站 |
| `Debug/` | **联调工作笔记（非故障）**：状态/接线/设计/命令/栈堆/已修复 Bug | `Debug/README.md` + `Debug/联调笔记模板.md` |
| `Error/` | **运行期故障归档**：per-task `Error/<TASK>_Error.md`（事件 E N） | `Error/README.md` |
| `Test/` | **测试方案**：per-task `Test/<TASK>_test.md`（量化 KPI） | `Test/test_template.md` |
| `Ref/` | **全局共用**：门控总表 / 架构图 / VOFA 通道 | 见下 |
| `Tools/` | **调试脚本**：遥测帧校验 `verify_dbg_frame.py` | — |

## 二、per-task 三件套（铁律）

每个 FreeRTOS 任务（`StartXxxTask`）对应三份平行文档，命名与任务一一对应：

| 类型 | 路径 | 性质 | 编号 |
|------|------|------|------|
| 联调笔记 | `Debug/<TASK>_Debug.md` | 非故障 | 否 |
| 故障归档 | `Error/<TASK>_Error.md` | 运行期异常（事件 E N） | 是 |
| 测试方案 | `Test/<TASK>_test.md` | 验收规范 | 否 |

> 三者**绝不混编**：笔记讲「怎么调通」，故障账讲「哪里坏」，测试讲「怎么证明好」。
> 命名对齐 `freertos.c`：`MOTOR→motor`、`SENSOR→sensor`、`SCREEN→screen`、`LOGGER→logger`、`DIAG→diag`、`NET/FLASH/ESP32` 同理。

## 三、全局门控 / 通道（Ref/）

| 文件 | 内容 |
|------|------|
| `Ref/log_gating.md` | **门控总表（去重合并版）**：级别门控 Gate1-4 天花板级联 + 四级/六级表 + 通道分流 + 看门狗双喂 + 发射流/遥测 mermaid；运行期调级 `debug<n>`/Keil |
| `Ref/module_gating.md` | 功能/Profile 门控（L1 `APP_ENABLE_X`）+ 命令台架构（E20 修复）；与日志级别门控（L2/L3）正交 |
| `Ref/vofa_telemetry.md` | VOFA 44 通道映射 |
| `Error/crash_error.md` | 启动卡死 / HardFault 通用取证方法论 |

### 门控模型一句话（详见 `log_gating.md`）

文本日志是**优先级天花板级联，非并列 AND**：

```
Gate1(编译上限, 硬天花板, 最高) → Gate2(运行级别, 被 Gate1 封顶) → Gate3(任务开关, 仅裁 Debug/TRACE)
```

- **Gate1** `LOG_COMPILE_MAX_LEVEL`：高于它的级别永不进 `.bin`。
- **Gate2** `LOG_RUNTIME_DEFAULT_LEVEL` + `logger_set_level()`：有效 = `min(Gate1, Gate2)`，现场可调。
- **Gate3** `DBG_LOG_<TASK>`：只裁该任务 Debug/TRACE 文本；**只有 Gate2 ≥ Debug 才看得到效果**（INFO 及以下不受 Gate3 管）。
- **遥测 Gate4**：在文本门控之外额外叠加的最严一档 AND = `LOG_ENABLED × (DBG_TELEMETRY_IMU/MOTOR/SYSTEM) × DBG_TELEMETRY_ENABLE × 运行≥TRACE`。

## 四、新增一个任务的调试归档

见 `Error/README.md` 第三节「新增一个任务调试/故障归档的步骤」（联调笔记 → 故障归档 → 测试方案 → 登记映射表 → 交叉引用）。

## 五、不编造原则

未实测、未确认根因的故障**一律不建条目**；无记录的任务（NET/FLASH/ESP32）已建 `Error/<TASK>_Error.md` **模板占位**，新增事件时直接追加「事件 E N」、接全局连续编号。
