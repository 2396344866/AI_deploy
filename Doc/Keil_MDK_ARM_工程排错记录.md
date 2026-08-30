# Keil MDK-ARM 工程排错记录（STM32H743VIT6）

> 记录时间：2026-08-23（持续更新，2026-08-30 退化为索引）
> 工程：`C:\Users\123\Desktop\AI_deploy\MDK-ARM\STM32H743VIT6.uvprojx`
> 芯片：STM32H743VIT6（Cortex-M7）
> 编译器：ARMCLANG v6.23（Keil MDK-ARM）
> 关键组件：FreeRTOS、CMSIS-DSP（Components/CMSIS_DSP）、CMSIS-NN（Components/CMSIS_NN）、EdgeImpulse（已禁用 IncludeInBuild=0）、Fault_Diagnosis（用户 AI 推理代码）

---

## 文档层级分工（重要）

| 文档 | 层级 | 内容 |
|------|------|------|
| **本文**（`Doc/Keil_MDK_ARM_工程排错记录.md`） | **编译期 / 工程构建（索引）** | uvprojx 打不开、Build 报错、双登记、CubeMX 生成翻倍、CRLF、链接未定义符号（L6200E / L6218E）等**工程构建层面**故障的**索引**；详情迁到 `Components/Debug/Error/` |
| `Components/Debug/Error/compile_link_error.md` | **编译/链接期** | L6200E 重复符号、L6218E 未定义符号（原 问题 1 / 问题 5） |
| `Components/Debug/Error/uvprojx_error.md` | **uvprojx / CubeMX 生成** | 空白+177 Error、command input file、双登记（原 问题 2 / 3 / 4） |
| `Components/Debug/Error/<TASK>_error.md` | **运行期（runtime，按任务）** | 固件烧录后板子行为/死机，按任务分档：`sensor_error` / `logger_error` / `motor_error` / `net_error` / `flash_error` / `esp32_error` / `screen_error` / `diag_error`（E 编号全局顺序） |
| `Components/Debug/Error/post_error.md` | **运行期（启动/POST 卡死）** | 上电 POST 卡死类（原 问题 7 → 事件 E26） |
| `Components/Debug/Error/crash_error.md` | **运行期（调试取证）** | HSE 时钟误配、启动卡死全链路、死循环 LR 反汇编定位、HardFault CFSR 取证 |

> **分工原则**：编译/工程配置问题看本文索引 → 跳 `compile_link_error.md` / `uvprojx_error.md`；板子跑起来后的行为/死机看 `Components/Debug/Error/`（按任务）。两者不混编。
> **归档铁律**：发现**新的编译/链接期故障** → 本文追加"问题 N"索引行 + 详写 `compile_link_error.md`；发现**uvprojx/CubeMX 生成类** → 详写 `uvprojx_error.md`；发现**运行期故障** → 详写 `Error/<TASK>_error.md`（或 POST 启动类 `post_error.md`），并全局顺序分配"事件 E N"编号，禁第三套故障文档。

---

## 一、最终状态（已解决）

- **Build 结果：0 Error(s), 1 Warning(s)**
- **Program Size**：
  - Code=78496 / RO-data=113952 / RW-data=212 / ZI-data=78148
- FromELF 正常生成 hex 文件。

---

## 二、问题归档索引（按发生顺序，详情见 `Components/Debug/Error/`）

> 2026-08-30 治理：本文退化为**编译/工程构建层索引**。原 §二 的"问题 1–7"详细小节已按类别迁出——
> 编译/链接 → `compile_link_error.md`；uvprojx/CubeMX → `uvprojx_error.md`；M_PI（传感器模块专属）→ `sensor_error.md` 事件 E27；POST 启动卡死 → `post_error.md` 事件 E26。
> 工程构建通用指导（CRLF / 路径风格 / FileName basename / 头新源旧）见下文 §四 根因复盘 + §六 CubeMX 注意事项，保持不变。

| 原问题 | 类别 | 归档位置（详情） | 一句话摘要 |
|--------|------|------------------|------------|
| 问题 1 L6200E 重复符号 | 链接 | `Components/Debug/Error/compile_link_error.md` §问题1 | 两套 CMSIS-NN 同名符号撞车，砍旧树 |
| 问题 2 command input file | uvprojx | `Components/Debug/Error/uvprojx_error.md` §问题2 | FileName 写全路径，改 basename（207 处） |
| 问题 3 双登记 `_1.o` | uvprojx | `Components/Debug/Error/uvprojx_error.md` §问题3 | CubeMX 追加条目翻倍，去裸留 Full（36 处） |
| 问题 4 空白+177 Error | uvprojx | `Components/Debug/Error/uvprojx_error.md` §问题4 | LF 致 Keil 拒读，强制 CRLF（核心） |
| 问题 5 L6218E ExitRun0Mode | 链接 | `Components/Debug/Error/compile_link_error.md` §问题5 | 头新源旧，新建 `Core/Src/exit_run0_mode.c` |
| 问题 6 M_PI 未声明 | 编译 | `Components/Debug/Error/sensor_error.md` 事件 E27 | ARM CLANG 无 M_PI，改 `DEG2RAD` 字面量 |
| 问题 7 POST 卡死 prvTaskExitError | 运行期/启动卡死 | `Components/Debug/Error/post_error.md` 事件 E26 | 禁用任务 return→全局关中断，自删修复 |

---

## 三、修复后验收清单（用户手动版 uvprojx）

| 检查项 | 结果 |
|--------|------|
| 换行符 CRLF | ✅ 4623 个 CRLF，0 裸 LF |
| XML 结构合法 | ✅ |
| FileName 含路径 | ✅ 0（全部 basename） |
| 双登记 | ✅ 0 |
| main.c 唯一 | ✅ 1 条 |
| 磁盘文件存在 | ✅ 0 缺失 |
| EdgeImpulse 组 | ✅ IncludeInBuild=0（149 文件，不参与编译） |
| IncludePath 脏路径 | ✅ 无 Drivers/NN、Middlewares/ST、ALGOBUILD |
| 关键 IncludePath | ✅ CMSIS_DSP/Include、Fault_Diagnosis/Inc 均在 |

> **唯一残留小问题**：`IncludePath` 与 `port.c`/`portmacro.h` 的 `FilePath` 曾有 3 处 `..\Components\FreeRTOS_Port\ARM_CM4F`（反斜杠），已统一为 `../` 正斜杠。

---

## 四、根因复盘（给以后的自己）

1. **Keil uvprojx 必须用 CRLF**。`\n` 单换行会让 Keil 拒绝读取（表现为文件全空白 + 重新加载失败），但 Python/通用 XML 工具能正常解析——**不能用通用 XML 解析器判断 Keil 工程文件好坏**。
2. **不要用脚本批量改写 uvprojx 后不验证 CRLF**。heredoc 里 `\r`/`\n` 会被 shell 转义吞掉，写文件后必须二进制核对换行符（`b'\x0d\x0a'` 计数）。
3. **CubeMX 对"文件是否已登记"用字符串精确匹配**。旧条目（`..\`）与新写（`../`）不等→每次生成追加→翻倍。所有路径必须统一为 `../` 正斜杠风格才幂等。
4. **FileName 只放 basename，全路径放 FilePath**。否则 Keil 拼 `.i` 输出路径到错误位置报 `cannot create command input file`。
5. **双登记判断标准**：CubeMX 新条目带 `<FileOption>`，旧条目常裸登记。去重时删裸条目、留带 FileOption 的。
6. **头新源旧导致 L6218E**：CubeMX 固件包升级后 `.h` 声明了新函数但 `.c` 未实现，新建独立 `.c` 提供实现（放 Core/Src，避 CubeMX 覆盖）。

---

## 五、关键文件位置

| 文件 | 路径 | 说明 |
|------|------|------|
| 工程文件 | `MDK-ARM/STM32H743VIT6.uvprojx` | 主工程 |
| 用户手动修改备份（完整文本） | `MDK-ARM/新建 文本文档.txt` | 等价 uvprojx 的纯文本版 |
| FreeRTOS port | `Components/FreeRTOS_Port/ARM_CM4F/port.c` + `portmacro.h` | 用户移动后位置 |
| CMSIS-DSP | `Components/CMSIS_DSP/` | arm_dot_prod/add/mat_init/mat_mult/max_f32 |
| CMSIS-NN | `Components/CMSIS_NN/` + `Components/EdgeImpulse/.../CMSIS/NN/` | 87 文件编译，149 文件禁用 |
| 用户 AI 推理 | `Components/Fault_Diagnosis/Src/ai_infer.c` | 调用 arm_fully_connected_s8 + DSP |
| ExitRun0Mode 实现 | `Core/Src/exit_run0_mode.c` | 防 CubeMX 覆盖的独立实现 |
| CubeMX 配置 | `STM32H743VIT6.ioc` | **必须取消勾选** X-CUBE-ALGOBUILD 的 DSP Library |

---

## 六、CubeMX 再次生成后的注意事项

若需重新 GENERATE CODE：
1. 打开 `STM32H743VIT6.ioc` → Software Packs → STMicroelectronics.X-CUBE-ALGOBUILD → **取消勾选 DSP Library** → GENERATE CODE。
2. 生成后若 FreeRTOS / 其他组又出现双登记或 `\` 反斜杠，**先检查路径风格是否全为 `../`**，再按"有无 FileOption"去重。
3. 禁止用脚本改写 uvprojx 后不二进制验证 CRLF。
4. 生成后若复现 L6218E Undefined symbol，优先检查 `system_stm32h7xx.h` 与 `.c` 版本是否匹配（参考问题 5）。
