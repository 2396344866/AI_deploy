# 工程排错与运行期故障归档总索引（STM32H743VIT6 / Keil MDK-ARM）

> **文档说明**：本文件由三份原文档合并而成，是排错与故障归档的**唯一检索 / 归档入口**：
> `Doc/Keil_MDK_ARM_工程排错记录.md`（编译期 / 工程构建）+ `Error/README.md`（运行期归档规范）+ `Error/error索引.md`（运行期事件导航）。
> 三份原内容已并入此处，原文件视为废弃。内部交叉引用统一改写为「本索引 §x」。
>
> **工程**：`C:\Users\123\Desktop\AI_deploy\MDK-ARM\STM32H743VIT6.uvprojx`
> **芯片**：STM32H743VIT6（Cortex-M7）  **编译器**：ARMCLANG v6.23（Keil MDK-ARM）
> **关键组件**：FreeRTOS、CMSIS-DSP、CMSIS-NN、EdgeImpulse（已禁用 `IncludeInBuild=0`）、Fault_Diagnosis（用户 AI 推理代码）

---

## 1 文档体系与层级分工（去哪找）

| 故障类别 | 归属 | 本文位置 | 详情文件 |
|----------|------|----------|----------|
| 编译期 / 工程构建（uvprojx 打不开、Build 报错、CubeMX 翻倍、L6200E / L6218E 等） | **问题 N** 编号 | **本索引 §2** | `Error/compile_link_error.md`、`Error/uvprojx_error.md` 等 |
| 运行期板子行为故障（IMU/I2C/UART/姿态/硬件坏/业务偏离） | **事件 E N** 编号（全局连续，不按任务重置） | **本索引 §3 + §4** | `Error/<TASK>_error.md` |
| 启动卡死 / HardFault 通用取证方法论 | 按机理分节 | 详见 `Error/crash_error.md`（具体实例仍记 E 事件） | `Error/crash_error.md` |
| 历史误判诚实归档（跨任务） | 摘要集中 | **本索引 §4.2** | 各 `Error/<TASK>_error.md` 内「误判记录」小节 |

> **分工原则**：编译 / 工程配置问题 → 本索引 §2 索引 → 跳 `Error/compile_link_error.md` / `Error/uvprojx_error.md`；板子跑起来后的行为 / 死机 → `Error/<TASK>_error.md`（按任务），导航见本索引 §4。两者不混编。
> **全局参考（非按任务）**：`Components/Debug/Ref/`（门控总表 `log_gating.md`、VOFA 59 通道 `motor_vofa_telemetry.md`）；`Components/Debug/Tools/`（`verify_dbg_frame.py`）。新增全局参考直接进 `ref/`，勿在 Debug 根散落。
> **归档铁律**：新编译 / 链接期故障 → 本索引 §2.2 追加「问题 N」+ 详写 `compile_link_error.md` / `uvprojx_error.md`；新运行期故障 → 详写 `Error/<TASK>_error.md`（或 POST 启动类 `Error/post_error.md`），全局顺序分配「事件 E N」；**禁第三套故障文档**。

---

## 2 编译期 / 工程构建排错（原 `Doc/Keil_MDK_ARM_工程排错记录.md`）

### 2.1 最终状态（已解决）

- **Build 结果：0 Error(s), 1 Warning(s)**
- **Program Size**：Code=78496 / RO-data=113952 / RW-data=212 / ZI-data=78148
- FromELF 正常生成 hex 文件。

### 2.2 问题归档索引（问题 1–7，详情见 `Error/`）

> 工程构建通用指导（CRLF / 路径风格 / FileName basename / 头新源旧）见 §2.4 根因复盘 + §2.6 CubeMX 注意事项，保持不变。

| 原问题 | 类别 | 详情位置 | 一句话摘要 |
|--------|------|----------|------------|
| 问题 1 L6200E 重复符号 | 链接 | `Error/compile_link_error.md` §问题1 | 两套 CMSIS-NN 同名符号撞车，砍旧树 |
| 问题 2 command input file | uvprojx | `Error/uvprojx_error.md` §问题2 | FileName 写全路径，改 basename（207 处） |
| 问题 3 双登记 `_1.o` | uvprojx | `Error/uvprojx_error.md` §问题3 | CubeMX 追加条目翻倍，去裸留 Full（36 处） |
| 问题 4 空白+177 Error | uvprojx | `Error/uvprojx_error.md` §问题4 | LF 致 Keil 拒读，强制 CRLF（核心） |
| 问题 5 L6218E ExitRun0Mode | 链接 | `Error/compile_link_error.md` §问题5 | 头新源旧，新建 `Core/Src/exit_run0_mode.c` |
| 问题 6 M_PI 未声明 | 编译 | `Error/sensor_error.md` 事件 E27 | ARM CLANG 无 M_PI，改 `DEG2RAD` 字面量 |
| 问题 7 POST 卡死 prvTaskExitError | 运行期/启动卡死 | `Error/post_error.md` 事件 E26 | 禁用任务 return→全局关中断，自删修复 |

### 2.3 修复后验收清单（用户手动版 uvprojx）

| 检查项 | 结果 |
|--------|------|
| 换行符 CRLF | ✅ 4623 个 CRLF，0 裸 LF |
| XML 结构合法 | ✅ |
| FileName 含路径 | ✅ 0（全部 basename） |
| 双登记 | ✅ 0 |
| main.c 唯一 | ✅ 1 条 |
| 磁盘文件存在 | ✅ 0 缺失 |
| EdgeImpulse 组 | ✅ `IncludeInBuild=0`（149 文件，不参与编译） |
| IncludePath 脏路径 | ✅ 无 Drivers/NN、Middlewares/ST、ALGOBUILD |
| 关键 IncludePath | ✅ CMSIS_DSP/Include、Fault_Diagnosis/Inc 均在 |

> **唯一残留小问题**：`IncludePath` 与 `port.c`/`portmacro.h` 的 `FilePath` 曾有 3 处 `..\Components\FreeRTOS_Port\ARM_CM4F`（反斜杠），已统一为 `../` 正斜杠。

### 2.4 根因复盘（给以后的自己）

1. **Keil uvprojx 必须用 CRLF**。`\n` 单换行会让 Keil 拒绝读取（表现为文件全空白 + 重新加载失败），但 Python/通用 XML 工具能正常解析——**不能用通用 XML 解析器判断 Keil 工程文件好坏**。
2. **不要用脚本批量改写 uvprojx 后不验证 CRLF**。heredoc 里 `\r`/`\n` 会被 shell 转义吞掉，写文件后必须二进制核对换行符（`b'\x0d\x0a'` 计数）。
3. **CubeMX 对"文件是否已登记"用字符串精确匹配**。旧条目（`..\`）与新写（`../`）不等→每次生成追加→翻倍。所有路径必须统一为 `../` 正斜杠风格才幂等。
4. **FileName 只放 basename，全路径放 FilePath**。否则 Keil 拼 `.i` 输出路径到错误位置报 `cannot create command input file`。
5. **双登记判断标准**：CubeMX 新条目带 `<FileOption>`，旧条目常裸登记。去重时删裸条目、留带 FileOption 的。
6. **头新源旧导致 L6218E**：CubeMX 固件包升级后 `.h` 声明了新函数但 `.c` 未实现，新建独立 `.c` 提供实现（放 Core/Src，避 CubeMX 覆盖）。

### 2.5 关键文件位置

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

### 2.6 CubeMX 再次生成后的注意事项

若需重新 GENERATE CODE：
1. 打开 `STM32H743VIT6.ioc` → Software Packs → STMicroelectronics.X-CUBE-ALGOBUILD → **取消勾选 DSP Library** → GENERATE CODE。
2. 生成后若 FreeRTOS / 其他组又出现双登记或 `\` 反斜杠，**先检查路径风格是否全为 `../`**，再按"有无 FileOption"去重。
3. 禁止用脚本改写 uvprojx 后不二进制验证 CRLF。
4. 生成后若复现 L6218E Undefined symbol，优先检查 `system_stm32h7xx.h` 与 `.c` 版本是否匹配（参考 §2.2 问题 5）。

---

## 3 运行期故障归档规范（原 `Error/README.md`）

### 3.1 三类文档命名与分工（铁律）

每个 FreeRTOS 任务（`StartXxxTask`）对应三份并行文档，命名与任务一一对应：

| 文档类型 | 路径 | 内容 | 是否进编号体系 |
|----------|------|------|----------------|
| 联调工作笔记（非故障） | `Components/Debug/debug/<TASK>_debug.md` | 状态/接线/设计/命令/栈堆/Bug 链（已修复） | 否 |
| 运行期故障归档 | `Components/Debug/Error/<TASK>_error.md` | 板子跑出来的异常，用 **事件 E N** 编号 | 是（E 编号） |
| 测试方案 | `Components/Debug/Test/<TASK>_test.md` | 目标/环境/用例表/量化 KPI | 否 |

> 命名与 `freertos.c` 的 `StartXxxTask` 对齐：`MOTOR`→`motor`、`SENSOR`→`sensor`、`SCREEN`→`screen`、`LOGGER`→`logger`、`DIAG`→`diag`、`NET`/`FLASH`/`ESP32` 同理。
> 现有实例：`debug/motor_debug.md`（联调）、`Error/motor_error.md`（故障）、`Test/motor_test.md`（测试）。

### 3.2 层级边界（绝不混编）

| 故障类型 | 归属文件 | 编号 |
|----------|----------|------|
| 运行期板子行为故障（IMU/I2C/UART/姿态/硬件坏/业务偏离） | 本目录 `Error/<TASK>_error.md` | **事件 E N**（全局连续，不按任务重置） |
| 编译期 / 工程配置故障（uvprojx 打不开、Build 报错、CubeMX 翻倍、L6200E 等） | 见本索引 §2 | **问题 N** |
| 启动卡死 / HardFault 通用取证方法论 | `Error/crash_error.md` | 按机理分节（具体实例仍记 E 事件） |
| 历史误判诚实归档（跨任务） | 本索引 §4.2（摘要集中） | — |

### 3.3 新增一个任务调试 / 故障归档的步骤

1. **联调笔记**：在 `Components/Debug/debug/` 建 `<TASK>_debug.md`，写状态/接线/设计/命令/栈堆；首行注明「本文件是 <TASK> 联调工作笔记，非故障归档；运行期故障归 `Error/<TASK>_error.md`」。
2. **故障归档**：在 `Components/Debug/Error/` 建 `<TASK>_error.md`；新故障用全局连续 **事件 E N**（接着已有最大编号，不重置）；每个事件含「现象 / 调试器定位 / 根因链 / 修复方案 / 验证步骤 / 状态」。
3. **测试方案**：在 `Components/Debug/Test/` 建 `<TASK>_test.md`，按三段式：① 目标/环境/数据策略/判据/流程；② 用例表（正常值/边界值/异常值/性能压力/抗噪）；③ 量化 KPI（功能覆盖/性能/鲁棒性）。
4. **登记**：在 **本索引 §4.1 事件映射表**加一行，把新事件指向 `Error/<TASK>_error.md#事件-eN`。
5. **交叉引用**：若其它文档（设计说明/手册）提及该故障，链到 `Error/<TASK>_error.md#事件-eN`，不再写死其它索引名。

### 3.4 当前目录映射（8 任务全量，按任务分类）

> **分类决策（按任务，非按元器件）**：本工程调试开关本就按任务命名（`DBG_LOG_<TASK>` 一一对应 8 个 `StartXxxTask`），且**每个任务天然拥有其硬件元器件**（SENSOR 拥有 MPU6050、MOTOR 拥有电机驱动、NET 拥有 ESP-01S、FLASH 拥有 W25Q64、ESP32 拥有 ESP32-S3 CAM、SCREEN 拥有淘晶驰串口屏、LOGGER 拥有日志引擎/黑匣子、DIAG 拥有 INT8 推理模型）。故「按任务」即与代码同构、且自动覆盖「按元器件」，避免 MPU6050 在 SENSOR/遥测/ESP32 图像流水线多处重复归档。

| 任务 | 故障文件 | 覆盖事件 | 测试方案 | 联调笔记 |
|------|----------|----------|----------|----------|
| SENSOR（IMU/姿态/陀螺/I2C） | `Error/sensor_error.md` | E1, E4, E6, E7, E11, E12, E14, E15, E19 | `Test/sensor_test.md` | `sensor_debug.md`（待建） |
| MOTOR（电机/控制环/命令分发） | `Error/motor_error.md` | E8, E9, E13 | `Test/motor_test.md` | `debug/motor_debug.md` |
| SCREEN（淘晶驰串口屏 UART4） | `Error/screen_error.md` | E2 | `Test/screen_test.md` | `screen_debug.md`（待建） |
| LOGGER（控制台/遥测/启动 boot/黑匣子） | `Error/logger_error.md` | E3, E5, E10, E17, E28, E29, E30 | `Test/debug_test.md`（§2.B 引擎机制） | `logger_debug.md`（待建） |
| DIAG（故障诊断 INT8 推理） | `Error/diag_error.md` | E18 | `Test/diag_test.md` | `diag_debug.md`（待建） |
| NET（ESP-01S WiFi/MQTT/阿里云） | `Error/net_error.md` | E31–E37 | `Test/net_test.md` | `net_debug.md`（待建） |
| FLASH（W25Q64 自检/黑匣子） | `Error/flash_error.md`（模板） | 暂无（占位不编造） | `Components/BSP/W25Q64/FLASH.md` | `flash_debug.md`（待建） |
| ESP32（ESP32-S3 CAM 图像结论帧） | `Error/esp32_error.md`（模板） | 暂无（占位不编造） | `Test/esp32_test.md` | `esp32_debug.md`（待建） |

> **覆盖说明**：
> - 已记录运行期故障的任务 = SENSOR/MOTOR/SCREEN/LOGGER/DIAG/**NET**（6 个），对应 `Error/<TASK>_error.md` 已建。
> - **FLASH/ESP32 目前无已记录的运行期故障 → 按用户要求不编造故障条目**，仍为**模板占位**（含「暂无记录」声明 + 重点排查方向）；新增事件时直接在对应文件追加「事件 E N」、接全局连续编号即可。
> - NET 事件于 2026-08-31 归档：**E31**（一次 AT 超时 → RxState 卡 BUSY_RX → 网络永久死亡）、
>   **E32**（USART2 NVIC 未使能 → 永远收不到字节；这是 E31 中"首次 AT 无回响"的真根因）、
>   **E33**（MQTT CONNECT 报文 217 B > 缓冲 160 B + PUBLISH 缓冲无长度检查的 P0 栈溢出隐患）、
>   **E34**（MCU 复位但模块没断电 → 卡在透传模式，AT 全部失效；症状同"模块没电"）、
>   **E35**（CONNACK rc 未参与判定 + 三元组 signmethod/timestamp/securemode 三处不一致）、
>   **E36**（配置密码带 `signmethod=/timestamp=` 前缀，broker 期望 RAW 64-hex 签名 → 静默丢 TCP）。
>   **E37**（`mqtt_build_connect` 剩余长度少算 1B → CONNECT 畸形，broker 静默断链、无 CONNACK；这是 E31–E36 一路修到 MQTT 层后暴露的"根"）。
> - 测试方案已覆盖全部 8 任务（`Test/<TASK>_test.md`）；联调笔记仅 MOTOR 已建，其余按需补（模板见 `debug/联调笔记模板.md`）。
> - 注：E16（uvprojx 全空白+177Error）已迁入本索引 §2.2 问题 4，不在本目录。

---

## 4 运行期事件导航索引（原 `Error/error索引.md`）

> 本索引 §4 是运行期（runtime）故障的**事件导航**：详细事件内容已按 FreeRTOS 任务拆分到 `Error/` 子目录（规范见本索引 §3），本索引 §4 只保留事件映射表、历史误判诚实归档摘要、复测清单。
> **编译期 / 工程配置类故障**单独归本索引 §2（用「问题 N」编号），**绝不混入本体系**。启动卡死 / HardFault 的**通用取证方法论**见 `Error/crash_error.md`（具体实例仍记本体系 E 事件）。
> 原 E16（编译期 uvprojx 全空白+177Error）已迁入本索引 §2.2 问题 4，本体系不再保留副本。

### 4.1 事件映射表（详细内容在 `Error/` 子目录）

| 事件 | 任务归属 | 详细文件 |
|------|----------|----------|
| E1  传感器任务跑一阵后卡死（I2C 总线锁死） | SENSOR | `Error/sensor_error.md` |
| E2  上电只打印 Success! 后无输出（UART4 ISR 饿死调度） | SCREEN | `Error/screen_error.md` |
| E3  UART1 波形偶发乱码 + 多次复位才有数据（GND/波特率） | LOGGER | `Error/logger_error.md` |
| E4  L12-L14 静止周期锯齿（Madgwick 公式抄错） | SENSOR | `Error/sensor_error.md` |
| E5  烧录后纯乱码（终端波特率未同步） | LOGGER | `Error/logger_error.md` |
| E6  静置仍跳变（陀螺零偏标定未生效） | SENSOR | `Error/sensor_error.md` |
| E7  全量分析：陀螺硬件坏值（推翻 E4/E6） | SENSOR | `Error/sensor_error.md` |
| E8  外环符号反（正反馈越倒越快） | MOTOR | `Error/motor_error.md` |
| E9  K 命令 kd 恒 0（逗号未前进） | MOTOR | `Error/motor_error.md` |
| E10 遥测扩到 34 通道 + 参考角命令合并 | LOGGER | `Error/logger_error.md` |
| E11 在线零偏把运动信号吃掉（grate 门限） | SENSOR | `Error/sensor_error.md` |
| E12 raw gyro 恒 0（burst/standby/硬件） | SENSOR | `Error/sensor_error.md` |
| E13 C/F/D 未路由到姿态命令解析 | MOTOR | `Error/motor_error.md` |
| E14 WHO_AM_I 0x68 严格比对误杀国产 0x70 | SENSOR | `Error/sensor_error.md` |
| E15 D 命令实测：陀螺硬件死（DOA） | SENSOR | `Error/sensor_error.md` |
| E17 上电卡 Error_Handler（FreeRTOS 初始化断言） | LOGGER | `Error/logger_error.md` |
| E18 watch 读 precision=0 实为观测假象（推理已正常） | DIAG | `Error/diag_error.md` |
| E19 VOFA 遥测只发一次（MPU6050 INT 边沿配反） | SENSOR | `Error/sensor_error.md` |
| E28 真实 HardFault：flash_hexdump 栈缓冲溢出 → 返回地址被 0x2E2E2E2E 污染（PRECISERR+BFARVALID） | LOGGER | `Error/logger_error.md` |
| E29 一次性自检用裸 printf → 脱离分级门控与黑匣子取证链（已收口到 LOG_*） | LOGGER | `Error/logger_error.md` |
| E30 UART4 接收未随 APP_ENABLE_SCREEN 门控 → 浮空 RX 触发 FE → ISR 内 while(1) 整机冻结 | LOGGER | `Error/logger_error.md` |
| E31 一次 AT 超时后网络永久死亡（RxState 卡 BUSY_RX → 每轮 RX start failed，回不到 AT 阶段） | NET | `Error/net_error.md` |
| E32 USART2 的 NVIC 从未使能 → 永远收不到任何字节（回环自测 rx 0 bytes 暴露） | NET | `Error/net_error.md` |
| E33 MQTT CONNECT 报文 217 B > 缓冲 160 B；顺带修复 PUBLISH 缓冲无长度检查（P0 栈溢出） | NET | `Error/net_error.md` |
| E34 MCU 复位但模块没断电 → 卡在透传模式，AT 全部失效（症状同"模块没电"，极易误判） | NET | `Error/net_error.md` |
| E35 CONNACK 返回码未参与判定（rc=4/5 被当成功）+ 三元组 signmethod/timestamp/securemode 三处不一致 | NET | `Error/net_error.md` |
| E36 配置密码带 `signmethod=/timestamp=` 前缀，broker 期望 RAW 64-hex 签名 → CONNACK 静默超时 | NET | `Error/net_error.md` |
| E37 `mqtt_build_connect` 剩余长度少算 1B（可变头 9→应 10）→ CONNECT 畸形，broker 静默断链、无 CONNACK | NET | `Error/net_error.md` |
| E38 QMC5883L 接 aux-bus(XCL/XDA) 时必须开 MPU6050 I2C 旁路，原代码未回读校验/无日志（终因=杜邦接触不良） | SENSOR | `Error/sensor_error.md` |
| E39 航向保持 engage 输出满舵阶跃（yaw_ref 默认 0 未对齐当前航向）→ 加 `Attitude_SnapYawRefToCurrent()` 上升沿自动对齐 | SENSOR | `Error/sensor_error.md` |
| E40 CH54 `loop_ms` 呈 9/10/11 周期性起伏（**观测型，非故障**：1ms 量化拍频 + 中断驱动异源时钟 + SendPoll 4.41ms 撑爆单拍 + 采样点在中段） | SENSOR | `Error/sensor_error.md` |
| E41 `H`(航向保持)命令完全无效：① 控制台分发表 `freertos.c:150` 漏配 `'H'` 路由 → handler(`attitude.c:745`)写了却永远调不到、命令**静默丢弃**；② TRACE 级遥测把 `g_uart1_text_mute_above` 抬到 WARN，`logger.c:139` 吞掉全部 `LOG_I` 回执。**判据只能看 CH43/CH58，不能看文本** | CONSOLE/SENSOR | `Error/sensor_error.md` |
| E42 运行时发 `C`(陀螺零偏标定)持续 `gyro cal skipped`：I2C1 总线无互斥锁，Sensor 任务热路径与控制台命令争用同一 `hi2c1`。修：`i2c1_mutex`(CubeMX) + per-transaction 加锁，热路径 `block=0`(丢帧) / 命令路径 `block=1`。⚠ TRACE 下 `CAL OK` 是 `LOG_I` 被静音 → **「无输出」=标定成功** | SENSOR/CONSOLE | `Error/sensor_error.md` |
| E43 串口命令集双轨重构暴露三类缺陷：① 路由层只认**大写**单键，`att.*/mag.*/filt.*` 是**小写**且归一化在 handler 内 → **长名全部不可达**（E41 同坑第二次，大小写维度）；② `K`/裸 `T` 无参经 `atof("")/atoi("")` 归零 → **PID 增益清零 / 静默关外环**；③ 归一化丢分隔符(`mag init`→`"Minit"`)。另订正旧文档 4 处（大小写说明反、StartMotorTask 路由、未匹配原样回显、缺 `P`/`T`） | CONSOLE/SENSOR | `Error/sensor_error.md` |

> 命名约定：每个事件文件内仍以 `# 事件 E N` 为二级标题；新增事件继续全局连续编号，不按任务重置。
> **覆盖状态（8 任务全量）**：已记录运行期故障的任务 = **SENSOR / MOTOR / SCREEN / LOGGER / DIAG / NET**（6 个），对应 `Error/<TASK>_error.md` 已建。其余 **FLASH / ESP32** 目前**无已记录的运行期故障**——按约定**不编造故障条目**，结构已预留，后续实测发现异常时直接建 `Error/<TASK>_error.md`、接全局连续 E 编号即可。分类粒度采用**按任务**（与 `DBG_LOG_<TASK>` 一一对应、且每任务天然拥有其硬件），非按元器件。测试方案已覆盖全部 8 任务（`Test/<TASK>_test.md`）。详见本索引 §3.4。

### 4.2 历史误判诚实归档（跨任务摘要）

> 完整细节见各 `Error/<TASK>_error.md` 对应事件内的「误判记录」小节；此处仅列摘要便于复盘。

- **M1（E1 相关）**：曾把卡死错判为 UART1 TX 非线程安全 → 实为 I2C 总线锁死；`?`/`` 是 I2C 坏字节，非 UART 冲突。
- **M2（E3 相关）**：`verify_dbg_frame.py` 路径算错（多一层 `dirname`）→ 改为单层 `dirname(__file__)`。
- **M3（E4 相关）**：误报 `steer` 分组错位 → 核对后 `dbg_config.h`/`README` 一致，纯误报。
- **R1（E17 相关）**：曾称"进了 main 循环跑、Error_Handler 非致命" → 实测停在死循环，非"仍在运行"。
- **H1–H4（E19 相关）**：时钟改回 480M 错乱 / CubeMX 冲掉 EXTI / TIM1 是遥测触发源 / I2C 写 INT_ENABLE 失败 → 全部证伪，真因是 PC3 上升沿 vs MPU6050 active-low 边沿反。

### 4.3 复测清单（用户侧，跨任务通用）

1. **静置**（板子不动）一轮：波形稳定不卡 → 验证速率降档 + 基础链路（E1/E19）。
2. **慢转**板子一轮：姿态角平滑跟随，无非法字符（E4/E6/E11）。
3. **剧烈运动**一轮：应不再卡死在 `I2C_IsErrorOccurred`（bus recovery 生效，最坏有限重试后返回 -1 而非死等）（E1/E7）。
4. **调试观测**：推理结果盯 `g_Test_results.data_is_ready` 而非瞬时 `macro_precision`（E18）；区分二进制遥测帧与文本日志（E18）。
