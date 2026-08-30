# LOGGER 任务运行期故障归档（控制台 UART1 / 遥测 / 启动 boot）

> 本文件是 **LOGGER 任务相关**运行期故障归属地：串口控制台（UART1）、VOFA 遥测帧、
> 以及启动/FreeRTOS 初始化阶段（boot）的卡死，统一用全局连续编号 **E N**（与 `Components/Debug/error.md` 一致）。
> 日志/遥测门控总表见 `Components/Debug/ref/log_gating.md`；启动死机通用取证见 `Components/Debug/Error/crash_error.md`。
> 编译期故障归 `Doc/Keil_MDK_ARM_工程排错记录.md`（问题 N）。

---

## 事件 E3：UART1 波形偶发乱码 + 需多次复位才有数据（2026-08-19）

### 现象（用户日志）
- 上电后**有时**只打印 `System Init Success!` 毫无后续，需反复按复位多次才出数据；出数据后呈"密密麻麻"且夹带乱码：
  `?`、以及 `\?\u4fa1\u4fa0\u4fd7` 这类被终端当成多字节中文渲染的废字节。
- MPU6050 模块指示灯常亮（模块供电正常）；accel/gyro 数值本身合理（如 `az≈-16080≈-0.98g`），说明 **MPU6050/I2C 读数正常，问题在传输链路，不在传感**。

### 根因（两条，叠加）
1. **硬件（主因）：UART1 与 PC 串口转 USB 之间没接 GND。**
   没有共地参考，UART 信号电平悬浮 → 接收端采样到垃圾字节 → 表现为 `?` 与中文乱码；
   同时串口芯片可能经 TX/RX 反灌、抖动板子复位/电源，造成"偶尔干净启动、偶尔死寂"，
   即"按几次复位才出数据"。**用户自己已怀疑 GND，确认正确。**
2. **配置（次因）：`DBG_UART_BAUD=115200` 扛不住 30 通道@200Hz。**
   每帧约 270 字节，200Hz → 54KB/s 需求；115200 波特理论上限仅 11.5KB/s。
   `BSP_LOG_UART1_SendPoll` 是逐字节轮询阻塞，单帧发送约 24ms，而数据就绪每 5ms 一次
   → 任务几乎全程阻塞在发送、data-ready 信号量饱和、实际仅 ~40Hz；
   长阻塞窗口内若 PC 下发命令，`BSP_LOG_UART1_OnFrame` 的 `HAL_UART_Transmit` 与轮询发送**抢同一 TDR** → 字节错乱。

### 修复方案
- **治本（硬件，用户手边）**：UART1(TX/RX) 与 USB 串口之间**补一根 GND 线**。这一步做完乱码应消失。
- **治标（配置，`dbg_config.h`）**：`DBG_UART_BAUD` 115200U → **921600U**（CH340 实测可达 2M，留足余量）；
  发送窗口从 24ms 降到约 3ms，TX 争用窗口缩小 8 倍。VOFA+ 波特率同步设为 921600。
- **可观测性（代码，`main.c` / `freertos.c`）**：
  - `main.c` `Attitude_Init()` 返回值判错，打印 `MPU6050 Init OK` / `WARN: ... Init FAILED!`（原无条件打印 Success）。
  - `freertos.c` `StartSensorTask` 内 `MPU6050_EnableInt()` 返回值判错，失败打印告警
    （失败 → data-ready 中断不触发 → 传感器任务卡在 acquire 超时 → 波形静默，便于定位"偶发静默"）。
- **可选进一步优化（非本次）**：UART1 所有 TX 收口到一把锁 / DMA+环形队列，彻底消除多任务/ISR 抢 TDR。

### 状态
- [x] 根因定位（GND 缺失为主 + 115200 带宽不足/争用为次）
- [x] `dbg_config.h` 波特率 115200U → 921600U（本次实现）
- [x] `main.c` Attitude_Init 判错打印（本次实现）
- [x] `freertos.c` MPU6050_EnableInt 判错打印（本次实现）
- [ ] 用户补 GND 线后复测：乱码消失、单次复位即出数据
- [ ] VOFA+ 波特率改为 921600 看 30 通道波形

---

## 事件 E5：烧录后串口输出纯乱码 / 无可读字符（2026-08-19）

### 现象
- 烧录后串口助手（XCOM/VOFA+）收到**持续不断的乱码**，形如 `?$寗???(劕?寣)` 等多字节中文乱码，完全看不到 `System Init Success!` 或 firewater 数字帧。
- 与 E3 不同：E3 是“有数据但夹杂乱码”，E5 是“全屏不可读”。

### 根因
1. **主因：串口助手波特率未同步到 921600。**
   - 代码已把 `DBG_UART_BAUD` 改为 921600，`Dbg_Telemetry_Init()` 会在运行时把 `huart1.Init.BaudRate` 覆盖为 921600 并重新 `HAL_UART_Init`。
   - 若 PC 端串口助手仍保持 **115200**，接收端采样位宽与发送端差 8 倍 → 每个字节都被错误采样 → 持续全屏乱码。
2. **叠加因：GND 仍未连接（E3 遗留）。**
   - 没有共地时，即使波特率一致，也可能偶发字节错误；但在“全屏持续乱码”场景下，波特率不匹配是主导因素。

### 修复方案
- **第一步（立刻做）**：把串口助手 / VOFA+ 的波特率设为 **921600**，与 `DBG_UART_BAUD` 一致。
  - VOFA+：端口 → 波特率 → 921600。
  - XCOM：串口设置 → 波特率 → 921600。
- **第二步（硬件）**：UART1 的 TX/RX 与 USB-转串口之间**补 GND 线**（E3 的治本动作）。
- **第三步（验证）**：复位一次，应能直接看到 `System Init Success!` 和随后的 firewater 数字帧。

### 若 USB-转串口芯片不支持 921600 的回退方案
- 部分 CH340G/杂牌模块在 921600 下不稳定；CP2102/FT232/CH340C 通常可支持到 1M 以上。
- 若确认适配器不支持 921600，则回到 115200 并降频：
  - `dbg_config.h`：`DBG_UART_BAUD 115200U`
  - `dbg_config.h`：`DBG_TELEMETRY_DECIMATE 5`（200Hz / 5 = 40Hz，115200 理论上限约 42Hz，留余量）
  - 同时把 VOFA+ 波特率改回 115200。

### 状态
- [x] 根因定位（终端波特率未同步到 921600，叠加 GND 缺失）
- [ ] 串口助手 / VOFA+ 波特率改为 921600
- [ ] 补 GND 线后复测：应出现可读帧
- [ ] 若适配器不支持 921600，执行 115200 + DECIMATE=5 回退

---

## 事件 E10：遥测帧扩到 34 通道 + 参考角命令合并（2026-08-20）

### 现象 / 用户诉求
- 用户指出：VOFA 通道 **15~16 只有 pitch_ref / pitch_err**，roll / yaw 的参考角与误差没有进波形，不合理（二次开发要用）。
- 串口命令 `P` / `PR` / `PY` 三个子命令易歧义，要求**合并成一条**，仿 `K<kp>,<ki>,<kd>` 的 "单字母 + 逗号分隔三数" 写法。

### 根因（设计层面）
- 旧帧 30 通道，15/16 只给 pitch，未给 roll/yaw ref/err；参考角接口虽已泛化（`g_att_ref[3]` + getter），但遥测没接、命令还拆成 P/PR/PY。

### 修复方案（代码 + 文档，本次实现）
- **dbg_config.h**：`DBG_FRAME_N` 30 → **34**；同步更新分区注释（IMU 组 0-23、电机 24-29、系统 30-33）。
- **dbg_telemetry.c** 帧布局：
  - 15-16 `roll_ref/roll_err`；17-18 `pitch_ref/pitch_err`；19-20 `yaw_ref/yaw_err`；21-23 `kp/ki/kd`；
  - 电机 A 24-26 / B 27-29；系统 30-33。**三轴 ref/err 全部进波形**。
- **attitude.c** `Attitude_ProcessCommand` 的 P 命令：去掉 `PR`/`PY` 子命令歧义，改为
  - `P<roll>,<pitch>,<yaw>` 三轴参考角一起设（仿 K 风格，用 `strtof` 解析 1~3 个数）；
  - `P<deg>`（单参）兼容仅设 pitch（旧 `P-3.6` 仍可）；
  - `P`（无参）打印三轴 ref + err 用于验证。
- **文档同步**：
  - `Components/Debug/README.md`：通道映射表 15-20 三轴 ref/err、电机 24-29、系统 30-33；命令列表改 `P<r>,<p>,<y>`；"30 通道"→"34 通道"。
  - `姿态外环测试指南.md`：命令表去掉 PR/PY，公式/重点通道号更新（15-20 / 24-29）。
  - `串级PID测试完整手册.md`：通道表 15-20 三轴、增益 21-23、电机 24-29、系统 30-33；命令集改 `P<r>,<p>,<y>`；CH 引用号全部顺移。

### 验证步骤（用户侧）
1. Keil 重编 0 错误（帧数组 `s_frame[34]`，索引 0-33；确认 `DBG_FRAME_N=34` 一致）。
2. 烧录后 VOFA 按 **34 通道**重拖控件（旧 30 通道布局失效）。
3. 发 `P0,-3.6,0` → 串口打印 `ref(r,p,y)=0.00,-3.60,0.00`；VOFA 看 CH17(pitch_ref)=-3.6、CH18(pitch_err) 在期望姿态≈0。
4. 验证 roll/yaw：发 `P5,0,10` → CH15(roll_ref)=5、CH19(yaw_ref)=10，确认三轴都进波形。
5. 增益仍用 `K<kp>,<ki>,<kd>`（kd 已修，见 E9），VOFA CH21-23 对应 kp/ki/kd。

### 状态
- [x] `dbg_config.h` / `dbg_telemetry.c` / `attitude.c` 代码修改（本次实现）
- [x] 三份文档同步（README / 测试指南 / 串级手册）
- [ ] 用户重编烧录 + VOFA 34 通道重布局 + `P0,-3.6,0` 验证三轴 ref/err 进波形

---

## 事件 E17：上电卡在 Error_Handler 死循环（FreeRTOS 初始化阶段）— 2026-08-23

### 现象（用户实测）
- Keil 调试，全速运行后**关闭全速**，暂停发现 PC 停在 `Error_Handler` 的 `while(1)`（main.c:249-258），
  并非停在 main 的业务循环里。程序冻死，不上进入 osKernelStart 后的调度。
- AI **先前误判**：曾称"程序进了 main 在循环跑，semithosting 回调 Error_Handler 非致命"。
  **该判断错误**——用户指出实际一直停在 error handler 死循环，已纠正。

> **通用排查链路（读 LR / 反汇编定位 / HardFault 的 CFSR 解码）见 `Components/Debug/Error/crash_error.md`；本事件为具体实例，下面只列根因方向。**
> （map 反查铁证 PC=`0x0800F2A0` ∈ `queue.o(.text.xQueueGenericSend)` 已记入该手册，此处不重复。）

### 根因方向（已缩小，待调用栈最终确认）
- `configASSERT`（FreeRTOSConfig.h:156）定义：
  `#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}`
  —— 断言失败即 `for(;;)` 死循环，表现与 `Error_Handler` 的 `while(1)` 相同，都会冻死在调试器里。
- `MX_FREERTOS_Init` 顺序（freertos.c:181-256）：
  `logger_init()` → `osMutexNew`(×1) → `osSemaphoreNew`(×4) → `osMessageQueueNew`(×1) → `osThreadNew`(×7)。
- `xQueueGenericSend` 被调用，最典型触发：
  1. **`osThreadNew`/`osMessageQueueNew` 返回 NULL**（heap_4 分配失败），后续代码未判空继续用 → 断言；
  2. 调度器未启动（osKernelStart 之前）却以阻塞方式调用了队列/信号量 API；
  3. 某任务栈/队列大小配置非法（如 `stack_size` 计算为 0 或溢出）。
- 资源面：`configTOTAL_HEAP_SIZE=65536`（heap_4），7 任务栈合计 4096+6×2048=16384 字节 + TCB/队列，
  静态看 heap 够用，但仍需在调用栈确认是哪个 `osXxxNew` 触发。

### 精确定位步骤（用户在 Keil 里做，无需改代码）
1. 重新进入调试，点 **Stop / Pause**（确保停在死循环）。
2. 打开 **Call Stack + Locals** 窗口（菜单 View → Call Stack Window）。
3. 从栈顶往下读，应能看到类似：
   `configASSERT` → `xQueueGenericSend` / `xQueueGenericCreate` → `osMessageQueueNew` 或 `osThreadNew` → `MX_FREERTOS_Init` → `main`。
4. 看栈里 `MX_FREERTOS_Init` 下一帧是 `osMessageQueueNew` 还是某个 `osThreadNew` → 即失败对象。
5. 在该行加断点重跑，或直接查对应句柄变量（Watch 窗口加 `g_cmd_qHandle`/`Task_InferenceHandle` 等）是否为 NULL。

### 修复方向（确认对象后再动手，保持 CubeMX 零手动）
- 若 heap 不够：调大 `configTOTAL_HEAP_SIZE`（FreeRTOSConfig.h）或缩小任务栈。
- 若某 `osXxxNew` 返回 NULL 未判空：在 `MX_FREERTOS_Init` 后加 `configASSERT(handle != NULL)` 或 if 判空。
- 若是调度器启动前调了阻塞 API：把该调用移到任务函数内。

### AI 误判记录（诚实归档）
- 误判 R1："进了 main 在循环跑，semithosting 回调 Error_Handler 非致命"。
  用户实测反驳：全速后暂停即停在 Error_Handler 死循环，不在业务循环。
  纠正：map 反查 PC=0x0800F2A0 ∈ `xQueueGenericSend`，确认卡在 FreeRTOS 初始化阶段，
  `configASSERT`/Error_Handler 死循环均为致命冻死，非"仍在运行"。

### 状态
- [x] 现象确认（卡 Error_Handler 死循环，用户在调试器实测）
- [x] map 反查定位到 `xQueueGenericSend`（FreeRTOS 初始化阶段）
- [x] 纠正 AI 先前"在 main 循环跑"误判
- [~] 待用户用 Call Stack 确认具体是哪个 osXxxNew 触发断言
- [ ] 修复方案取决于上一步结论

---

## 事件 E18（已更正）：UART1 921600 收端乱码 = VOFA 端波特率不匹配，非 921600 故障（2026-08-27）

> **更正说明**：本事件初版（下方"修复方案（本次实现）"）误判为"本机 CH340 克隆芯片在 921600 下位错误"，并据此把 `DBG_UART_BAUD` 降到 115200、降采样提到 8。**用户复盘确认：之前 PID 全波形采集在 921600 下完全正常，本次乱码是 VOFA+/串口助手被误设为 115200、而 MCU 仍在发 921600 的波特率不匹配**——收端采样位宽差 8 倍 → 全屏乱码。故**本机 921600 可用，恢复 921600 + DECIMATE=1（满速率）为最终配置**；初版的"降 115200"结论作废，保留作排错教训。

### 现象（用户实测，VOFA+ / 串口助手）
- 收端出现长时间二进制乱码 burst：`€€€€\0€\0...`（firewater 浮点帧字节被终端当文本渲染）。
- 根因：**PC 端 VOFA 波特率 = 115200，与 MCU 实际发出的 921600 不一致**（差 8 倍）→ 每个字节都被错误采样 → 持续全屏乱码。
- 将 VOFA 波特率**同步为 921600** 后：乱码 burst 消失，`debug3` 回显 `> log level -> 3` 干净出现，文本日志正常解析，波形满速率刷新。

### 根因（最终）
- **不是物理链路 / CH340 故障**：本机 CH340 克隆芯片在 921600 下稳定（PID 全波形采集已验证）；本次纯粹是**两端波特率不匹配**。
- 与 E3（GND 缺失）区分：本次 GND 已接、波特率对齐后一切正常。

### 修复方案（最终配置，撤销初版降速）
- **`dbg_config.h`**：`DBG_UART_BAUD` **921600U**（恢复；VOFA 端须同步 921600）。
- **`dbg_config.h`**：`DBG_TELEMETRY_DECIMATE` **1**（满速率；44ch@200Hz≈70KB/s << 921600 理论上限 115KB/s，余量充足）。
- **VOFA+ / 串口助手**：波特率设为 **921600**，与 `DBG_UART_BAUD` 一致。
- **CubeMX UART1**：建议也设为 921600 保持一致（运行期 `Dbg_Telemetry_Init()` 已用 `DBG_UART_BAUD` 覆盖，功能上非必须，仅为避免重跑 CubeMX 后看 .ioc 困惑）。
- **其他串口不动**：USART2(ESP-01S)=115200（模块 AT 默认）、USART6(ESP32-S3)=921600（代码强制，MCU 互联）、UART4(淘晶驰屏)=115200。

### 状态
- [x] 现象确认（VOFA 误设 115200 → 乱码；同步 921600 → 干净）
- [x] `dbg_config.h` 波特率恢复 921600U（最终实现）
- [x] `dbg_config.h` `DBG_TELEMETRY_DECIMATE` 恢复 1（满速率，最终实现）
- [ ] 用户 Keil 重编烧录 + VOFA 921600 复测波形/文本

> **supersedes 初版 E18 的"115200 降速"建议**：初版误判 CH340 在 921600 位错误；实为 VOFA 端波特率不匹配。本机 921600 可用，以 921600 + DECIMATE=1 为准。E3/E5 的"921600 是修复"方向正确。

> **深层根因见 E19**：E18 的"VOFA 误设 115200"是症状层；真正"为什么板子实际在发 115200（即便 `dbg_config.h` 已写 921600U）"是 `DBG_UART_BAUD` 的运行期覆盖被 `DBG_LOG_MOTOR` 宏门控、未开时整段变空实现——详见 E19。

---

## 事件 E19（架构陷阱）：改 `DBG_UART_BAUD` 不生效 → 板子仍跑 CubeMX boot 值（2026-08-27，用户实测复盘）

### 现象
- `dbg_config.h:25` 已设 `DBG_UART_BAUD 921600U`，但烧录后 VOFA 设 921600 仍**全屏乱码**；VOFA 设 115200 反而**干净**。
- 用户把 CubeMX 的 USART1 波特率直接改成 921600 → 重新 Rebuild + Download → VOFA 设 921600 后 `> log level -> 3` 干净、波形正常。
- 印证：**板子在改 `dbg_config.h` 期间实际跑的是 115200，并非 `DBG_UART_BAUD` 声称的 921600。**

### 根因（精确，代码实证）
- **运行期波特率覆盖被宏门控**：`Dbg_Telemetry_Init()` 内的 `huart1.Init.BaudRate = DBG_UART_BAUD`（dbg_telemetry.c:41-47）位于
  `#if DBG_TELEMETRY_ENABLE && defined(LOG_ENABLED) && (DBG_TELEMETRY_IMU || DBG_TELEMETRY_MOTOR || DBG_TELEMETRY_SYSTEM)（dbg_telemetry.c:29，Opt-B 后三组 OR；旧版为单一 DBG_LOG_MOTOR==1）**守卫块内**。
- **旧版默认 `DBG_LOG_MOTOR=0`（单一门控时代）** → 该守卫为假 → `Dbg_Telemetry_Init` 编译成**空实现**（dbg_telemetry.c:139-141 `#else` 空函数）。**Opt-B 后**：分组改三组独立 OR 且默认 `DBG_TELEMETRY_IMU/MOTOR=1`，故默认 build 守卫即真、`Dbg_Telemetry_Init` 为真实实现，`DBG_UART_BAUD` 覆盖在默认态即生效（原 115200 触发条件消失）。
  → **`DBG_UART_BAUD` 在运行期被完全忽略，从未写入 huart1。**
- 因此 UART1 实际波特率 = **CubeMX 给 USART1 的 boot 初始化值**（`usart.c:92`，原 115200）。
- 用户"刚才"乱码时：CubeMX 仍 115200 + `DBG_LOG_MOTOR=0`（覆盖空实现）→ 板子发 115200；VOFA 设 921600 → 两端差 8 倍 → 全屏乱码。VOFA 设 115200 → 匹配 → 干净。
- 用户这轮修复：CubeMX USART1 改 921600 → boot 值变 921600 → 板子直接发 921600 → 与 VOFA 对齐 → 干净。**该修复从 boot 值绕开了宏门控，正确且稳健。**

### 关键规则（防再踩）
1. **`DBG_UART_BAUD` 不是"设了就生效"**：它只在遥测守卫为真（任一分组开 + `LOG_ENABLED` 定义 + `DBG_TELEMETRY_ENABLE=1`，Opt-B 后默认 IMU/MOTOR 开即真）时经 `Dbg_Telemetry_Init()` 运行期覆盖生效；否则被空实现吞掉，板子用 CubeMX boot 值。
2. **要让 UART1 = 921600，两条路任选其一**：
   - **路径 A（用户采用）**：CubeMX 把 USART1 波特率设 921600（boot 值直接正确，不受宏影响；`DBG_LOG_MOTOR` 开不开都行）。
   - **路径 B（开宏）**：保持 `DBG_LOG_MOTOR=1`（或在 dbg_config.h 把遥测总闸打开），则 `Dbg_Telemetry_Init` 覆盖生效、`DBG_UART_BAUD=921600U` 才真正写入 huart1；此时 CubeMX 设何值都不影响运行期速率（仅 boot 一小段）。
3. **任何 `.ioc` / 源文件改动后必须 Keil Rebuild + Download**：只改 CubeMX/.c 不烧录，板子仍是上次固件（本次"刚才乱码"一半也是因为没重烧）。

### 修复方案（本次采用路径 A）
- **CubeMX USART1 BaudRate → 921600**（已生成 `usart.c:92 = 921600`）。
- `dbg_config.h:25` `DBG_UART_BAUD` 保持 921600U（与路径 A 一致；即便覆盖空实现也无害，仅作文档/运行期同源）。
- Rebuild + Download + VOFA 设 921600 → 干净（用户已验证 `> log level -> 3`）。

### 状态
- [x] 现象确认（改 dbg_config.h 不生效、板子跑 115200；CubeMX 改 921600 后干净）
- [x] 根因锁定（`DBG_UART_BAUD` 被遥测分组门控，未开时覆盖为空实现，退回 CubeMX boot 值；旧版单一 `DBG_LOG_MOTOR` 门控，Opt-B 后改三组 OR）
- [x] 修复（CubeMX USART1=921600 + Rebuild + Download），用户实测通过
- [ ] 备注：Opt-B 后默认分组即开，覆盖在默认 build 即生效；若全关三组分组又想保速率，才需走路径 A（CubeMX）。

---

## 事件 E20（架构缺陷，已修）：命令控制台硬绑 Motor 任务 → 单模块调试关 Motor 后控制台死亡（2026-08-27）

### 现象（设计目标 vs 缺陷）
- **目标**：单模块调试（关掉某模块、只跑目标）时，调试控制台（`debug<n>` 设级、回显、姿态/电机命令）应**始终可用**。
- **缺陷**：命令解析器（`debug<n>` / `X` / 姿态 `T/P/K/C/F/D` / 电机 `A/B/S/R`）原本在 `StartMotorTask` 内，是 `g_cmd_qHandle` 的唯一消费者。一旦 `APP_ENABLE_MOTOR=0`（或 Motor 任务不创建），命令无人消费 → 队列满后 ISR 丢弃 → **控制台死亡**，直接违背"只跑目标模块也能调试"。
- 这是单模块调试的**阻塞性架构缺陷**（不是运行时崩溃；今天 8 任务全建时 Motor 一直在跑，故未暴露）。

### 根因
- 命令分发误放进**业务任务（Motor）**，而非常驻的调试基础设施（Logger）。
- 耦合点：`freertos.c` `StartMotorTask` 内的 `g_cmd_qHandle` 消费者（原 :417-456）。
- 用户诉求对齐：`DBG_LOG_MOTOR` 应只管"Motor 调试文本"，绝不该让"Motor 控制台"失效——此前把命令分发放进 Motor 任务是根因。

### 修复方案（本次实现）
- 抽出 `DbgConsole_Process()`（`freertos.c` 文件顶部 `USER CODE BEGIN Variables`），由**常驻的 `StartLoggerTask`** 循环调用。
- 模块专属 handler 按 `APP_ENABLE_X` 守卫：`ESP32S3_PrintStats`(`#if APP_ENABLE_ESP32S3`)、`Attitude_ProcessCommand`(`#if APP_ENABLE_SENSOR`)、`Motor_ProcessCommand`(`#if APP_ENABLE_MOTOR`)；`logger_set_level` + 回显始终执行（Logger 常驻）。
- `PC13` 运行/刹车按键留 `StartMotorTask`（Motor 专属，Motor 关则无此键，符合预期）。
- 新增 **L1 功能包含门控**：`Components/Debug/Inc/app_config.h`（仅 7× `APP_PROFILE_*` 预设组用户接口，`APP_ENABLE_X` 为预设内部派生宏）；`freertos.c` 8 个 `osThreadNew` 按 `APP_ENABLE_X` 包裹（Logger 不包裹，常驻核心）。
- 三层门控模型（L1 功能 / L2 级别 / L3 每任务文本）与依赖组见 `Components/Debug/ref/module_gating.md`。

### 状态
- [x] 现象/根因确认（命令解析硬绑 Motor → 单模块调试控制台死亡）
- [x] 修复（命令解析迁 Logger + `APP_ENABLE_X` 任务门控 + `APP_PROFILE_*` 依赖组）
- [ ] 用户 Keil Rebuild + Download 验证：设 `APP_PROFILE_MOTOR` → 仅 Motor+Sensor+Logger 三任务跑、`debug<n>` 仍回显（见 `debug_test.md` M-TC-01 / M-TC-02）

---

## 事件 E21（架构防 CubeMX 覆写）：freertos.c 生成区门控会被 CubeMX 抹除 →【已撤销 app_tasks.c】改为「函数体放 USER CODE 块、创建交 CubeMX」(2026-08-27)

### 现象（用户直觉对、命中的文件判断反了）
- 用户直觉：**"CubeMX 重生成后会覆写配置"** —— 方向正确，但命中的文件搞反了：
  - `app_config.h` 上一轮已从 `Core/Inc/` 迁到 `Components/Debug/Inc/`（旧文件已删）；CubeMX 只重写 `Core/` 等生成目录，**不碰 `Components/`**，故该文件本身安全。
  - **真雷在 `freertos.c`**：7 个可门控任务的 `osThreadNew` 调用 + 任务函数体 落在 CubeMX 生成区（其 `#if APP_ENABLE_X` 门控包裹不在任何 `USER CODE` 块内），重跑 CubeMX `.ioc` 重生成会被**整段抹掉** → 门控丢失、任务恢复全建（等于没做单模块调试）。
- 早期若 `.ioc` 里仍挂着这 7 个任务，CubeMX 重生成还会在 `freertos.c` 重新生成默认 `osThreadNew` → 与 `app_tasks.c` 的 `App_CreateTasks()` **重复创建同名任务** + 句柄/属性符号**多重定义**（链接器 `L6200E: Duplicate Symbol`）。

### 根因（CubeMX 守护机制）
- CubeMX 只在 `/* USER CODE BEGIN/END XXX */` 守护块内保留手写内容；生成区（如 `MX_FREERTOS_Init` 内 `osThreadNew` 默认位置）每次重生成都被整体覆盖。
- 把"编译期可变条件"的 `osThreadNew` / 句柄 / 实现 写进生成区 = 把软开关交到 CubeMX 手里，重生成即清零。

### 修复方案（修订：撤销手写 app_tasks.c，回到 CubeMX 合规）
- **删除 `Components/Debug/Src/app_tasks.c` + `Inc/app_tasks.h`**：手写独立文件把 `osThreadNew` 搬出 CubeMX 生成区，字面违反本规范 §1.6「RTOS 对象由 CubeMX 生成、不手搓 osXxxNew」与 §3 红线（绕开 CubeMX）。用户明确要求改回 CubeMX 位置。
- **7 个任务函数体放回 `freertos.c` 各自的 `USER CODE BEGIN StartXxxTask` / `END StartXxxTask` 块**，按 `#if APP_ENABLE_X` 包裹——这些块 CubeMX 重生成**保留、不覆盖**，正是「对应位置」。
- **`osThreadNew` / 句柄 / 属性 / 前向声明 交还 CubeMX 生成区**（`.ioc` 保留这 7 个任务），重生成自动恢复，禁手改。
- `StartLoggerTask`（CubeMX 原生 `Task_logger`，常驻核心）保留在 `freertos.c`，其 `DbgConsole_Process()` 调用不变。
- **门控语义变化（代价）**：关 `APP_ENABLE_X` ⇒ 任务仍被 CubeMX **创建**（TCB+栈照分配），仅函数体为空（模块代码不编译）；不再像 app_tasks.c 方案那样「不创建」。若要连 TCB/栈都不占，须在 `.ioc` 删对应任务（每次换 profile 都动 .ioc，权衡后不选）。

### 关键规则（防再踩）
1. **任务函数体（含 `#if` 门控）一律放 `USER CODE BEGIN StartXxxTask` 块**——CubeMX 保留、不覆盖；禁另起 CubeMX 不碰的文件手搓 `osThreadNew`。
2. **`osThreadNew`/句柄/属性/前向声明 一律 CubeMX 生成（`.ioc` 管）**，禁手搓、禁手改生成区（§1.6 / §3 最高红线）。
3. 改 `app_config.h`（`APP_PROFILE_*`）/ `freertos.c` 后照例 **Keil Rebuild + Download**；CubeMX 重生成不动 USER CODE 块内函数体。
4. `.ioc` **保留** 7 个任务（勿删）；仅在「要彻底剔除某任务占用的 TCB/栈」时才在 .ioc 删除对应任务。

### 状态
- [x] 根因定位（`freertos.c` 生成区门控会被 CubeMX 抹除；`app_config.h` 实际已安全）
- [x] 撤销 app_tasks.c（手写独立文件违反 §3）；函数体迁回 `freertos.c` USER CODE 块、创建交 CubeMX
- [ ] 用户：CubeMX Generate Code（保留 7 任务）→ 恢复生成区 + 空 `StartXxxTask` USER CODE 块
- [ ] 我：把 7 个 `#if APP_ENABLE_X` 函数体注入各 `StartXxxTask` USER CODE 块（内容见 `_TEMP_freertos_task_bodies.c` 临时备份，注入后删）
- [ ] 用户 Keil Rebuild + Download 验证：选 `APP_PROFILE_MOTOR` → 仅 Motor+Sensor+Logger 三任务的函数体生效

---

## 事件 E22（E21 回滚遗漏）：删 app_tasks.c 却漏还原 7 句柄 → `L6218E: Undefined symbol Task_Esp32S3Handle` (2026-08-27)

### 现象
- Keil Rebuild 链接期报错：`.\STM32H743VIT6\STM32H743VIT6.axf: Error: L6218E: Undefined symbol Task_Esp32S3Handle (referred from esp32s3.o)`。
- `esp32s3.h:55` 声明 `extern osThreadId_t Task_Esp32S3Handle;`，`esp32s3.c:146` 用 `osThreadFlagsSet(Task_Esp32S3Handle, ESP32S3_FLAG_RX)` 唤醒图像解析任务 → 链接器找不到定义。

### 根因（回滚未做对称还原）
- E21 曾把 7 个任务**句柄+属性+`osThreadNew`+前向声明**从 `freertos.c` 生成区挪到 `app_tasks.c`（手写独立文件）。
- 用户否决 app_tasks.c、要求回滚时，**删了 `app_tasks.c`/`app_tasks.h`，却忘了把那 7 个句柄/属性/`osThreadNew`/原型还原回 `freertos.c` 生成区**。
- 结果：7 个符号在 `app_tasks.c`（已删）和 `freertos.c`（已删）**两边都没有** → `esp32s3.o` 链接 `Task_Esp32S3Handle` 失败。
- 用户当时未跑 CubeMX Generate Code（.ioc 仍保留 7 任务），故 CubeMX 没机会补回句柄；直接 Rebuild 即暴露。

### 修复（手工还原，格式对齐 CubeMX 生成区，Generate Code 后为安全 no-op）
- `freertos.c` 生成区按 CubeMX 原生格式补回：
  - 7 个 `osThreadId_t Task_XxxHandle;` + `const osThreadAttr_t Task_Xxx_attributes`（stack/priority 值精确保留自原 CubeMX 生成：Inference=1024×4/Normal、Motor=512×4/High、Network=512×4/Low、Sensor=512×4/AboveNormal、Screen=512×4/Low、Flash=512×4/Low1、Esp32S3=512×4/AboveNormal）。
  - 7 个前向声明 `void StartXxxTask(void *argument);`（生成区，紧接 `StartLoggerTask` 原型）。
  - 7 个 `osThreadNew`（`MX_FREERTOS_Init` 生成区，无条件创建，紧随 `Task_logger` 之后）。
  - 7 个函数定义（签名/Header 在生成区；**体内层** `#if APP_ENABLE_X` 包裹在 `USER CODE BEGIN StartXxxTask` 块内——CubeMX 保留、不覆盖）。
- 关键：句柄/属性/`osThreadNew`/原型全部落在**生成区**（CubeMX 重生成整体覆盖，无冲突）；只有函数体在 USER CODE 块。将来用户跑 CubeMX Generate Code 只会产出**完全相同**内容 → 干净 no-op，不会重复符号。
- 删除临时备份 `_TEMP_freertos_task_bodies.c`（已注入，不再需要）。

### 关键规则（防再踩）
1. **回滚「手写独立文件」时，必须同时把当初挪走的句柄/`osThreadNew`/原型对称还原回 `freertos.c` 生成区**——否则符号悬空。删文件 ≠ 撤销全部改动。
2. **E21 门控语义（已定）**：关 `APP_ENABLE_X` ⇒ 任务仍被 `osThreadNew` 创建（TCB+栈照分配）、仅函数体为空；要彻底不占资源须 `.ioc` 删任务。
3. 改完 `freertos.c` 后 **Keil Rebuild** 验证再交付（本次 L6218E 正是没先验证就交付回滚版所致）。

### 状态
- [x] 根因（回滚漏还原 7 句柄/osThreadNew/原型）
- [x] 修复（`freertos.c` 生成区补回 7 句柄+属性+原型+`osThreadNew`，函数体 `#if` 门控注入 USER CODE 块；临时备份已删）
- [ ] 用户 Keil Rebuild + Download 验证：选 `APP_PROFILE_MOTOR` → 仅 Motor+Sensor+Logger 三任务函数体生效；`debug<n>` 控制台回显正常

---

## 事件 E23（看门狗误复位）：最低优先级 Logger 被高优先级长段饿死 → IWDG 4.1s 复位（2026-08-28）

### 现象
- 上电循环打印 `MPU6050 Init OK` + `System Init Success!`，约每 4.1s 复位重来；`debug5` 等命令**无 `> log level` 回显** → 控制台任务从未跑到。
- 用户体感「定时复位、独立看门狗生效了」。

### 机制（根因模型）
- IWDG1 在 `main.c:128`（`MX_IWDG1_Init`）启动，**调度器之前**；超时 `Prescaler=32 / Reload=4095 / LSI≈32kHz ≈ 4.1s`，`Window=4095` 不约束提前喂。
- 喂狗点**原全工程仅一处**（此即复位环根因）：`StartLoggerTask` 的 `for(;;)`（`freertos.c` `log_wdt_feed()` + `osDelay(2)`），优先级 `osPriorityLow2`（CMSIS 枚举=1，**最低**）。→ **已根治**：现喂狗点迁 **`TIM7_IRQHandler`**（NVIC prio3，每 500ms 经 `watchdog_should_feed()` 喂），POST 期由各 `Xxx_Test` 协作喂（`selftest.c` 内 `log_wdt_feed()`），`StartLoggerTask` **不再喂狗**，杜绝低优先级喂狗被忙等饿死→IWDG 复位环。
- 任何**优先级高于 Logger** 且**长时间不放 CPU** 的任务，把 Logger 饿死 → 4.1s 无喂狗 → 复位。
- 优先级（数值越大越高）：Motor=High(5) > Sensor/Esp32S3=AboveNormal(4) > Inference=Normal(3) > **Logger=Low2(1)** > Flash=Low1(0) > Network/Screen=Low(-1)。
- 弱符号 `logger.c:153 __weak log_wdt_feed(){}` 被 `iwdg.c:60` 强定义接管（HAL_IWDG_Refresh）→ 喂狗有效（链接含 iwdg.c 否则 MX_IWDG1_Init 报 undefined）。

### 7 任务协同喂狗审计（是否饿死 Logger）
| 任务 | 优先级 vs Logger | 是否饿死 | 理由 / 处置 |
|---|---|---|---|
| **Inference** | 高(Normal) | **会（已修）** | 上电跑 `NUM_TEST_SAMPLES=1605` 次同步推理(≈130s)；每 10 样本 `osDelay(1)` 只让同级/高级，不保证放给 Logger。→ 循环内显式 `log_wdt_feed()` |
| **Esp32S3** | 高(AboveNormal) | 否（安全） | `ESP32S3_Task_Run` 用 `osThreadFlagsWait(FLAG,osWaitForever)` **阻塞**等 ISR 标志，队列空即挂起 → Logger 能跑。**勿改** |
| Motor | 高(High) | 否（安全） | `for(;;){ …; osDelay(100); }` 每 100ms 让出 → Logger 在间隙跑。留作存活证据，**不加喂狗**（否则真卡死被掩盖） |
| Sensor | 高(AboveNormal) | 否（安全） | `osSemaphoreAcquire(…,100U)` 阻塞等 data-ready → 让出。安全 |
| Network | 低(-1) | 否 | 优先级本低于 Logger + `osDelay(200)`。安全 |
| Flash | 低1(0) | 否 | 低于 Logger + `osDelay(1)`；`W25QXX_Test()` 一次性低优先级不饿 Logger。安全 |
| Screen | 低(-1) | 否 | `osDelay(1000)`。安全 |

### 修复（本次）
1. `StartInferenceTask` 推理循环内每样本 `log_wdt_feed()`（协同喂狗：长段自行踢狗，短段靠 Logger 兜底）。
2. `main.c` 上电初始化段（`System Init Success!` 后、调度器前）补一次 `log_wdt_feed()`，防 boot 偏长超时。
3. **未改** Motor/Sensor/Esp32S3/Network/Flash/Screen——它们靠 `osDelay`/阻塞原语让出，Logger 能跑即证明系统健康；在此加喂狗会**掩盖真实卡死**。

### 诚实边界（代码层审计不到的饿死源）
- 逐行审计 7 个任务函数体，Motor/Sensor/Esp32S3 均让出 CPU，**本不应**饿死 Logger。若加固后仍 4.1s 复位，饿死源在**任务函数体之外**：
  - 库内部忙等（`AI_Inference` / `Motor_App_Init` / `Attitude_Init` / `ESP01S_*` AT 指令超时用 `HAL_Delay` 长阻塞）；
  - **ISR 优先级高于 SysTick**（参考 screen_error E2：ISR 优先级 6 > SysTick 15 → 任务上下文拿不到 CPU）；
  - boot 段某初始化 >4.1s。
- **确诊最快路径**：CubeMX 里**临时**关 IWDG（`.ioc` → Independent Watchdog → 取消勾选 → Generate Code），Keil Rebuild+Download 跑一次：狗不复位后串口持续输出，能直接看到**哪个任务在空转/卡死**（如推理 summary 不打印=卡在库、某任务狂刷=忙等）。找到真凶后**务必重新勾选 IWDG**再修对应任务。

### 状态
- [x] 协同喂狗加固（Inference 循环 + boot 段）
- [ ] 用户 Keil Rebuild + Download 验证：复位是否消失、`debug<n>` 是否回显
- [ ] 若仍复位：**临时关 IWDG 确诊**真凶（库忙等 / ISR 优先级 / boot 超时），复现后重开 IWDG 修对应任务

---

## 事件 E24（架构）：上电诊断推理抽成独立 POST 自检（Task_Test），治愈 E23 根因 + 栈水位定大小（2026-08-28）

### 动机
- E23 根因 = 把「1605 样本推理」这种**诊断/test 负载**塞进了 `StartInferenceTask` 运行循环（高优先级长段），饿死最低优先级 Logger → 看门狗复位。属「诊断混入运行任务」反模式。
- 用户主张：Init/Test 必须**独立于运行循环**（只关任务=只关 `for(;;)` 体内）；各任务都该有 test 函数；test 前先读外部 Flash 黑匣子打印再清空；关键失败→复位+提示。

### 工业界对照（POST 范式）
- 上电自检（POST）应放**独立一次性任务/调度器前段**，不是嵌进各运行任务。本工程选「独立高优先级一次性任务 `Task_Test`」：有 RTOS 服务、可显式排序、长段自喂狗、跑完 `osThreadTerminate` 自删 → 不污染控制任务实时性。
- 门控（与代码 `selftest.c` 一致）：`#if APP_ENABLE_X` 包**运行循环体 + `Xxx_Test()` 函数体 + `g_selftests[]` 表项**三者（未使能→均不编译、不进表；多 profile 编译必须，否则引未编模块符号→链接失败）；`Xxx_Init()` **常驻编译**（硬件 init 不依赖运行循环），调用处随 `APP_ENABLE_X` 门控。→ 在某使能 profile 内「所有功能都要测、启动前先 catch 故障」；关闭某模块 profile 即视为无该硬件、其自检一并裁掉。

### 实现
- 新建 `Components/POSTest/Src/selftest.c` + `Components/POSTest/Inc/selftest.h`（CubeMX 不碰）。
- `FaultDiag_ML_Test()`：原 `StartInferenceTask` 推理段**整段迁入**（dummy 推理 init + 1605 循环 + 指标计算 + 写 `g_Test_results` + accuracy 阈值 0.85 判通过），循环内 `log_wdt_feed()` + `osDelay(1)`。
- `Flash_Test()`：**先 `W25QXX_BufferRead(0x7FF000,512)` 打印黑匣子 → `W25QXX_SectorErase(0x7FF000)` 清空 → `W25QXX_Test()` 芯片自检**（用户要求的顺序）。
- `Sensor/Esp32S3/Motor/Network/Screen_Test()`：桩（打印+返回 0，**待补全真实校验**）。
- `Selftest_RunAll()`：顺序 Flash→Inference→Sensor→Esp32S3→Motor→Network→Screen；**关键清单 {Inference,Sensor,Esp32S3,Flash}** 失败→`logger_flush_to_flash()`+`NVIC_SystemReset()`；非关键失败→`LOG_W`+继续。
- `StartTestTask`（CubeMX 生成骨架，体在 USER CODE 块）：`Selftest_RunAll()` → **栈水位打印**（`osThreadGetStackSpace`，CMSIS-RTOS2 已声明 cmsis_os2.h:395）→ `osThreadTerminate(osThreadGetId())`（CMSIS 自删必须传有效句柄；`osThreadTerminate(NULL)` 是 NO-OP，会落入 `prvTaskExitError`）。
- `TestResults_t` 类型从 `freertos.c` 局部**提到 `ai_infer.h`**（故障诊断模块头，CubeMX 不碰），`freertos.c`/`selftest.c` 共享；`g_Test_results`/`InferenceDataMutexHandle` 仍定义在 `freertos.c`，`selftest.c` 用 `extern` 引用。
- `StartInferenceTask` 运行循环体改为**仅空闲 `for(;;) osDelay(1000)`**（诊断已由 Test 任务跑、结果已写 `g_Test_results`）。

### 栈水位定大小（用户诉求）
- CubeMX `Task_Test` Stack = `2048*4 = 8192 字节`（用户填 2048，疑问「是否太大」）。
- 串口将打印 `Task_Test stack: used=X/8192 bytes (free=Y) -> 建议 final stack ≈ X+256`。
- **定大小流程**：Keil Rebuild+Download → 读 UART1 该行 → 把 CubeMX Stack 改为 `used/4 向上取整 + 余量`（如 used≈4200 → 改 1280=5120B，或 1536=6144B 留余）。当前 2048 是** generous 临时值**，量完即缩。
- 注意：`osThreadGetStackSpace` 返回自任务启动以来**未用栈最小值（高水位）**，即峰值时剩余；`used = total - free` 即峰值用量。

### 状态
- [x] 代码落地（selftest.c/.h + freertos.c 注入 StartTestTask + 清理 StartInferenceTask + TestResults_t 提头）
- [ ] 用户 Keil 把 `selftest.c` 加入 `Components/POSTest/Src` 源组（否则 Selftest_RunAll 链接 L6218E）
- [ ] 用户 Keil Rebuild + Download → 读 `Task_Test stack: used=...` 定最终 Stack；关键失败会复位（属预期，看黑匣子）
- [ ] 5 个桩（Sensor/Esp32S3/Motor/Network/Screen）待补全真实校验

---

## 事件 E25（任务退出故障，已修）：`StartTestTask` 用 `osThreadTerminate(NULL)` 自删 → CMSIS 当参数错 NO-OP → 函数自然 return → `prvTaskExitError` 安全网（2026-08-30）

### 现象（用户二次进入，串口取证）
- POST 全程打印正常：`[BOOT] IWDG DISABLED` → `ResetSrc=SFTRST` → `System Init Success!` → `Power-On Self-Test start (1 modules)` → `Logger OK` → `=== Self-Test done ===`；随后**进入 `prvTaskExitError`**（`port.c:235` `configASSERT(uxCriticalNesting == ~0UL)`）。
- 串口正确执行了 BOOT/POST 段 ⇒ 非 HardFault、非栈溢出（用户原疑"栈溢出"，经 TCB 解名 + `ulDummy` 取值排除）。
- 调试器：`pxCurrentTCB=0x24008B30`，其 `pcTaskName @0x24008B68` 解小端 = **`Task_Test`**（POST 自检任务）；局部 `ulDummy=0x00000000`。

### 调试器定位（定凶手 + 第二重印证）
- 停住看 `pxCurrentTCB` → `((TCB_t*)pxCurrentTCB)->pcTaskName` 定凶手（或像用户那样直接解 TCB 内存小端：`0x6B736154=Task / 0x7365545F=_Tes / 0x00000074=t`）。
- `ulDummy=0` 说明刚跳入 `prvTaskExitError` 入口、**还没执行 `ulDummy=1` 赋值** → 是干净的任务函数自然 return 路径（LR 被编译期压成 `prvTaskExitError`），不是 HardFault 强跳、也不是临界区未配对（本次 `configASSERT(uxCriticalNesting==~0UL)` 未二次触发，排除嵌套泄漏）。

### 根因链（读 CMSIS 源码坐实）
1. **FreeRTOS 安全网机制**：任务函数一旦自然 `return`，编译期压入的 LR 即指向 `prvTaskExitError`，收口进死循环。它不是硬件故障，是"任务忘了自终/自循环"的兜底。**处置 SOP**：①勿慌栈损坏；②定凶手任务名；③保证该任务函数永不 return（持久 `for(;;)`，一次性/禁用 `vTaskDelete(NULL)`/`osThreadTerminate` 收尾）；④若 `configASSERT(uxCriticalNesting==~0UL)` 也触发，才是临界区未配对（另查）。
2. **直接根因**：`StartTestTask`（`freertos.c`，原末行）写的是 `osThreadTerminate(NULL);` 想"自删"。但读 `Middlewares/.../CMSIS_RTOS_V2/cmsis_os2.c` 的 `osThreadTerminate` 实现：
   ```c
   osStatus_t osThreadTerminate (osThreadId_t thread_id) {
     TaskHandle_t hTask = (TaskHandle_t)thread_id;
     if (IRQ_Context() != 0U)        stat = osErrorISR;
     else if (hTask == NULL)         stat = osErrorParameter;   // ← NULL=非法句柄，直接拒，根本不调 vTaskDelete
     else { if (eTaskGetState(hTask) != eDeleted) vTaskDelete(hTask); stat = osOK; }
     return stat;
   }
   ```
   `hTask==NULL` 分支直接 `return osErrorParameter` → **自删是空操作** → 函数落到 `}` 自然 return → FreeRTOS 收口 `prvTaskExitError`。
3. **契约错配（为什么 `vTaskDelete(NULL)` 行、而 `osThreadTerminate(NULL)` 不行）**：
   - `vTaskDelete` 是 FreeRTOS 原生 API，契约里 **NULL = "删除调用者自身"官方哨兵**，无 NULL 守卫，直达删当前任务。
   - `osThreadTerminate` 是 CMSIS-RTOS2 抽象层封装，**NULL = "无对象/非法句柄"通用哨兵**（与 FreeRTOS 不同）；"删除当前任务"语义在 CMSIS 被单独抽成 `osThreadExit()`（内部 `vTaskDelete(NULL); for(;;);`，标 `__NO_RETURN`）。
   - **错不在 API，是把 FreeRTOS 的"NULL=自身"语义错套到 CMSIS 上**——CMSIS 不接受 NULL 当自身。
4. **为什么 `Task_Test stack: used=...` 没打印**：`osThreadGetStackSpace`+`LOG_I` 已入环，但 `osThreadTerminate(NULL)` 空操作 → return → `prvTaskExitError` 死循环，最低优先级 `Task_logger` 抽空任务永不被调度，栈日志滞留环内未落线（与"POST 全打印"自洽）。

### 修复（已落地，全工程统一）
- `freertos.c` 原末行 `osThreadTerminate(NULL);` → **`osThreadTerminate(osThreadGetId());`**（传有效句柄 → CMSIS 走到 `vTaskDelete(hTask)` 删当前任务；行为等价裸 `vTaskDelete(NULL)`，但留在 CMSIS-RTOS2 抽象层、可移植）。
- **全工程 8 处任务自删统一**为 `osThreadTerminate(osThreadGetId())`：7 个 `APP_ENABLE_X` 门控任务（StartInference/Motor/Network/Sensor/Screen/Flash/Esp32S3，函数体被 `#if` 裁空时自删，避免空函数 return 触发 `prvTaskExitError`）+ `StartTestTask`（POST 跑完真删，不长期占高优先级 8192B 栈+TCB）。
- `INCLUDE_vTaskDelete=1` 已开（`FreeRTOSConfig.h`）；`osThreadGetId()` 本文件 `StartTestTask` 内已用（栈水位测量），符号可用、编译无忧。
- 文档同步（防再误导）：`module_gating.md:19` / `Keil_MDK_ARM_工程排错记录.md:112` / `post_test.md:98` / `AI_deploy_开发规范与提示词.md:36` / `待处理.md:181/183/185` 全部改指 `osThreadTerminate(osThreadGetId())`；`freertos.c` 仅留一处反例注释说明"osThreadTerminate(NULL) 即 NO-OP"。

### 铁律（防复发，已入项目日志）
- 任务**自删当前任务**：用 `osThreadTerminate(osThreadGetId())`（CMSIS 抽象层、可移植）／等价裸 `vTaskDelete(NULL)`；**绝不用 `osThreadTerminate(NULL)`**（CMSIS 下 NULL=参数错 NO-OP → 落 `prvTaskExitError`）。
- 持久任务末端 `for(;;)`；一次性/禁用任务末端显式自删收尾；**任务函数永不自然 return**。
- `osThreadTerminate` 未标 `__NO_RETURN`，删完当前任务后那行 `}` 静态分析可能提示"函数可达结尾"——任务已删永不再调度，无害；要消除提示可其后加 `for(;;);`（纯冗余保险，不加也行）。

### 关联
- **E24**（同源：Task_Test POST 架构，本事件是其末行自删写法的具体踩坑）；**E23**（喂狗，Task_Test 跑完不拖控制环）；本事件机制同时解释了早期的"7 门控任务空函数 return 触发全局关中断冻结串口"（同一 `prvTaskExitError` 安全网，不同触发点）。

### 状态
- [x] 根因坐实（读 cmsis_os2.c：NULL 分支 NO-OP）
- [x] 代码修复：`freertos.c` StartTestTask + 7 门控任务共 8 处统一 `osThreadTerminate(osThreadGetId())`
- [x] 文档同步（6 处）+ 反例注释保留
- [ ] 用户 Keil Rebuild + Download 验证：POST 后**不再进 `prvTaskExitError`**、`Task_Test` 真删、`Task_logger` 接管、`Task_Test stack: used=...` 应落线

