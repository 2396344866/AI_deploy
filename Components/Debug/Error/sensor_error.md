# SENSOR 任务运行期故障归档（IMU / 姿态 / 陀螺 / I2C）

> 本文件是 **SENSOR 任务**（StartSensorTask + IMU/姿态解算/陀螺）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/error.md` 一致）。
> 联调笔记见 `Components/Debug/debug/sensor_debug.md`（待建）；测试方案见 `Components/Debug/Test/sensor_test.md`。
> 编译期故障归 `Doc/Keil_MDK_ARM_工程排错记录.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

## 事件 E1：传感器任务运行一会儿后卡死，VOFA 波形停（2026-08-19）

### 现象
- 上电打印 `System Init Success!`，firewater 帧正常输出约 1~2 分钟。
- 板子被运动 / 甩动后波形彻底停住，不再刷新。
- 日志中偶发非法字符：`?12.685`、`-0.835`（VOFA 端 UTF-8 解码失败）。
- 陀螺原始值频繁撞 ±32768（INT16 满量程），属运动过烈的正常饱和，不是卡死主因。
- 电机组（通道 20–25）恒为 0，说明卡死仅发生在 IMU/姿态/传感器任务路径。

### 调试器定位（关键）
- 暂停后 PC 落在 HAL 库 **`I2C_IsErrorOccurred()`**，位于 **AF（NACK）分支**等待 `STOPF` 标志恢复。
- **真实死因**：MPU6050 某次 I2C 读未收到 ACK（NACK）→ I2C 总线被从机拉住 / SDA 被锁低 →
  HAL 的错误恢复在总线锁死时无限等待 → **200Hz 传感器任务永久阻塞** → 不再产生 firewater 帧 → 波形停。

### 根因链
1. **触发**：板子剧烈运动 / 线束抖动 / 上拉不足 → I2C 瞬时 NACK。
2. **放大**：`HAL_I2C_Mem_Read` 在总线锁死时，HAL 自身的错误恢复（`I2C_IsErrorOccurred`）仍可能挂死（等不到 `STOPF`）。
3. 那两个非法字符 `?` / ``：**是 I2C 读出的坏字节被遥测原样转发**，不是 UART TX 打架。

### 修复方案（详见下方状态）
- **治本（代码，用户层文件，CubeMX 零影响）**：`Components/BSP/IMU/imu_mpu6050.c` 增加
  - `MPU6050_I2C_BusRecovery()`：SCL 手动 toggle ≥9 个时钟，释放被从机拉低的 SDA，再重初始化 I2C1。
  - `MPU6050_ReadRaw()` / `I2C_WriteReg` / `I2C_ReadReg` 失败先做 bus recovery 再重试 `MPU_I2C_RECOVERY_RETRIES`(=3) 次；仍失败才返回 -1。
- **硬件 / 配置（CubeMX + 手边）**：I2C1 降为 Standard 100kHz；确认 SDA/SCL 有 4.7kΩ 上拉；线束牢靠。
- **未采用**：UART1 TX 改 DMA + 环形队列 —— 那是针对"UART 非线程安全"假设（见 M1 误判）提出的，并非本次真因，故不作为主修复，仅作可选优化。

### 状态
- [x] 根因定位（I2C 总线锁死，非 UART）
- [x] CubeMX I2C 降速指南已给（见对话流）
- [x] `imu_mpu6050.c` 加 bus recovery + 重试（本次实现）
- [ ] 用户 CubeMX 改 100kHz + Generate
- [ ] 静置 / 慢转 / 剧烈运动三轮复测

---

## 事件 E4：VOFA 通道 L12-L14（roll/pitch/yaw）静止时周期性锯齿跳变（2026-08-19）

### 现象
- 板子静止不动，VOFA 上 L12-L14（`att->roll` / `att->pitch` / `att->yaw`）却呈周期性锯齿/跳变。
- 数值在 -103°~+195° 之间大幅摆动，完全不像是静止姿态。
- 原始 accel/gyro（通道 0-11）数值本身合理，问题出在姿态解算层，不在传感器/I2C。

### 根因
`Components/BSP/IMU/Src/madgwick.c` 中 Madgwick 梯度下降校正步的 **s1/s2/s3 公式被抄错**：
- 原版应为 `q1`、`q2`、`q3`（四元数分量）的位置被错写成 `ax`、`ay`、`az`（加速度测量值）。
- 结果：加速度对姿态的修正方向完全错误，四元数无法收敛到重力方向；陀螺零偏被持续积分 → 欧拉角线性漂移 → 跨越 ±180° / ±90° 时发生 `atan2`/`asin` 周期性跳变（锯齿波形）。

### 修复方案
- **治本（代码，`madgwick.c`）**：按 xioTechnologies 原版 MadgwickAHRSupdateIMU 还原 s0-s3。
- **配套（代码，`attitude.c`）**：
  - `Attitude_Init` 增加陀螺零偏标定（50 次采样平均），并打印 `gyro offset(dps)`。
  - `Attitude_Update` 在调用后端前扣除 `s_gyro_offset[0..2]`，并统一改用 `s_imu.*`。
- **可观测性**：上电日志会打印三轴零偏；若标定后 roll/pitch 仍明显漂移，说明板子没放稳或安装轴与 `IMU_PITCH_SIGN` 不符。

### 状态
- [x] 根因定位（Madgwick 公式 s1/s2/s3 抄错，q 与 ax/ay/az 混淆）
- [x] `madgwick.c` 按原版还原（本次实现）
- [x] `attitude.c` 增加陀螺零偏标定与扣除（本次实现）
- [ ] 用户 Keil 重编、烧录：静置看 L12-L14 是否稳定到 0° 附近
- [ ] 慢转板子：roll/pitch 应平滑跟随，无跳变

---

## 事件 E6：重新烧录后静置三轴仍周期跳变（陀螺零偏标定未生效，非 Madgwick 公式）

### 现象
- 上一轮按 E4 把 `madgwick.c` 的 s 向量还原（已与 xioTechnologies 原版逐行核对一致）后重新烧录。
- 用户确认**静置**录制，roll/pitch/yaw（L12-L14）仍有周期性锯齿跳变，且波形与修复前“不一致”，主观感觉“欧拉角换了位置”。
- 同帧原始陀螺（通道 9-11，VOFA 显示的是**未扣零偏**的 raw）读数为 **−33.6 / +34.0 / −4.1 °/s**（静止应为 ≈0）。

### 根因
- Madgwick **无磁力计**，yaw 不可观测；任何**未被扣除的陀螺零偏**会被持续积分 → 姿态欧拉角匀速扫过 ±180° → 周期性锯齿（三轴都跳）。
- 真凶是陀螺自标定 `s_gyro_offset` 没生效：原 `Attitude_Init` 在**上电搬动/放置过程中**就采样 50 次求均值当零偏，把手部运动也算进去；或采样全失败（`good==0`）导致偏移保持 0。两者都使 Madgwick 仍收到 −33°/s 量级的净转速 → 锯齿。
- “角度像换了位置”是观感：修正后的梯度与旧（错误）梯度对同一个净陀螺转速的修正方向不同，锯齿落在不同通道/相位，并非真有通道交换。

### 修复方案（代码，`attitude.c`）
- 抽出 `Attitude_CalibrateGyroStatic()`：采 100 拍，算各轴**极差(°/s)**；若任一轴极差 > `GYRO_STATIONARY_DPS`(5°/s) 判定“板子没放静”，**拒绝标定**（保留旧偏移并打印 WARN），不再把运动当零偏。
- `Attitude_Init` 改为调用该函数；静止加速度均值日志保留（确认安装方向）。
- 新增串口命令 **`C`**：静置后随时重标定陀螺零偏（路由到 `Attitude_CalibrateGyroStatic`）。

### 验证步骤（用户侧）
1. 上电看日志 `gyro offset(dps)=...`：应为 ≈(−33,34,−4)；若打印 `rejected/SKIPPED` 说明上电没放静。
2. 板子**完全放静**后，串口发 `C` 命令重标定，日志应再打印一次 offset 且这次无 rejected。
3. 静置看 L12-L14：应稳定到常量附近不再锯齿。
4. 慢转板子：roll/pitch 平滑跟随。
5. 注意：yaw 无磁力计**会缓慢漂移**（线性，非锯齿），属正常，不要当成故障。

### 状态
- [x] 根因定位（陀螺零偏标定未生效 → Madgwick 积分净转速 → 锯齿；非 Madgwick 公式）
- [x] `attitude.c`：`Attitude_CalibrateGyroStatic()` 静默判定 + `Attitude_Init` 改用 + `C` 命令（本次实现）
- [ ] 用户烧录复测：静置 L12-L14 稳定；发 `C` 后 offset 生效
- [ ] 确认上电日志无 `gyro cal rejected/SKIPPED`

---

## 事件 E7：全量 VOFA 数据分析——姿态漂移真凶是**陀螺硬件坏值**，非 Madgwick/非 UART（2026-08-19）

### 背景
- 用户导出完整 `vofa+.csv`（42760 帧 ≈ 214s @200Hz）要求分析。此前 E4/E6 基于**预览的 29 行**推断，
  结论（Madgwick 公式错 / 零偏标定未生效）在**全量数据下被推翻**。
- 教训：用 VOFA 预览的前几十行下结论会漏掉统计量；**分析姿态必须用全量导出 CSV**。

### 全量统计（python 直接算 csv）
| 维度 | 数值 | 判读 |
|---|---|---|
| 加速度模长（干净帧） | 均值 **0.93g / std 0.046g** | 加速度计**健康**、板子朝向恒定（≈静止） |
| 串口损坏帧（加速度偏离 >0.3g） | **0.2%**（100/42760） | **UART 链路基本正常**，不是主因（推翻 E3/E5 的“链路主导”印象） |
| 陀螺原始饱和帧（\|raw\|>30000 ≈±250°/s） | **28.0%**（即使在加速度干净的帧内） | **陀螺在疯狂输出坏值** |
| 陀螺 dps（干净帧）mean/std | **-24.5 / 10.7 / -4.3 °/s**，std 37/39/7 | 静止应为≈0，实为大幅偏置+噪声 |
| 姿态角范围 | roll/yaw **±180° 绕满一圈**，pitch 摆 110° | 即“周期锯齿/换位置”观感 |

### 根因（关键更正）
1. **加速度与陀螺是同一笔 I2C 14 字节读取**（accel 6 + temp 2 + gyro 6，`MPU_REG_ACCEL_XOUT_H` 起）。
   加速度在 99.8% 帧内完全干净、模长稳定 ≈1g ⇒ **I2C 读没出错**，排除“读错/字节错位”。
2. 同一笔读取里陀螺却 **28% 饱和到 ±250°/s** 且其余帧带 ±40°/s 偏置 ⇒ **是 MPU6050 的陀螺 MEMS/ADC 部分失效**，
   加速度 MEMS 完好（两者是独立结构，单只坏很常见，尤其这块经历过缺 GND、热插拔）。
3. 结论链：陀螺坏值 → Madgwick（无磁力计）持续积分坏转速 → 姿态角匀速扫过 ±180° → 三轴一起锯齿。
   **E4（Madgwick 公式抄错）与 E6（零偏标定未生效）对错各半，但都不是这次全量数据的真凶**——
   公式已还原正确、标定即使生效也压不住 28% 的饱和尖峰与 ±40°/s 量级坏偏置。

### 修复方案（代码，`attitude.c` `Attitude_Update`，本次实现）
- **(1) 陀螺异常剔除**：本拍任一轴 `raw_gyro` 饱和（\|raw\| ≥ 32000）即判 `gyro_bad`，
  该帧把 `gx/gy/gz` 置 0 再喂 Madgwick ⇒ 退化为“仅靠加速度求姿态”，尖峰不再把角度甩到 ±180°。
- **(2) 在线零偏估计** `s_gyro_bias_est[3]`：仅当“静止代理”成立
  （加速度模 `|amag-1|<0.05g` 且本拍陀螺未饱和）时用慢 leaky 积分（k=0.002）把陀螺均值当零偏扣除；
  运动/饱和时段不更新，避免把真实转速或尖峰当零偏。补偿掉 ±40°/s 量级的恒定坏偏置。
- 两者叠加：静置时姿态由**健康的加速度**兜底，应当稳定；不再锯齿。

### 根因治本（硬件，用户侧）
- **换一块 MPU6050 模块 / 或确认模块 VCC 与去耦**（陀螺部分疑似损坏）。软件兜底只是让“静置调试”能看，
  真要上车平衡、需要动态陀螺，必须硬件好。换模块后建议重新跑 E1 的 I2C 100kHz + 三轮复测。

### 验证步骤（用户侧）
1. Keil 重编 0 错误、烧录。
2. 静置看 L12-L14：应稳定到常量附近（不再 ±180° 绕圈），pitch 围绕某倾角小幅波动即可。
3. 慢转板子：roll/pitch 平滑跟随；yaw 仍缓慢线性漂移（无磁力计，正常）。
4. 若静置仍漂移 ⇒ 基本确认陀螺硬件坏了，按上面换模块。

### 状态
- [x] 全量数据分析（42760 帧），定位真凶为陀螺硬件坏值（推翻 E4/E6 的归因）
- [x] `attitude.c`：陀螺饱和剔除 + 在线零偏估计（本次实现）
- [ ] 用户烧录复测：静置 L12-L14 稳定
- [ ] 换 MPU6050 模块 / 查 VCC 去耦（根因治本）

---

## 事件 E11：在线零偏估计把正常/慢速运动的陀螺信号吃掉——gx/gy/gz 动静止恒为 0（2026-08-20）

### 现象（用户实测）
- VOFA 通道 9~11（滤波后 gx/gy/gz，°/s）在**静止**和**运动（T1+R）**时都≈0，几乎不动。
- 之前 E7 曾怀疑"陀螺硬件坏（28% 饱和）"，但用户按诊断流程验证：**板子放平静止发 `C` 重标定，
  再用手快速猛甩（180°/0.2s≈900°/s），CH9–11 出现很高尖峰（数值超 VOFA 量程看不清，但确为尖峰，仅 2~3 处）**
  → **陀螺是活的**（之前判断"硬件死"被推翻，至少陀螺能输出真实高速率）。
- 甩完之后、以及随后大段 T1/R 运动时间里，CH9–11 又回到"几处尖峰后纹丝不动"。

### 根因
- `Attitude_Update` 的在线零偏估计 `s_gyro_bias_est[]`（`attitude.c` 原 147 行）的"静止代理"**只看
  加速度模 `|amag-1|<0.05g`**。但**纯旋转不改变 |g|**（加速度计测不到角速度），于是板子转动时
  `amag` 仍≈1，bias 估计**仍在更新**，把真实角速度当成"零偏"用 leaky 积分（k=0.002）慢慢扣掉。
- 快甩 ≈900°/s 远超 bias 跟踪速度（τ≈2.5s），尖峰短暂漏出；甩完 bias 重新收敛，随后正常/慢速
  运动（平衡摆动角速度没那么猛）又被吃成 ≈0。这就是"动静止都纹丝不动"的真因——
  **不是硬件死，是软件把运动信号抵消了**。
- 推翻 E7 部分结论：E7 的"28% 饱和"是真实现象（陀螺确实出过饱和坏值），但"陀螺整体硬件死、
  必须换模块"在**本片**被本次验证否定——本片在高速率下能正常响应，慢速段被 bias bug 掩盖。

### 修复方案（代码，`attitude.c`，本次实现）
- 新增宏 `GYRO_BIAS_STILL_DPS = 4.0f`（在线零偏估计"静止"角速度门限，°/s）。
- 静止判定改为**双条件**：`|amag-1|<0.05g` **且** 三轴合角速度 `grate < GYRO_BIAS_STILL_DPS`。
  - 板子 truly still：grate≈0 < 4 → 学零偏（保留治漂移能力）。
  - 板子转动/平衡摆动：grate 超门限 → **不更新 bias** → 真实角速度透传到 CH9–11 与 Madgwick。
- bias 估计职责收敛为"静止时补 Init `C` 标定之外的缓慢漂移"，不再污染运动信号。

### 验证步骤（用户侧）
1. Keil 重编 0 错误、烧录。
2. 板子放平静止发 `C`，静置看 CH9–11：应≈0±小噪声（此时 grate<4 学零偏，输出干净）。
3. 慢转 / 开 T1+R 让车摆动：CH9–11 应**跟随真实角速度变化**（不再恒 0）。
4. 快甩仍应出尖峰（验证未破坏高速率通路）。
5. 若静止 CH9–11 非零且偏大 → 说明 `C` 标定时板子没放静，offset 没捕到；重新静置发 `C`。

### 状态
- [x] 根因定位（bias 静止代理只看 accel 模，旋转不改 |g| 导致把真转速当零偏吃掉）
- [x] `attitude.c`：新增 `GYRO_BIAS_STILL_DPS` 宏 + 静止判定加角速度门限（本次实现）
- [x] 验证推翻 E7"本片陀螺硬件死"判断（快甩出尖峰，陀螺活）
- [ ] 用户重编烧录：静止 CH9–11≈0；慢转/T1+R 时 CH9–11 跟随变化

---

## 事件 E12：raw gyro（CH3–5）恒为 0，gx/gy/gz 也恒为 0（2026-08-20）

### 现象（用户实测）
- VOFA 通道 **I3/I4/I5 = raw_gyro x/y/z 全部 ≈ 0**，即使开 `T1+R` 让车摆动也几乎不动。
- 滤波后 **CH9–11（gx/gy/gz）同样恒为 0**。
- 但 raw accel（I0–I2）正常更新，说明 I2C 链路活着、MPU6050 至少 accelerometer 在工作。

### 根因（待 D 命令最终确认，但已排除 bias bug）
- E11 修复的是"bias 估计把正常运动吃掉"，前提是有真实的非零 raw gyro 进来；
  现在 raw gyro **从根源就是 0**，所以不是 E11 的同一类问题。
- 两种可能：
  1. **软件配置**：某些 MPU6050 clone 在连续 14 字节 burst（ACCEL_XOUT_H → GYRO_ZOUT_L）时，
     gyro 字节会返回 0；或 `PWR_MGMT_2` 陀螺轴被意外置为 standby。
  2. **硬件损坏**：陀螺 MEMS 已死（与 E7 的饱和坏值现象一致，只是现在完全归零）。

### 修复/诊断方案（代码，本次实现）
- **`imu_mpu6050.c` `MPU6050_ReadRaw`**：把 14 字节连续 burst 改为 **accel(6) + gyro(6) 两次独立读取**，
  绕过跨寄存器 burst 在部分 clone 上 gyro 字节返回 0 的问题。
- **`MPU6050_Init`**：显式写 `PWR_MGMT_2 = 0x00`，确保陀螺轴未进待机。
- **新增 `MPU6050_DumpStatus()` + 串口命令 `D`**：
  打印 `WHO_AM_I`、`PWR_MGMT_1/2`、`GYRO_CONFIG`、`ACCEL_CONFIG`、`CONFIG` 及一次 `rawA/rawG` 采样，
  帮助一次性看清是配置错还是芯片坏。

### 验证步骤（用户侧）
1. Keil 重编 0 错误、烧录。
2. 串口发 **`D`** → 观察回显：
   - `pwr1=0x01`（PLL 时钟、未 sleep）、`pwr2=0x00`（无轴待机）→ 配置正确；
   - `gcfg=0x00`（±250°/s）、`acfg=0x00`（±2g）→ 量程正确；
   - `rawG=x,x,x` 仍全 0 → **陀螺硬件坏，需更换 MPU6050**。
3. 若 `D` 显示 rawG 有非零值，但 VOFA 的 I3–I5 仍全 0 → 检查遥测通道映射是否把 37 通道拖对。
4. 开 `T1+R` 摆动：CH9–11 应跟随角速度变化；若仍 0 且 `D` 也 0，结论=硬件。

### 状态
- [x] `MPU6050_ReadRaw` 改为 accel/gyro 分读（本次实现）
- [x] `MPU6050_Init` 显式写 `PWR_MGMT_2=0x00`（本次实现）
- [x] 新增 `D` 诊断命令 + `MPU6050_DumpStatus()`（本次实现）
- [ ] 用户发 `D` 确认是配置问题还是芯片坏

---

## 事件 E14：MPU6050 WHO_AM_I 严格比对 0x68 导致国产新版本（0x70）被 Init 判死（2026-08-20）

### 背景（用户提供的兼容说明）
用户购买的国产 MPU6050 模块为新版本，芯片 `WHO_AM_I` 寄存器返回 **0x70**（老版本为 0x68）。
厂商说明：代码里若写死 `if(id != 0x68) return` 会把新版本直接判死，症状是"检测不到模块"。
除 ID 不同外，新版本寄存器映射/功能/参数与老版本**完全一致**。

### 代码事实（确实存在同款坑）
- `imu_mpu6050.c` `MPU6050_Init()` 第 103 行原逻辑：
  ```c
  if (MPU6050_ReadWhoAmI(&id) != 0 || id != MPU_WHO_AM_I_VAL)  // MPU_WHO_AM_I_VAL=0x68
      return -1;
  ```
  与厂商说明里的 `if(Re != 0x68) return 0` **结构完全一致**。
- 一旦返回 -1，`Attitude_Init` 随之失败，`PWR_MGMT_1=0x01`（唤醒）/`GYRO_CONFIG` 等**全未写**，
  芯片停在默认 **SLEEP 模式（PWR_MGMT_1=0x40）**，accel/gyro 不产出 → 全 0。

### 修复方案（代码，本次实现）
- `imu_mpu6050.h`：新增 `#define MPU_WHO_AM_I_VAL_NEW 0x70`。
- `MPU6050_Init`：判定改为
  - **仅当 `MPU6050_ReadWhoAmI` 返回失败（I2C NACK，器件真的不在）才 `return -1`**；
  - `id == 0x68 || id == 0x70` → 打印 `WHO_AM_I=0x.. ok`，正常继续；
  - 其它异常 ID → 打印 `unexpected, continue anyway`，**仍继续配置**（不死判）。
- 无论老版还是新版国产模块，都不会再被 Init 卡在 SLEEP。

### 诊断逻辑（关键：到底是不是这个坑）
- 用户此前 VOFA 实测：**raw accel（I0–I2）有有效值**（约 0.64/0.01/-0.58，单位 g 矢量且逐帧微变）。
- 若芯片真是 0x70 且 Init 在改前失败 → 整机 SLEEP → accel 也应冻结为 0，**不应出现有效 accel**。
- 因此**大概率 Init 此前已成功**（芯片是 0x68），gyro 全 0 更可能是**陀螺 MEMS 硬件死亡**（与 E7 饱和坏值一脉相承）。
- **但属推断**，最终以 `D` 命令回显的 `who=` 为准：
  - `who=0x68` → Init 本就成功，gyro=0 = 硬件坏，需换模块；
  - `who=0x70` → 改前被判死，本次修复后重测 gyro 是否复活；
  - `who=0x00` / `read=fail` → I2C 根本没连上（查接线/地址/上拉）。

### 状态
- [x] `MPU6050_Init` WHO_AM_I 改为兼容 0x68/0x70（本次实现）
- [ ] 用户重编烧录 + 发 `D`，以 `who=` 区分"配置坑"还是"硬件坏"

---

## 事件 E15：D 命令实测 —— who=0x70 已复活，但 gyro 数据寄存器恒为 0（硬件死）（2026-08-20）

### 实测回显（用户串口发 D）
```
who=0x70 pwr1=0x01 pwr2=0x00 gcfg=0x00 acfg=0x00 cfg=0x03
| rawA=-11540,432,10548 rawG=0,0,0 read=ok
```
### 解读（结论已定）
- `who=0x70` → **E14 兼容修复确已生效**：国产新版本不再被 Init 判死，芯片已正常唤醒并完成全部配置。
- `pwr1=0x01`（PLL+未sleep）、`pwr2=0x00`（无轴待机）、`gcfg=0x00`（±250°/s）、`acfg=0x00`（±2g）、`cfg=0x03`（DLPF）全部正确回读 → **芯片电源/通信/配置链 100% 正常**。
- `rawA=-11540,432,10548` → 加速度计在同一颗芯片、同一套 I2C 读机制下**工作完全正常**（≈0.70/-0.03/-0.64g，重力矢量正确）。
- `rawG=0,0,0 read=ok` → **陀螺数据寄存器返回的就是 0**（read=ok 排除读取失败/总线问题）；且 VOFA 连续数千帧 `I3–I5` 恒为 0。

### 结论
- **陀螺 MEMS / 信号链硬件损坏（DOA）**，与 E7 的"28% 饱和坏值"是同一片陀螺崩坏的连续剧情（先饱和尖峰，最终彻底归零）。
- 国产 0x70 clone 的"加速度计能用、陀螺永久归零"是**经典到手即坏（DOA）签名**，软件（含本次所有修复）无法恢复。
- E14 的推断"可能 Init 本就成功（0x68）"被推翻：实测就是 0x70，且 E14 修复前确实会被判死；之前能读到有效 accel 说明用户当时固件已含部分修复/或处于其它状态，不影响当前结论。

### 处置
- **软件到此为止**：Init/读取/D 命令/滤波/bias 全部正确，无需再改。
- **硬件必须换**：更换 MPU6050 模块。建议优先选 WHO_AM_I=0x68 的真 Invensense 或口碑稳定的供应商，0x70 clone DOA 率偏高；代码已同时兼容 0x68/0x70，换上即用（只要不是 DOA）。
- **换片前**：系统只能靠加速度计做姿态（Madgwick 退化为重力场求姿），能"站"但必抖、微分项为 0，**不具备实车平衡能力，不要继续调 PID**。

### 状态
- [x] E14 兼容修复实测生效（who=0x70 通过 Init、配置全对）
- [x] 陀螺硬件死确诊（rawG 恒 0，read=ok，accel 正常对照）
- [~] 用户已购**正品（非国产原装）MPU6050 并更换 IC**；代码对 0x68/0x70 均兼容、I2C 地址默认 0x68 与正品默认一致，故**无需改代码**。待重烧发 `D` 见 rawG 非零；若 `D` 无回显则新模块 AD0 接高（地址 0x69），需改 `MPU6050_I2C_ADDR=0x69`。
- [ ] 换正品后重测 `D` 应见 rawG 非零；再按 E11 验证 CH9–11 跟随、最后整定 PID

---

## 事件 E19：VOFA 遥测只发一次，之后停；MPU6050 INT 边沿配反（CubeMX 配置分裂）（2026-08-24）

### 现象（用户实测）
- 上电打印 `MPU6050 Init OK` + `System Init Success!` 后，VOFA firewater 帧**只发一次**（首拍信号量初值=1 那帧）。
- 之后波形彻底停；`htim1.Instance->CNT` 仍在涨（TIM1 是 PWM，与遥测触发无关 → 一度误导排查方向）。
- debug：`g_int_cnt`（ISR 内 MPU6050 INT 自增计数）不增长；`EXTI->PR3 = 0`（无 pending）；PC3 引脚恒为高电平。
- NVIC 窗口：`EXTI Line 3` E=1 已使能，抢占优先级=1。

### 调试器定位
- 逐行读全链路：`main.h`(MPU6050_INT_Pin=GPIO_PIN_3/GPIOC) → `gpio.c:94-98`(PC3 配置) → `stm32h7xx_it.c:373-380`(HAL_GPIO_EXTI_Callback 释放信号量) → `freertos.c:204`(信号量初值=1) → `freertos.c:490`(Task_Sensor 100ms 超时 acquire)。
- 链路本身全部正确，代码无 bug。
- **真因**：`gpio.c:96` 配 `GPIO_MODE_IT_RISING`（等上升沿），但 MPU6050 `INT_PIN_CFG` 默认 `0x00`
  → **INT 为 active-low**（data-ready 时把 INT 拉低，PC3 高→低 = 下降沿）。边沿方向反了，中断永远不触发。

### 根因链
1. **触发**：CubeMX 里 PC3 的 GPIO mode 被设为 `External Interrupt Mode with Rising edge trigger`，与 MPU6050 默认 active-low 极性相反。
2. **放大**：`g_semAttitudeDataReadyHandle` 初值=1，Task_Sensor 首拍不阻塞直接发一帧；之后永远拿不到新信号量（acquire 100ms 超时 continue），故遥测停。
3. **隐蔽点**：TIM1 的 CNT 在涨让人误以为"定时源在跑"，实际 TIM1 是电机 PWM，与遥测触发（MPU6050 INT→EXTI3→信号量）是两条独立路径。

### 修复方案（★ CubeMX 优先工作流，见 `AI_deploy_开发规范与提示词.md` §3）
- **不直接手改 gpio.c**，而是走 CubeMX 优先流程：
  1. `Pinout & Configuration → System Core → GPIO → PC3`：`GPIO mode` 改 `External Interrupt Mode with Falling edge trigger detection`；`GPIO Pull-up/Pull-down` 保持 `Pull-up`。
  2. `System Core → NVIC → EXTI Line 3 interrupt`：`Enabled` 勾选；`Preemption Priority` 由 `1` 改为 `6`（数值 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`，ISR 内 `osSemaphoreRelease` 才合法）。
  3. 点 `GENERATE CODE`。
  4. 生成后核对（AI 闸门）：`gpio.c:96`=GPIO_MODE_IT_FALLING、`gpio.c:152`=HAL_NVIC_SetPriority(EXTI3_IRQn,6,0)；`.ioc:444`=GPIO_MODE_IT_FALLING、`.ioc:260`=EXTI3_IRQn…\:6。
- **额外发现**：原 NVIC 优先级=1（数值 < 5），ISR 内调 `osSemaphoreRelease` 属未定义行为；边沿修好后高频进中断会暴露，故一并上调到 6。

### 已证伪的初判假设（诚实归档）
- ~~H1：系统时钟从 75M 改回 480M 导致 TIM7 心跳错乱~~ → SystemCoreClock=480M 正确，CNT 涨说明时钟正常，与遥测无关。
- ~~H2：CubeMX 重生成把 EXTI 配置冲掉~~ → 读 `gpio.c` 发现 EXTI 模式确实存在，只是边沿反了，不是丢失。
- ~~H3：TIM1 是遥测触发源~~ → TIM1 是 PWM，遥测触发源是 MPU6050 INT→EXTI3，两者无关。
- ~~H4：I2C 写 INT_ENABLE 失败~~ → 读代码 MPU6050_EnableInt()=I2C_WriteReg(INT_ENABLE,0x01) 正常，且首帧能发说明链路通顺，根因在 GPIO 边沿。

### 防复发
- 配传感器 INT 脚前，先查该器件 `INT_PIN_CFG` 默认极性（active-high / active-low），GPIO 边沿必须匹配。
- MCU 配置一律 CubeMX 优先：AI 先给 CubeMX 方案并弹提示框等用户生成，生成后再核对 .ioc 与代码一致才继续（已写入范式文档第16–18节）。
- 板子行为故障若源于 CubeMX 配置，修复须走 CubeMX 重生成，禁止手改生成文件（否则下次生成冲掉 + 状态分裂）。

### 状态
- [x] 定位：PC3 上升沿 vs MPU6050 默认 active-low → 边沿反了，中断不触发
- [x] 走 CubeMX 优先流程修复：PC3 改 FALLING + EXTI3 优先级 1→6，GENERATE CODE
- [x] 生成后核对 .ioc 与代码一致（gpio.c:96/152、.ioc:444/260 全部匹配）
  - [x] 证伪 H1/H2/H3/H4
- [x] 防复发条款写入 `AI_deploy_开发规范与提示词.md` §3

---

## 事件 E27：attitude.c 用 `M_PI` 未定义导致编译失败（error: use of undeclared identifier 'M_PI'）【编译期 / 传感器模块】

> 原 `Doc/Keil_MDK_ARM_工程排错记录.md` 问题 6 迁入此处（传感器模块专属编译错）。通用编译/链接期故障归档见 `Error/compile_link_error.md`。

### 现象
Build 报：
```
../Components/BSP/IMU/Src/attitude.c(256): error: use of undeclared identifier 'M_PI'
../Components/BSP/IMU/Src/attitude.c(257): error: use of undeclared identifier 'M_PI'
```
2 Error(s)，0 Warning(s)，axf 未生成。

### 根因
ARM CLANG 的 `math.h` **默认不定义 `M_PI`**（与 GCC/newlib 不同）。`Attitude_Update` 的 §1.2 倾角补偿罗盘用 `(float)M_PI / 180.0f` 把度转弧度，触发未声明标识符。

### 修复（编译器无关，推荐）
- `attitude.c` 宏定义区新增：
  ```c
  #ifndef DEG2RAD
  #define DEG2RAD   0.0174532925f   /* ARM CLANG math.h 不默认定义 M_PI，用字面量规避 */
  #endif
  ```
- §1.2 处 `pr = s_att_raw.pitch * (float)M_PI / 180.0f;` → `pr = s_att_raw.pitch * DEG2RAD;`（roll 同理）。
- 同文件 `BuildMagAlign()` 早已用 `0.0174532925f` 字面量（度→弧），风格统一；全文件不再出现 `M_PI` 依赖。

### 边界 / 易错
- 不要 `#define _USE_MATH_DEFINES`（ARM CLANG 不认）；其它 `.c` 若用到 `M_PI` 同样换成 `DEG2RAD` 或字面量。

### 状态
- [x] 修复实测（2026-08-23）：Rebuild 0 Error / 0 Warning；§1.2 倾角补偿罗盘编译通过。
- [ ] 烧录绕竖轴验证 heading 单调（待用户实测）。
