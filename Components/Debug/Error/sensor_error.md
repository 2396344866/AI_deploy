# SENSOR 任务运行期故障归档（IMU / 姿态 / 陀螺 / I2C）

> 本文件是 **SENSOR 任务**（StartSensorTask + IMU/姿态解算/陀螺）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/Error/Error_Readme_idx.md` 一致）。
> 联调笔记见 `Components/Debug/debug/sensor_debug.md`（待建）；测试方案见 `Components/Debug/Test/sensor_test.md`。
> 编译期故障归 `Components/Debug/Error/Error_Readme_idx.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

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
3. 若 `D` 显示 rawG 有非零值，但 VOFA 的 I9–I11 仍全 0 → 检查遥测通道映射是否把 59 通道拖对。
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
- `rawG=0,0,0 read=ok` → **陀螺数据寄存器返回的就是 0**（read=ok 排除读取失败/总线问题）；且 VOFA 连续数千帧 `I9–I11` 恒为 0。

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

### 修复方案（★ CubeMX 优先工作流：先核对 `.ioc` 与代码一致再改）
- **不直接手改 gpio.c**，而是走 CubeMX 优先流程：
  1. `Pinout & Configuration → System Core → GPIO → PC3`：`GPIO mode` 改 `External Interrupt Mode with Falling edge trigger detection`；`GPIO Pull-up/Pull-down` 保持 `Pull-up`。
  2. `System Core → NVIC → EXTI Line 3 interrupt`：`Enabled` 勾选；`Preemption Priority` 由 `1` 改为 `6`（数值 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`，ISR 内 `osSemaphoreRelease` 才合法）。
  3. 点 `GENERATE CODE`。
  4. 生成后核对（CubeMX 生成后人工确认）：`gpio.c:96`=GPIO_MODE_IT_FALLING、`gpio.c:152`=HAL_NVIC_SetPriority(EXTI3_IRQn,6,0)；`.ioc:444`=GPIO_MODE_IT_FALLING、`.ioc:260`=EXTI3_IRQn…\:6。
- **额外发现**：原 NVIC 优先级=1（数值 < 5），ISR 内调 `osSemaphoreRelease` 属未定义行为；边沿修好后高频进中断会暴露，故一并上调到 6。

### 已证伪的初判假设（诚实归档）
- ~~H1：系统时钟从 75M 改回 480M 导致 TIM7 心跳错乱~~ → SystemCoreClock=480M 正确，CNT 涨说明时钟正常，与遥测无关。
- ~~H2：CubeMX 重生成把 EXTI 配置冲掉~~ → 读 `gpio.c` 发现 EXTI 模式确实存在，只是边沿反了，不是丢失。
- ~~H3：TIM1 是遥测触发源~~ → TIM1 是 PWM，遥测触发源是 MPU6050 INT→EXTI3，两者无关。
- ~~H4：I2C 写 INT_ENABLE 失败~~ → 读代码 MPU6050_EnableInt()=I2C_WriteReg(INT_ENABLE,0x01) 正常，且首帧能发说明链路通顺，根因在 GPIO 边沿。

### 防复发
- 配传感器 INT 脚前，先查该器件 `INT_PIN_CFG` 默认极性（active-high / active-low），GPIO 边沿必须匹配。
- MCU 配置一律 CubeMX 优先：先在 `.ioc` 中完成配置并生成代码，再核对 `.ioc` 与代码一致才继续（已写入工程范式文档）。
- 板子行为故障若源于 CubeMX 配置，修复须走 CubeMX 重生成，禁止手改生成文件（否则下次生成冲掉 + 状态分裂）。

### 状态
- [x] 定位：PC3 上升沿 vs MPU6050 默认 active-low → 边沿反了，中断不触发
- [x] 走 CubeMX 优先流程修复：PC3 改 FALLING + EXTI3 优先级 1→6，GENERATE CODE
- [x] 生成后核对 .ioc 与代码一致（gpio.c:96/152、.ioc:444/260 全部匹配）
  - [x] 证伪 H1/H2/H3/H4
- [x] 防复发条款写入 CubeMX 优先工作流规范（先核对 `.ioc` 与代码一致再改）。

---

## 事件 E27：attitude.c 用 `M_PI` 未定义导致编译失败（error: use of undeclared identifier 'M_PI'）【编译期 / 传感器模块】

> 原 `Components/Debug/Error/Error_Readme_idx.md` 问题 6 迁入此处（传感器模块专属编译错）。通用编译/链接期故障归档见 `Error/compile_link_error.md`。

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

---

## 事件 E38：QMC5883L 磁力计在 aux-bus 接线下必须走 I2C 旁路，原旁路代码未校验/未日志（2026-09-01）

### 现象 / 背景
- 用户实际把 GY-273（QMC5883L）的 SCL/SDA 接到了 **MPU6050 的 XCL/XDA（auxiliary I2C bus）**，而不是 STM32 的 PB8/PB9 主总线。
- 当前为杜邦线临时连接，用户明确：**暂时无法改到 PB8/PB9 主总线，只能走旁路模式（bypass）启动磁力计**。
- 此前 VOFA 遥测（45→59 通道，firewater 抓包）中 **MAG 三通道恒为 0**，即磁力计从未被 STM32 主总线访问到——根因正是 aux-bus 接线 + 未确认旁路是否生效。
- Madgwick 6 轴（无磁）可解算 roll/pitch，但 **yaw 无磁校正会漂移**；heading 锁定依赖磁力计，磁力计可达性依赖旁路。

### 取证（关键）
- 复查 `Components/BSP/IMU/Src/drivers/imu_mpu6050.c`：旁路代码**原本就在 `MPU6050_Init` 里**——
  - `I2C_WriteReg(MPU_REG_USER_CTRL, 0x00)` 关 MPU 内部 I2C master（释放 aux bus）；
  - `I2C_WriteReg(MPU_REG_INT_PIN_CFG, MPU_I2C_BYPASS_EN)` 置 `INT_PIN_CFG[1]=1`，主总线(PB8/PB9) 直通 XCL/XDA；
  - `Attitude_Init` 调用顺序正确：`MPU6050_Init()` → `QMC5822_Init()`。
- **缺口**：原旁路是"盲写"，既没回读校验、也没任何日志；aux bus 上 QMC 到底可达不可达，全靠运气/抓包反推。这就是 VOFA MAG 全 0 却无告警的原因。

### 根因链
1. 硬件：QMC 挂在 MPU aux bus（XCL/XDA），STM32 主总线默认到不了 aux 设备。
2. 软化：旁路若未真正生效（clone 时序/顺序问题），QMC 全 NACK → `QMC5822_Init` 失败 → `ready=0` → 磁通道全 0，且无日志暴露。
3. 后果：yaw 只能陀螺积分漂移；调试时只能从 VOFA 抓包反推"磁没了"，定位慢。

### 修复方案（用户层文件，CubeMX 零影响）
1. **`imu_mpu6050.h`**：新增 `int MPU6050_IsBypassEnabled(void);`（`imu_mpu6050.h:49`）。
2. **`imu_mpu6050.c`**：
   - 新增 `MPU6050_IsBypassEnabled()`（`imu_mpu6050.c:107-111`）：回读 `INT_PIN_CFG[I2C_BYPASS_EN]` 返回 1/0/-1(NACK)。
   - `MPU6050_Init` 旁路写后加 **回读校验 + 日志**（`imu_mpu6050.c:153-170`）：
     - ENABLED → `LOG_I` 确认 QMC 可达；
     - NOT set → `LOG_W` 告警 aux bus 不可达；
     - read-back NACK → `LOG_W` 提示 I2C 不可读。
3. **`mag_qmc5883l.c`**：
   - 新增 `QMC_ADDR_ALT 0x1A`、`QMC_INIT_RETRIES 3`、`static uint8_t s_qmc_addr`（命中后写入）；
   - `qmc_write/qmc_read` 改用 `s_qmc_addr`；新增 `qmc_probe(addr)`；
   - 重写 `QMC5822_Init`（`mag_qmc5883l.c:80-103`）：依次探测 `0x0D`→`0x1A`，命中记 `s_qmc_addr`，配置写失败按 `QMC_INIT_RETRIES` 重试；全 NACK 时 `LOG_E` 提示"查 XCL/XDA 接线 + MPU 旁路"；
   - `QMC5822_DumpStatus` 日志补充 `addr=0x%02X`。
4. **`attitude.c` `Sensor_Test`**（`attitude.c:951-978`，非致命）：
   - 先打 `MPU6050_IsBypassEnabled()` → ENABLED/DISABLED/NACK 三级日志；
   - `QMC5822_IsReady()` 则读原始三轴，打 `raw(X,Y,Z)=.. |mag|=.. counts`，超限（=0 或 >4000²）打 suspicious `LOG_W`；
   - 未 ready 打提示"查旁路+接线，yaw 走陀螺积分"。

### 边界 / 易错
- 旁路前提是 **MPU 内部 I2C master 必须关（`USER_CTRL[I2C_MST_EN]=0`）**，否则 aux bus 被 MPU 占着，主机发不了 QMC。原代码顺序已正确（先 0x6A=0x00 再 0x37=0x02）。
- 部分 MPU clone 对写 `INT_PIN_CFG` 有时序/顺序敏感，故**必须回读校验**，不能以"写了就当生效"。
- QMC 地址有 `0x0D`（常见）与 `0x1A`（少数批次）两版，探测兜底避免批次差异导致永远 NACK。
- 磁力计校验**非致命**：`Sensor_Test` 仅 MPU 失败才 halt；QMC 失败只告警、不影响站立/启动。

### 状态
- [x] 旁路代码确认原本存在，已补回读校验 + 日志（`imu_mpu6050.c`）。
- [x] QMC 初始化加地址探测(0x0D/0x1A) + 重试 + 诊断日志（`mag_qmc5883l.c`）。
- [x] `Sensor_Test` POSTEST 日志细化：旁路状态 + QMC 原始三轴幅值（`attitude.c:951-978`）。
- [x] `attitude.c:979` QMC 块末尾补 sentinel：`Sensor QMC check done (non-fatal; ...)`（即使只看尾段也能确认磁分支跑过）。
- [ ] 重编烧录，看 POSTEST 日志确认 `QMC bypass: ENABLED` + `QMC mag OK raw(...) |mag|=...`（待用户实测）。
- [ ] VOFA 重抓包确认 MAG 三通道不再恒 0（待用户实测）。

### 补充诊断（2026-09-01 用户实测日志问询）
- 用户贴 POSTEST 日志：`[2] Sensor_Test enter (attitude.c:915)` → `[36] Sensor OK (Postest.c:121)`，中间无 MPU-data / QMC 日志。
- 取证：`attitude.c:911` 整函数一体门控（含 QMC 块，无独立 `#if` 屏蔽）；当前源码 915→948→955-976→979 完整，磁分支必执行。
- 结论：**贴文为头尾裁剪**（行首计数 [2]→[36] 差 34，中间有大量日志被省去），QMC 日志落在被裁掉的中间段；非源码缺陷。若全量日志仍无 `Sensor QMC ...` 行，才是未重编烧录的旧固件。
- 处置：加 sentinel 收尾日志，使"磁分支是否执行"在尾段即可判定。

### 补充诊断2（2026-09-01 完整 POSTEST 日志：bypass ENABLED 但 QMC NOT ready）
- 用户给全量日志，关键四行确认旁路已生效：
  `[36][I] (attitude.c:955) Sensor QMC bypass: ENABLED (aux bus reachable via PB8/PB9)`
  `[36][W] (attitude.c:976) Sensor QMC NOT ready, non-fatal ...`
  `[36][I] (attitude.c:979) Sensor QMC check done`
- 反常点：整段日志**无任何 `MAG` / `ATT` 标签的 QMC 初始化日志**。`QMC5822_Init()` 任一分支（成功 `QMC found at addr`/`QMC5883L ready`、失败 `probe NACK`/`config writes failed`/`data all 0xFF`）都会打日志；`Attitude_Init()` 失败也会打 `ATT "QMC5883L init FAIL"`。
- 根因：POSTEST 与 `Attitude_Init` 均在**调度器启动前**运行，模块日志走 Channel A（环形缓冲），在 Logger 任务启动排空前若启动序列过长则**最早的一批 MAG/ATT 初始化日志被覆盖丢失**。故"看不到失败原因"≠"没失败"。
- 真因（软件，时序）：`QMC5822_Init()` 紧跟 `MPU6050_Init()`（仅 5ms 延时）执行，此时 aux-bus 旁路尚未稳，QMC 探测 NACK → `s_ready=0` → `QMC5822_IsReady()` 返回 0 → `NOT ready`。运行到 Sensor_Test 时旁路已完全就绪（`MPU6050_IsBypassEnabled()`=ENABLED）。硬件用户确认 OK、旁路也通，故是启动期旁路就绪时序问题，非硬件/接线。
- 双标志位隐患：融合层读 `attitude.c:s_mag_ready`（Attitude_Init 据 `QMC5822_Init()` 返回值置位），而 `QMC5822_IsReady()` 返回 `mag_qmc5883l.c:s_ready`——两套不同。仅让 `s_ready=1` 不够，须同步 `s_mag_ready` 融合层才用磁（yaw 磁校正/航向锁定）。
- 修复（`attitude.c` Sensor_Test）：
  1. 旁路 ENABLED 但 `!QMC5822_IsReady()` → 调 `QMC5822_Init()` **重新 Init 自愈**一次 + `QMC5822_DumpStatus()` 暴露寄存器状态（ready/ctrl1/who/raw）。
  2. 最终按 `QMC5822_IsReady()` 同步 `s_mag_ready`（1=用磁航向校正；0=退化为陀螺积分漂移）。
- 根因加固（`mag_qmc5883l.c` `QMC5822_Init`）：地址探测前加 `HAL_Delay(10)` 让旁路稳定，尽量使 boot 期即成功，避免误导 FAIL 告警（自愈仍兜底）。
- 待用户实测：重编烧录后应见 `Sensor QMC NOT ready at boot -> re-init now` + `QMC found at addr=0x0D` + `QMC5883L ready` + `Sensor QMC mag OK raw(...) |mag|=...`；VOFA 重抓 MAG 不再恒 0、yaw 不再漂。

### 补充诊断3（2026-09-01 重编实测：旁路 ENABLED 但 QMC 全地址 NACK → 硬件可达性）
- 用户重编烧录后日志：`Sensor QMC bypass: ENABLED` → `re-init now (bypass ENABLED)` → `QMC probe NACK on 0x0D/0x1A` → DumpStatus `addr=0x00 ready=0 who=0x00 ctrl1=0x00 ctrl2=0x00 rst=0x00` → `still NOT ready`。
- 判读：`DumpStatus` 全部 0x00 = **I2C 读无 ACK = aux 总线上无任何设备应答**（若有芯片在总线应答，CHIP_ID 通常 0xFF 而非 0x00）。即：旁路已开(STM32 摸到 aux 引脚)，但 aux 总线无设备响应 → **非软件时序，是 aux 侧硬件可达性问题**。
- 用户硬件描述：`MPU XDA→QMC SDA，MPU XCL→QMC XCL，VCC3.3，GND，DRDY 悬空`。接线方向正确(SDA-XDA / SCL-XCL)；DRDY 悬空无妨。但"上午测试还通过来着" → 同一天由通过变 NACK，几乎锁定**杜邦线临时接法物理不可靠**(某根 VCC/GND/SDA/SCL 松脱) 或 aux 侧缺上拉。
- 代码侧补全漏洞：原探测数组仅 `{0x0D,0x1A}`，但头注释声明可能有 `0x1C`，且未覆盖 HMC 兼容 `0x1E`。已扩为 `{0x0D,0x1A,0x1C,0x1E}` 四候选全试（`mag_qmc5883l.c` `QMC_ADDR_ALT2=0x1C`/`QMC_ADDR_ALT3=0x1E` + 循环 `sizeof(cand)`），失败日志也列出四地址并提示 `power + pullup`。
- 结论：重编后若仍全 NACK（DumpStatus 全 0），则 100% 是硬件：① 杜邦线逐一重插/测通断(XDA↔SDA、XCL↔SCL、3.3V↔VCC、GND↔GND)；② 量 QMC 引脚 VCC-GND≈3.3V；③ 确认 GY-273 模块 SDA/SCL 有板载 4.7k 上拉（无则 aux 总线浮空→NACK，需补上拉或改回 PB8/PB9 主总线）；④ 若仍 NACK 且上午能过，优先怀疑杜邦接触。
- 待用户：按上述清单排查硬件；若补 0x1C/0x1E 后仍全 NACK，贴 DumpStatus 行即可定位是供电/上拉/接触。

### 补充诊断4（2026-09-01 硬件复检仍全 NACK → 加 I2C 总线扫描取证）
- 用户复检：杜邦重插、通断 OK、VCC≈3.3V、GY-273 板载 4.7k 上拉齐全（声称符合要求）。但重编后日志仍 `QMC probe NACK on 0x0D/0x1A/0x1C/0x1E` + `DumpStatus addr=0 who=0 ctrl1=0 ctrl2=0 rst=0` → 全地址 NACK。
- 软件已无罪：旁路 `ENABLED` 确认、四地址探测+重试+10ms+自愈+DumpStatus 全在；失败=aux 总线无器件应答（NACK），非代码 bug。
- 两大剩余根因（无法靠猜区分）：(1) 该 MPU6050 旁路 mux 未真正把 XDA/XCL 透传到主总线（位为1但内部开关未通，兼容片常见）；(2) QMC 实际未接在 XDA/XCL（MPU 排针侧松/错脚，QMC 端 VCC 量到≠SDA/SCL 到位）。
- 加决定性取证：`MPU6050_ScanBus()`（`imu_mpu6050.c`，声明 `imu_mpu6050.h`）扫 0x08..0x77，打出所有 ACK 从机；在 `attitude.c` Sensor_Test 的 `!QMC5822_IsReady()` 分支（DumpStatus 后）调用。
  - 结果判读：仅 `0x68`(MPU) 无 `0x0D` → QMC 不在总线（旁路未透传/接线）；`0x68`+`0x0D` 都在 → 探测逻辑 bug（概率低）。
- 待用户实测贴扫描结果；若确认旁路未透传且杜邦难改到 PB8/PB9，则正路是切 **MPU6050 I2C Master 模式**（MPU 内部 master 读 QMC、数据放外部 sensor 寄存器、STM32 从 MPU 读），不依赖旁路 mux。
- 工具澄清：POSTEST/QMC 文本日志走 UART LOG 通道(USART1 921600)，用串口助手看；VOFA+ 仅看 59 路 firewater 波形，不显文本日志。

### 结案（2026-09-01 重编实测：QMC mag OK，确认是硬件接触不良）
- 用户最终日志：`Sensor QMC bypass: ENABLED` → `Sensor QMC mag OK raw(X,Y,Z)=1881,2181,475 |mag|=2919 counts` → `Sensor QMC check done`。**直接 mag OK、未走 re-init 分支** → 启动期 `QMC5822_Init()` 成功，硬件这次接实。
- `|mag|=2919 counts` 为真实地磁读数（2G 量程下地球磁场量级），非全0/全FF。
- 结论：此前四地址全 NACK + DumpStatus 全0 = **杜邦线未插实/接触不良的纯物理层问题，非软件 bug**。所有软件手段（旁路回读/四地址探测/自愈/DumpStatus/总线扫描）均正确，仅当时硬件未接通。
- 生效：s_mag_ready=1（代码在 IsReady 真时同步）→ 融合层 yaw 走磁航向校正，航向锁定生效、不再纯陀螺漂移；VOFA MAG 三通道应不再恒0、yaw 应稳定。
- 处置建议：MPU6050_ScanBus 仅失败时触发，留着无害；四地址探测/DumpStatus/sentinel 建议保留作工业级排查基线。
- **状态：E38 已结案（根因=aux 总线杜邦接触不良，接实即恢复；软件链路全部验证正确）**。

## 事件 E39 — 航向保持 engage 会输出满舵阶跃（yaw_ref 默认 0 未对齐当前航向）

- **日期**：2026-09-01（59ch 遥测第二轮抓包后，用户要求"你来执行"）
- **现象**：CH58 `hdg_err` 长期显示 **−65.698°**（静止、未启用航向保持时）；同时 `yaw_ref(43)=0`、`kp/ki/kd=6/0/0`。
- **根因**：`Attitude_GetHeadingErr(target)`（`attitude.c:519`）在 `s_mag_ready` 时取 `s_mag_heading`，
  返回 `target - cur`。而 `g_att_ref[ATT_YAW]`（yaw_ref）**上电默认 0、从未初始化为当前航向**，
  故 `hdg_err = 0 − mag_hdg ≈ −65.7°`。该值在 `g_k_yaw=0` 时不参与控制，本无害；
  但一旦下发 `H<k>` 使 `k_yaw>0`，`RunController`（`attitude.c:793`）直接算
  `g_steer = k_yaw × hdg_err` —— **整段绝对航向误差瞬间变成转向指令**，经 ±200 限幅即满舵，
  表现为"一开航向保持就猛甩一下"。
- **为什么必须自动对齐**：航向是绝对量（磁北参考），开机朝向随机。要求使用者每次先读 mag_hdg
  再手填 `P<r>,<p>,<y>` 不现实，且填错即甩尾。
- **修复（已实施）**：
  1. `attitude.c` 新增全局 `g_yaw_ref_snapped`（engage 对齐标志）。
  2. 新增 `Attitude_SnapYawRefToCurrent()`：把 yaw_ref 设为 `s_mag_ready ? s_mag_heading : s_att.yaw`
     —— **基准量与 `Attitude_GetHeadingErr` 内部取值完全一致**，保证 snap 后 `hdg_err` 恰为 0。
  3. `Attitude_SetHeadingK(k)`：在 `k` 由 `0 → 正` 的**上升沿**且 `s_mag_ready` 时自动 snap；
     `k <= 0` 时复位标志（下次再开重新对齐）。
  4. `RunController` 航向保持分支加兜底：若 `!g_yaw_ref_snapped` 则先 snap 再算 steer，
     覆盖"`H` 在 mag 就绪前下发（彼时无有效航向可对齐）"的场景。
  5. `attitude.h` 导出 `Attitude_SnapYawRefToCurrent()` / `Attitude_IsYawRefSnapped()`，可手动重新定航向。
- **验证方法**：下发 `H<k>` 前后看 CH58 `hdg_err` —— snap 生效后应从 ≈−65.7° **跳到 ≈0°**，
  且 `steer(57)` 平滑起转、无满舵阶跃；日志出现
  `yaw_ref snapped to current heading XX.XX deg (hdg_err->0, engage without step)`。
- **备注**：本问题**不是**滤波/融合故障 —— 同期实测 `yaw_innov` 峰值 0.006°、
  重力方向自洽误差 0.0016（归一化），融合层完全健康。属控制指令层的初始化缺陷。
- **✅ 验收通过（2026-09-02）**：用户烧录含 E41 路由修复的固件后实测 ——
  下发 `H 0.5` 后 **CH43 `yaw_ref` 由 `0.000` → `310.xxx`**（= 当前 mag_hdg，snap 命中），
  **CH58 `hdg_err` 由 ≈50 → `±0.001` 内波动**，无满舵阶跃。
  两项判据全部达成 → **E39 结案**。
- **补充**：`±0.001` 的残差即"磁噪声经 snap 后的稳态误差"，量级远小于
  `mag_hdg` 本身的 p-p 0.195°，说明 snap 后的航向锁定由融合 yaw 主导、磁仅作慢校正，符合设计。
- **状态**：**已结案**（E39 + E41 一并验收）。

## 事件 E40 — CH54 `loop_ms` 呈 9/10/11 周期性起伏（观测型，非故障）（2026-09-02）

- **日期**：2026-09-02（59ch 遥测第三轮：用户报"i54 周期性出现波峰波谷 11 10 9，又有一段时间恒定 10"）
- **定性结论**：**不是故障，也不需要为它改控制/融合代码**。它是「1 ms 时基量化 + 传感器中断与 MCU
  时钟不同源 + 阻塞发送把节拍撑爆」三者叠加的**观测现象**，工程上可解释、可预测。
- **现象**：`loop_ms(54)` 名义 10 ms，实际在 9 / 10 / 11 之间跳；图案呈**缓慢周期性**——
  一段时间是"11→10→9→10→…"的起伏，另一段时间又稳定在 10。

### 根因（三层叠加，按贡献排序）

1. **1 ms 量化锯齿 + 拍频（解释"周期性"）**
   - Sensor 任务**不是** `osDelayUntil` 固定周期，而是**阻塞等 MPU6050 data-ready 中断**：
     `freertos.c:691` `osSemaphoreAcquire(g_semAttitudeDataReadyHandle, 100U)`。
   - 节拍由 MPU6050 给出：`MPU_SMPLRT_DIV=4` → `1000/(1+4)=200 Hz`（`imu_mpu6050.h:44`，
     `CONFIG=0x03` 即 DLPF 使能、内部 1 kHz 分频）。该时钟源与 STM32 的 1 ms SysTick **不同源**，
     必然存在频差（MPU6050 内部/陀螺 PLL 时钟 vs H743 外部晶振，典型 ±0.5~1 %）。
   - → 采样时刻相对 tick 边界的相位 φ **持续漂移**；而 `dt` 用 `HAL_GetTick()`（**1 ms 分辨率**）
     相减（`dbg_telemetry.c:175-178`）。真值 9.9x / 10.0x 只能被读成 9 / 10 / 11，
     且图案随 φ 漂移周期性变化 —— 正是"一阵起伏、一阵恒定"的**量化锯齿 + 拍频**特征。
2. **阻塞发送把单拍撑爆（解释 11 的能量来源）**
   - 单帧实测 406 B，`SendPoll` 逐字节等 TXE：`406 B ÷ (921600/10) ≈ 4.41 ms` 忙等。
   - 带发送那一拍总耗时 ≈ I2C 读 0.5~1 ms + 计算 0.3 ms + 4.41 ms ≈ **5.2~5.7 ms > 5 ms 采样周期**
     → 该拍期间必有一个 data-ready 中断被"积压"（计数信号量 +1），下一拍 `acquire` **立即返回**（背靠背补跑）。
   - 结果：真实拍序列不是均匀的 5/5/5，而是"等 5 ms → 跑 → 立即跑 → 等 5 ms…"的**爆发式**。
     2 拍窗口（DECIMATE=2）内总量守恒（均值仍 10 ms），但边界相位被推挤，放大 ±1 tick。
3. **采样点位置被污染（解释为什么这个数不能当周期用）**
   - CH54 采样点在**任务体中段**（`dbg_telemetry.c:173`，SYSTEM 组），即
     "唤醒 → I2C 读 → 滤波 → 融合 → 外环 PID → 自治 FSM"之后。
     `dt = 拍间隔 + (τ_i − τ_{i−1})`，混入了这段执行时间 τ 的**逐拍差值**。
   - τ 的扰动源：`TIM7` 1 ms ISR（电机速度/位置闭环 + IWDG 心跳判决，见 `freertos.c:433` 注释）、
     MPU6050 EXTI ISR、`Task_Motor`(High, `osDelay(100)`)、`Task_Test`(High1)、
     `Task_Esp32S3`(与 Sensor 同为 AboveNormal，时间片轮转)。
     （`Task_Flash`(Low1)/`Task_logger`(Low2) 优先级更低，不抢占 Sensor。）

### 影响评估（为什么不用修）

- **不进入姿态解算**：`attitude.c:259` `dt = 1/ATTITUDE_RATE_HZ`、`:341` `ydt = 1/ATTITUDE_RATE_HZ`、
  `Madgwick_Init(ATTITUDE_RATE_HZ)` 全部用**写死的 5 ms**，从不读 `loop_ms`。
  中断驱动 + 固定步长恰是正确的工业做法（中断长期严格 200 Hz，写死步长比用抖动实测值更准）。
- **没有丢采样**：data-ready 中断照常 Give，信号量累积后补跑，长期无样本丢失。
  同期 `yaw_innov` 峰值 0.006°、姿态平稳，印证无实时性塌陷。
- **唯一需盯的边界**：单拍耗时不得 > 2×5 ms。当前 4.41 ms 有 ~2.3 倍余量；
  **若以后再扩帧宽**（帧长 > ~900 B → SendPoll > 10 ms），信号量会持续积压、补跑追不上，
  表现为 `loop_ms` 出现 15/20 且控制环滞后 —— 扩通道时必须重算（同 `logger_error.md` E18 带宽结论）。

---

## 事件 E41 — `H`(航向保持)命令完全无效：分发表漏配路由 + TRACE 级静音吞掉回执（2026-09-02）

- **日期**：2026-09-02（E39 修复后的首轮实测验收）
- **现象**（三个独立表现，曾一度被误判为"用户命令格式错"）：
  1. 串口发 `H` / `H 0.5` **无任何文本响应**（只有终端本地回显）；
  2. **CH43 `yaw_ref` 恒为 `0.000`**（12 帧连续全 0，一次都没变过）；
  3. **CH58 `hdg_err` 恒 ≈ 49.9~50.2**，逐帧严格等于 `360 − CH34(mag_hdg)`。

### 根因一：控制台分发表漏配 `'H'`（主因，命令从未到达 handler）

- handler **存在**：`attitude.c:745` `else if (op == 'H')` 完整实现了查询/设置/ACTIVE 判定。
- 但**路由不存在**：`freertos.c:150` 的单消费者分发表只列举
  `T / P / K / C / F / D / M`，**没有 `H`** → 落在 unknown 分支被**静默丢弃**，
  `Attitude_ProcessCommand()` 一次都没被调用 → `Attitude_SetHeadingK()` 从未执行 →
  E39 的 snap 逻辑**写得对但不可达**。
- 文档同步缺失：`freertos.c:94` 注释同样写的是 `T/P/K/C/F/D/M -> Attitude_ProcessCommand`，
  连注释都漏了 H（代码与注释"一致地错"，审查时更难发现）。
- **历史成因**：`Attitude_ProcessCommand` 早期由 `StartMotorTask` 路由（旧 map 可证），
  后迁到常驻 `StartLoggerTask`（`freertos.c:88-89` 注释），路由表迁移时漏带新增的 H。

### 根因二：TRACE 级遥测静音吞掉 `LOG_I` 回执（次因，导致人工误判）

- `dbg_telemetry.c:101` 在**运行级 ≥ TRACE** 时把门限抬到 WARN：
  `logger_set_uart1_text_mute_level(logger_get_level() >= LOG_LVL_TRACE ? LOG_LVL_WARN : 0xFF)`。
- `logger.c:139` 判定：`if (level > g_uart1_text_mute_above) return;`
  级别 FATAL0/ERROR1/**WARN2**/**INFO3**/DEBUG4/TRACE5 → **INFO 及以上全部丢弃**。
- 而所有命令回执（含 `heading_hold k_yaw=...` / `yaw_ref snapped to ...`）都是 `LOG_I`。
- → **即使路由修好，VOFA 遥测运行时也看不到任何命令文本回执**。
  这是"有回显但没其他输出"的另一半解释，且与根因一相互掩盖。

### 判据（关键：别用文本回执判断命令是否生效）

- **唯一硬证据 = CH43 `yaw_ref` 是否变化**：`0` → `≈ mag_hdg`（当前 ≈310）。
  它由 snap 直接写入 `g_att_ref[ATT_YAW]`，与日志静音无关。
- 辅助证据 = **CH58 `hdg_err`**：`≈50` → `≈0`。
- **反证 `s_mag_ready=1`**（排除"磁未就绪导致不 snap"）：
  实测 CH58 = `360 − CH34` 严格成立（360−310.054=49.946 ✓），
  说明 `Attitude_GetHeadingErr()` 走的是 `s_mag_heading` 分支 → `s_mag_ready` 必为 1。
  故只要命令进得去 handler，snap 必然触发——CH43 恒为 0 直接证明命令没进去。

### 修复（已落地）

- `freertos.c:152-155` 分发表补 `|| lbuf[0] == 'H'`，并加注"E41 曾漏配路由"。
- `freertos.c:94-96` 注释同步为 `T/P/K/C/F/D/M/H`，并加**警示**：新增命令必须同时改注释与分发表。
- 安全性：该段位于 `USER CODE BEGIN Variables`(67) … `END Variables`(180) 内，CubeMX 重生成不覆盖。

### 验证 SOP（修后）

1. **重编烧录**（必做，否则跑的仍是旧固件）。
2. TRACE 级（VOFA 运行中）发 `H 0.5` → **不要看文本**，看 **CH43：0 → ≈310**、**CH58：≈50 → ≈0**。
3. 若想读文本回执：先发 `debug4`（降到 DEBUG → 遥测停 + 静音解除）→ 发 `H 0.5` →
   应见 `yaw_ref snapped to current heading 309.xx deg` 与 `heading_hold k_yaw=0.50 ACTIVE`
   → 再发 `debug5` 恢复遥测。

### 教训 / 预防

- **单消费者分发表 + 模块内 handler 的架构下，"新增一条命令"是【两处改动】而非一处**，
  且漏改时**无任何编译/运行期报错**，属最隐蔽的静默失效。
- 建议（未实施）：① 给 unknown 分支加 `LOG_W("unknown cmd")` 回执，让漏配立刻暴露；
  ② 命令回执改用 `BSP_LOG_UART1_SendPoll` 直发（同 `debug<n>` 的做法）绕过静音门限。

### ✅ 验收通过（2026-09-02）

- 用户重编烧录后实测：**CH43 `yaw_ref` → `310.xxx`**、**CH58 `hdg_err` → `±0.001`**，
  两项硬判据全部达成 → 根因一（漏配路由）已证伪为唯一主因并修复到位。
- **根因二（TRACE 静音吞 LOG_I）的成因也已由用户侧代码确认**：在
  `LOG_RUNTIME_DEFAULT_LEVEL >= LOG_LVL_TRACE` 成立的分支下，`DBG_LOG_SENSOR` 被定义为 1，
  遥测路径与文本日志路径共用 UART1，故 `dbg_telemetry.c:101` 主动抬高静音门限以保遥测带宽；
  **该行为是设计意图，不是 bug** —— 真正的 bug 只有根因一。
  → 结论：**"看不到回执"属预期行为，判断命令生效只能看通道值**（见上方"判据"一节）。
- **状态**：**已结案**。

### 可选改进（均未实施，待用户拍板；按性价比排序）

1. **采样点前移（零风险）**：把 CH54 的 `HAL_GetTick()` 从 SYSTEM 组移到 Sensor 任务循环
   **最开头**（`acquire` 成功后第一行），`dt` 才只反映中断节奏，不再被 τ 污染。
2. **换 µs 时基（低成本，唯一能证实本结论的手段）**：`HAL_GetTick` 1 ms 太粗；改用
   `DWT_CYCCNT`（H743 有，≈2.08 ns/tick）或 TIM2/5 32 位计数，输出 `loop_us`（或小数 ms）。
   一眼可区分"量化假象（9.95 ms）"与"真抖动（9.2 ms）"。
3. **治本**：`SendPoll` → DMA/中断 + 环形缓冲（E18 已记为根治项，尚未做）。做完后第 2 层抖动消失，
   `loop_ms` 会明显收敛到 10。
4. **不要做**：给 Sensor 任务加 `osDelayUntil` —— 与中断驱动模型冲突，反而会引入丢拍。

- **判读速查**：`loop_ms` 只在 **9/10/11** 之间波动 → 正常，忽略；出现 **15/20** 或长期 **>12**
  → 真实时性塌陷，查信号量积压/发送耗时/抢占。
- **同步更正**：`dbg_config.h` CH54 注释与 `vofa_panel.json` 的 `note_loop_ms`（原注释写作
  "实测 10 ms 反推任务周期 5 ms(200 Hz)，符合预期"，**暗示任务为固定延时周期**，与中断驱动的代码
  事实不符，已改为三重失真说明）。
- **状态**：判明为观测现象，**无需修复**；留作遥测判读知识，避免日后重复排查。

---

## 事件 E42 — 运行时发 `C`(陀螺零偏标定)持续 `gyro cal skipped`：I2C1 总线无互斥锁，Sensor 任务与控制台命令争用同一 hi2c1（2026-09-01）

- **日期**：2026-09-01（串级 PID 调参期间，用户实测"上电 C 能过、运行时连发 C 必 skipped"）
- **现象**：
  1. **上电后第一次 `C` 标定常成功**；进入正常运行（Sensor 任务 200 Hz 跑起来）后，串口连发
     `C C C C` 几乎必出 `CAL SKIPPED` / `CAL FAIL: I2C read only N/100 ok`。
  2. `C C C C` 之后**回显要等很久才回来**（每次碰撞触发 BusRecovery 的 `HAL_I2C_DeInit+ReInit` 极慢）。
  3. 被**误判过一轮**为"杜邦线 I2C 物理掉包"（E38 是真实接触不良，但本事件与物理层无关——见下方根因）。

### 根因：I2C1 全程无互斥锁 + 任务优先级不对称 + BusRecovery 破坏性 DeInit/ReInit

- **无锁**：`hi2c1` 全程裸调 HAL I2C。全仓 grep 仅 `InferenceDataMutex`（与 I2C 无关），**没有任何 I2C 锁**。
  `Sensor(AboveNormal,200Hz)` 与 `C/M` 命令（经 `StartLoggerTask`→`DbgConsole_Process`→
  `Attitude_ProcessCommand`→`Attitude_CalibrateGyroStatic`，`osPriorityLow2`）**共用同一 `hi2c1`**。
- **非可重入碰撞**：HAL I2C 句柄 `hi2c1` 非可重入。高优先级 Sensor 可抢占低优先级 Logger 在途的
  I2C 事务（C 标定的 100 次读），导致 HAL 内部状态错乱、读失败。
- **BusRecovery 雪崩**：`I2C_ReadBuf/ReadReg/WriteReg` 失败即调 `MPU6050_I2C_BusRecovery()` →
  `HAL_I2C_DeInit(&hi2c1)+HAL_I2C_Init(&hi2c1)`。这会把**另一任务正在用的共享句柄从底层拆掉**，
  标定的大量读被反复打断/碰撞 → `good<50` → skipped；且每次 DeInit/ReInit 很慢 → 回显延迟。
- **判别点吻合**：上电 `Attitude_Init` 在**调度器启动前**调标定 → 无并发 → 通常成功；
  运行时发 C → Sensor 已跑 → Logger 的 100 读被频繁打断/碰撞 → skipped。**真因是总线争用，非杜邦线**。

### 修复（方案 A，CubeMX 优先）：I2C1 互斥锁包住所有 hi2c1 访问

- **锁对象由 CubeMX 生成**（不手改 freertos.c 生成区）：在 `.ioc` 的
  `FREERTOS.Mutexes01` 追加 `i2c1_mutex,Dynamic,NULL,Available` → Generate Code 产生
  `osMutexId_t i2c1_mutexHandle` + `i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes)`，
  **重生成不丢**。
- **加锁接口**（`imu_mpu6050.h` / `imu_mpu6050.c`，用户层逻辑，非生成区）：
  `MPU6050_I2C_Lock(int block)` / `MPU6050_I2C_Unlock(int locked)`，含**内核态守卫**
  （`osKernelGetState()!=osKernelRunning` 或锁未创建 → 直接返回 0 不锁，预调度单线程安全）。
- **per-transaction 加锁**（非整段持锁，避免非递归 mutex 同任务二次加锁死锁）：
  `I2C_WriteReg` / `I2C_ReadReg` / `I2C_ReadBuf` 每笔事务 `Lock→HAL→Unlock`；
  `block!=0` 阻塞等锁(`osWaitForever`)，`block==0` 仅尝试一次(0 超时)。
- **调用方 block 策略**：
  - `StartSensorTask`：`MPU6050_ReadRaw(ra,rg,...,**0**)`，抢不到锁即 `continue` 丢一帧
    （标定期间偶丢几帧可接受，不反转优先级、不阻塞）。
  - `Attitude_CalibrateGyroStatic`(C 命令/Logger 上下文)、`Attitude_Init`、`Sensor_Test`、
    `MPU6050_DumpStatus`(D)、`MPU6050_Init`、`MPU6050_EnableInt`、`MPU6050_ScanBus`：
    全部 `block=1`，**阻塞等锁跑完**。
  - `QMC5822_ReadRaw` 在 `Attitude_Update`(Sensor 热路径) 传 `0`；`QMC5822_DumpStatus`(M 命令)、
    `QMC5822_Init`、`Sensor_Test` 传 `1`。QMC 与 MPU 共用 `hi2c1`，**同源加锁**（mag 直接调
    `MPU6050_I2C_Lock/Unlock`）彻底封住 QMC↔Sensor 争用窗口。
- **BusRecovery 不再重复加锁**：只在已持锁的事务内调用，自身 `Lock/Unlock` 由外层负责。

### 验证 SOP（修后）

1. **重编烧录前先 CubeMX Generate Code**（`.ioc` 已改，必须生成 `i2c1_mutexHandle` 否则链接失败）。
2. 上电后进入运行态（Sensor 200 Hz 跑起来），连发 `C C C C` → 应**全部 `CAL OK`**，
   且回显即时（不再有 BusRecovery 雪崩延迟）。
3. 运行时发 `M`(DumpStatus) / `D`(MPU DumpStatus) 同时观察 VOFA 波形：Sensor 偶丢 1 帧（不可见级），
   无 `gyro cal skipped`、无 `CAL FAIL: I2C read only N/100`。
4. 极端压测（可选）：运行时长跑 `C` + 频繁 `M` 交替，确认无 HardFault / 无总线锁死。

### 教训 / 预防

- **共享外设句柄（hi2c1/hi2c*/SPI/UART）在 RTOS 多任务下必须配互斥锁**，哪怕"只有两个地方用"；
  HAL 句柄非可重入，优先级不对称会瞬间暴露。
- **per-transaction 加锁优于整段持锁**：避免非递归 mutex 自死锁，且把争用窗口压到单笔事务。
- **BusRecovery 是"核选项"**：`DeInit/ReInit` 共享句柄会殃及并发任务，必须在持有总线锁的前提下调用，
  且不应作为常规重试手段（本事件里它把碰撞放大成雪崩）。
- **判别物理层 vs 总线争用**：上电成功+运行时失败 = 典型争用；随机 NACK 且与任务并发相关 = 争用；
  物理掉包通常上电也大概率失败。E38(接触不良) 与 E42(无锁) 表象相似但根因不同。

- **状态**：**已修复 + 验收通过（2026-09-02）**。

### ✅ 验收通过（2026-09-02，用户实测）

- 用户按 SOP 操作：**CubeMX Generate Code**（物化 `i2c1_mutexHandle`）+ Keil 0 警告 rebuild + 烧录。
- 进入运行态（Sensor 200Hz 跑起来）后连发 `C C C C` → **全部 `CAL OK`**（含运行时抢占场景），
  无 `gyro cal skipped`、无 `CAL FAIL: I2C read only N/100`、回显即时（BusRecovery 雪崩消失）。
  → **I2C1 互斥锁修复生效，运行时 C 标定的总线争用根因彻底消除**。
- **关键旁证（澄清"输入 C 无回显"的误解）**：验收前用户曾反馈"输入 `C` 只回显 `C`、板子无任何输出"，
  怀疑配置/代码错。经取证为 **E41 根因二（TRACE 静音吞 LOG_I）**：
  - 固件默认 `LOG_RUNTIME_DEFAULT_LEVEL = LOG_LVL_TRACE`（`logger.h:49`）→ 运行即 TRACE；
    `dbg_telemetry.c:101` 运行级≥TRACE 把 UART1 文本静音门限抬到 WARN → `logger.c:139` 丢弃所有
    `level>WARN` 文本（INFO/DEBUG/TRACE 全吞）。
  - `C` 标定**成功** 打的是 `LOG_I("CAL OK...")`（`attitude.c:158` 标定函数内 **与** `:664` case 'C' 分派处，
    **两处都是 INFO**）→ 在 TRACE 下**全部被静音** → 用户"啥也看不到"。
  - 标定**失败** 打的是 `LOG_W("CAL SKIPPED"/"CAL FAIL")` → WARN 不过滤 → 可见。
  - → **"无输出" = 标定成功 + 静音**，恰恰证明互斥锁修复后 `C` 已能跑完；若仍像旧固件那样争用失败，
    反而会打 WARN 级 `gyro cal skipped` 可见。**这不是回归，是修复生效的证据。**
- 验证法（已与用户对齐）：先发 `debug4`（降 DEBUG → 遥测停 + 静音解除），再发 `C` →
  直接见 `CAL OK: zero-bias(dps)=...` / `CAL OK: gyro zero-bias recalibrated...`；完事发 `debug5` 恢复遥测。
  用户按此法实测得到 `[909][I][ATT] (attitude.c:158) CAL O...` 与 `(attitude.c:664) CAL O...` 两条 CAL OK，确认无误。
- **通用判据（跨命令）**：`C/M/D/H` 等靠 `LOG_I` 回执的命令在 TRACE 下文本全不可见，
  判断命令是否生效**只能看 VOFA 通道值**（E41 已立此判据）。本事件进一步坐实该判据对 `C` 同样成立。
- **可选代码改动（未实施，待拍板）**：若要让 `C` 成功信息在 TRACE 下也常显，把 `attitude.c:158` 与 `:664`
  的 `LOG_I` 改 `LOG_W`；但会破坏静音设计（遥测波形掺文本），不推荐。

## 事件 E43 — 串口命令集双轨重构暴露三类缺陷：① 路由层漏配**小写**长名前缀 → `att.*/mag.*/filt.*` 全部不可达；② `K`/`T` 无参经 `atof("")/atoi("")` 归零 → **增益清零 / 静默关外环**；③ 归一化输出丢分隔符（2026-09-02）

- **日期**：2026-09-02（用户拍板「双轨并存 + 三条公约 + 63 B」重构后的实施与自检阶段）
- **触发**：把旧单键（`T/P/K/C/F/D/M/H`、`A/B/S/R`、`X/n/debug<n>`）扩建为 `域.动作` 长名
  （`att.*`/`mag.*`/`filt.*`/`mot.*`/`sys.*`），长名在入口**原地归一化为旧键**后复用既有解析链。
- **发现方式**：**离线复刻两段归一化算法 + 路由分发表**跑 50 条用例（旧键 24 / 长名 24 / 负例 7），
  未烧录即暴露。→ **该手段值得固化为惯例：纯字符串/路由逻辑改完先离线跑用例表。**

### 根因一：路由层只认**大写**单键，长名是**小写** → 归一化在 handler 内，路由阶段看不到（致命）

- `freertos.c` 的分发表按 `lbuf[0]` 匹配**大写** `T/P/K/C/F/D/M/H` 才调 `Attitude_ProcessCommand()`。
- 而 `att.ref` / `mag` / `filt` 以**小写** `a/m/f` 开头 —— 归一化发生在 `Attitude_ProcessCommand()` **内部**
  （`att_cmd_normalize()`，与姿态语义同模块），**路由阶段看到的仍是小写** →
  全部落进链尾 `else`，报 `> ?unknown cmd`。**所有 `att.*/mag.*/filt.*` 一个都进不去。**
- 这是 **E41 的同一个坑第二次出现**（E41 = `H` 漏配路由），只是这次是**大小写维度**而非「漏列字母」。
  说明「新增命令必须改路由表」这条靠人记是不靠谱的，必须在结构上约束。
- **修复**：分发表补一条**精确前缀**路由（`att` / `mag` / `filt` + token 边界 `'\0'/空格/Tab/'.'`）。
  ⚠ **不可写成 `lbuf[0]=='a'||'m'||'f'`**：`mot` 会被 `'m'` 抢走送进姿态模块 → 报 `unknown att cmd`。
  ⚠ 该分支必须排在 `mot` / `sys` 分支**之后**（虽然前缀互斥，但顺序上留出安全余量并便于阅读）。

### 根因二：「无参即 `atof("")/atoi("")` = 0」的写命令，在「无参=查询」语义下变成**破坏性操作**

| 命令 | 旧行为（危险） | 后果 |
|---|---|---|
| `K`（即 `att.pid`） | `atof("")==0` → `Attitude_SetGains(0,0,0)` | **kp/ki/kd 全部清零 → 姿态外环失效/失衡** |
| 裸 `T` | `atoi("")==0` → `Attitude_SetEnable(0)` | **静默关闭姿态外环** |

- 用户按公约①输入 `att.pid` 本意是**查**增益，实际是**清零**——这是重构引入公约后才暴露的语义冲突
  （旧单键时代没人会裸发 `K` 去查询，因为旧文档没承诺「无参=查询」）。
- **修复**：`K` 与 `T` 无参分支改为查询并 `LOG_I` 打印当前值；带参才写入。
- **一般教训（可复用检查项）**：凡是 `atoi/atof(&cmd[1])` 且未判空串的写命令，
  在「无参=查询」的命令体系里都是隐患。同类待查：`A`/`B`（无参 → 目标速度 0 = 停车，语义上可接受，保留）。

### 根因三：归一化丢分隔符（当前可跑，但脆弱）

- `mag init` → `"Minit"`、`filt lag 1 0.3` → `"Flag 1 0.3"`（**不是** `"M init"` / `"F lag 1 0.3"`）。
- **当时能跑**：各分支入口都有 `while (*p==' ') p++;` 跳前导空格，所以 `"Minit"` 的 `q` 仍落到 `"init"`。
- **但脆弱**：一旦下游新增不跳空格的分支就静默错解析；且回显/日志里的命令串不规范。
- **修复**：改为「参数非空则补一个分隔空格，无参时输出与旧键逐字节相同」（`att.ref` → `"P"`，不含尾空格）。

### 根因四（E41 同类，设计层根治）：未知命令**静默回显**

- 旧链尾只回显原串，打错命令看着像生效了（E41 根因一的老坑）。
- **修复**：引入 `int handled`（乐观置 1，链尾 `else` 置 0），未命中统一回
  `> ?unknown cmd (try: help | att / mag / filt / mot / sys)`；姿态域链尾 `LOG_W` 报未知子命令。

### 一并订正的文档陈旧项（重构同步时发现的**旧文档**错误，非本次引入）

- `uart_list.md` 曾写「指令**不分**大小写前缀（统一取首字符）」→ **错误且危险**：
  `M`=磁力计（姿态）、`m`=`mot`（电机），大小写是**唯一**区分手段。
- `uart_list.md` / `attitude.c` 注释均写命令「由 **StartMotorTask** 路由」→ 实为 **StartLoggerTask**（E41 已迁）。
- `uart_list.md` 「未匹配指令 = 原样回显」→ 已改为报错。
- `uart_list.md` 缺 `P`（参考角，用户高频在用）、`T`、`A/B/S/R` → 已补齐；调试手册 §5.1 改为精简速查 + 指向 `uart_list.md` 为唯一真相源。

### 验证

- 离线用例表 50 条全通过（含负例：`mot.astop` / `magfoo` / `filt-x` / `sys.foo` 必须落 unknown；
  `att.foo` 路由放行后由姿态链尾报 `unknown att cmd`）。
- 括号平衡用脚本按函数体粒度统计（去注释/字符串后比 `{`/`}`、`(`/`)`），`freertos.c` 与 `attitude.c` 均平衡。
- ⚠ **未做**：Keil 编译（0 警告）与烧录冒烟 —— 本环境无 Keil/CubeMX，需用户执行。
- ⚠ **用户必做**：CubeMX Generate Code（`.ioc` `FREERTOS.Queues01` 的 `g_cmd_q` item size 已改 32→64，
  未生成则队列仍 32 B；代码已加 `lbuf[63]='\0'` 兜底防越界，但长命令会撞墙）。

### 关联

- 与 **E41** 同源（命令路由漏配 + 静默丢弃），E41 是「漏列字母」，本事件是「大小写维度不可达」。
  → **共同结论：路由表是命令可达性的唯一开关，新增命令必须同时改「路由表 + 路由表上方注释 + uart_list.md + handler」四处。**
- 沿用 E41 判据：TRACE(5) 下 `LOG_I` 回执被静音，**判断命令是否生效看 VOFA 通道值 / 先 `sys.lvl 4` 再看文本**。
