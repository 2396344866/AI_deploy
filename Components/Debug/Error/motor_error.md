# MOTOR 任务运行期故障归档（电机 / 控制环 / 命令分发）

> 本文件是 **MOTOR 任务**（StartMotorTask + 电机驱动/外环控制/串口命令分发）的运行期故障归属地，
> 事件用全局连续编号 **E N**（与 `Components/Debug/error.md` 一致）。
> 联调笔记见 `Components/Debug/debug/motor_debug.md`；测试方案见 `Components/Debug/Test/motor_test.md`。
> 编译期故障归 `Doc/Keil_MDK_ARM_工程排错记录.md`（问题 N）；启动死机取证归 `Components/Debug/Error/crash_error.md`。

---

## 事件 E8：外环符号反了——前倾时轮子向后转，正反馈越倒越快（2026-08-19）

### 现象（用户实测）
- 外环开（`T1` + `R` + `A0`），手持板子测试，VOFA 通道 22（tgtA）：
  - 静置 pitch≈-3.6° → tgtA ≈ +12~15（本应接近 0）。
  - 前倾 → tgtA ≈ **-50**（应为正，轮子应前进扶直，实测却向后）。
  - 后仰 → tgtA ≈ **+100**（应为负，实测却向前）。
- 即：倾倒方向与轮子驱动方向**相反**，属正反馈，机器人会越倒越快。

### 根因
- `Attitude_RunController` 的误差符号写反：`err = g_pitch_ref - pitch`。
- 物理正确链路（倒立摆"轮子追重心"）：前倾(pitch↑) → err 应为正 → u 正 → 轮子前进扶直。
  - 类比：手掌立扫帚，扫帚往前倒手必须往前追。
  - 力矩视角：电机驱动轮子前进 → 反作用力矩使车身后仰 = 扶直方向。
- 旧代码 `ref - pitch` 在前倾时得到负 err → 轮子后退 → 反作用力矩把车身进一步前按 → 正反馈加速倾倒。

### 修复方案（代码）
- `attitude.c` `Attitude_RunController`：`err = g_pitch_ref - pitch` → **`err = pitch - g_pitch_ref`**。
  - 微分项 `- g_kd * rate` 方向本来正确，**未改动**。
- 连带（否则 VOFA 通道 16 显示反）：`Attitude_GetPitchErr()` 同步改为 `s_att.pitch - g_pitch_ref`。
- 连带（命令解析 bug）：`P` 命令解析从 `atoi` 改 `atof`——原 `atoi("P-3.6")` 会截断成 `-3`，改后才能精确设 `-3.6`。
- 文档同步：`attitude.h` 注释、`姿态外环测试指南.md` 公式、`README.md` 通道 16 说明均改为 `pitch - ref`。

### 验证步骤（用户侧）
1. Keil 重编 0 错误、烧录。
2. 发 `P-3.6`（补偿安装偏角）+ `K5,0,2`（kp=5,ki=0,kd=2 加阻尼）。
3. `T1` + `R` 手持测试方向：前倾 → tgtA **正**（前进扶直）；后仰 → tgtA **负**（后退扶直）。
4. 方向对了再落地试直立（先用手护住，随时 `T0`/`S` 急停）。
5. 若方向仍反 → 检查 `IMU_PITCH_SIGN` 或 `cmd_dir`，但本轮代码已按正确物理修正。

### 状态
- [x] 根因定位（外环误差符号反，正反馈）
- [x] `attitude.c`：`err` 符号 + `Attitude_GetPitchErr` + `P` 命令 atof（本次实现）
- [x] 文档同步（attitude.h / 测试指南 / README）
- [x] IMU 轴方向验证通过（用户实测）：前倾 pitch -2.5→+14（正）、后仰 -3.2→-20（负）；roll 前倾仅动 2°、后倾 5.6°，相对 pitch 16~17° 属动作耦合，**非串轴**。结论：MPU6050 竖插安装下 pitch 符号/轴仍正确，"方向反了"的怀疑不成立。
- [ ] 用户落地验证：手持前倾松手，看车身是被扶直（方向对，只调增益）还是加速前倒（才查电机转向/`cmd_dir`）
- [ ] 落地前确认：前倾时左右两轮是否同向（都往前）；若左前右后打转 = `cmd_dir` 与实际安装不符

---

## 事件 E9：K 命令 kd 解析 bug——kd 恒为 0（2026-08-20）

### 现象（用户实测）
- 发 `K0.1,0,5`，期望 kd=5，但 VOFA 通道 19（kd）始终为 0；kp 正常变为 0.1。
- 追溯发现：此前所有 `K` 命令（如 `K5,0,2`、`K8,0,2`）的 kd 实际都未被设置，恒为 0。

### 根因
- `Attitude_ProcessCommand` 的 K 分支：找到第 1 个逗号后解析 ki，接着 `while (*p && *p != ',') p++;` 找第 2 个逗号。
  但此时 p **已经停在第一个逗号上**，`*p != ','` 立即为假，循环体不执行，p 没有前进。
  于是 `kd = atof(p + 1)` 仍从 `"0,5"` 解析 → atof 在逗号处截断 = 0。
- 修复：解析完 ki 后 `p++` 跳过第 1 个逗号，第二个 while 才能从 `"5"` 开始找第 2 个逗号。

### 修复方案（代码，用户已在 Keil 改）
- `attitude.c` `Attitude_ProcessCommand` K 分支：`if (*p == ',') { ki = (float)atof(p + 1); p++; }`（增加 `p++;`）。
- **连带影响**：此前所有 K 命令的 kd 实际为 0 → 落地整定阶段一直**没有任何微分阻尼**，是早期落地易抖 / 站不稳的隐藏原因之一；修复后 kd 才真正生效。

### 状态
- [x] 根因定位（第二个逗号未前进 → kd 恒 0）
- [x] `attitude.c` K 分支加 `p++`（本次实现 / 用户已改）
- [ ] 用户重编烧录，发 `K0.1,0,5` 确认 VOFA I19 变为 5、串口打印 `gains kp=0.10 ki=0.00 kd=5.00`

---

## 事件 E13：串口命令 `C/F/D` 未路由到 `Attitude_ProcessCommand`（2026-08-20）

### 现象
- `Attitude_ProcessCommand` 里已经实现了 `C`（重标定）、`F`（滤波配置）、`D`（诊断）命令，
  但 `freertos.c` `StartMotorTask` 的命令分发只把 `T/P/K` 交给姿态模块，其余都进了 `Motor_ProcessCommand`。
- 结果：用户发 `C` / `F` / `D` 实际**被电机层丢弃**，表现为"命令无回显/无作用"，
  导致之前的 `F print`、`C` 重标定、`D` 诊断全都没有真正执行。

### 修复方案（代码，本次实现）
- **`Core/Src/freertos.c`**：命令分发条件改为
  `T/P/K/C/F/D` 全部进入 `Attitude_ProcessCommand`，其余才走电机层。

### 验证步骤
1. 重编烧录。
2. 发 `F print` → 应回显 `raw_lag=1 ...`（此前无回显）。
3. 发 `D` → 应回显 MPU 寄存器/采样（此前无回显）。
4. 发 `C` → 板子必须静置，回显 gyro offset 值或 `SKIPPED`（此前无回显）。

### 状态
- [x] `freertos.c` 命令路由修复（本次实现）
- [ ] 用户重编后验证 `F print` / `D` / `C` 有回显
