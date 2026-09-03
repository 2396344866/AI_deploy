# 平衡控制本地自治 — 实施计划（v4b）

> 配套状态机图：`balance_autonomy_fsm_v4b.html`（白底菱形、云端+本地+断网同机、所有节点在开始→结束内）。
> 本文是**框架合约 + 验收标准 + 阈值表**，图与文分离，互不破坏排版。

## 1. 设计原则（真相源）

- **本地自治为唯一真相源**；云端 / KEY 仅发**意图**（站起 / 坐下），绝不直接遥控电机 PWM。
- 任何使能动作必须依次过：传感器健康 → 倾角安全 → 网络状态（离线则锁存云端意图、仅 KEY 驱动，不耦合平衡）→ 站起/坐下条件校验。
- **角度上报永远开**，与平衡开关解耦（纠正旧名 `Euler_angle_open` 的误导语义）。

## 2. 状态机 v4b 文字版

| 节点 | 类型 | 条件/动作 |
|---|---|---|
| 上电·系统启动 | ○ | — |
| 安全锁 | ▭ | 平衡 OFF · 刹车 · 角度照报 |
| 读取输入 | ▭ | 云端指令 · 本地 KEY · 网络 · IMU |
| D1 传感器健康? | ◇ | 否→强制急停·关平衡(传感器失效) |
| D2 倾角>倾覆阈值? | ◇ | 是→强制急停·关平衡(姿态倾覆) |
| D3 网络在线? | ◇ | 否→断网保护：锁存云端意图·离线仅 KEY 驱动 |
| D4 站起请求?(云/KEY) | ◇ | 无→维持现状 |
| D5 直立且电机就绪? | ◇ | 否→拒绝站起·保持安全锁 |
| ENGAGE | ▭(绿) | 本地使能平衡 |
| D6 坐下请求?(云/KEY) | ◇ | 无→维持现状 |
| D7 接近安全姿态? | ◇ | 否→拒绝坐下·保持平衡 |
| 优雅放下 | ▭(绿) | 平稳关平衡 |
| 周期结束·下一循环 | ○ | — |

## 3. 代码改动清单（5 处，FSM 永久接管，无临时门控）

| # | 文件 | 改动 | 运行期行为 |
|---|---|---|---|
| 1 | `Components/BSP/IMU/Inc/attitude.h` + `Src/attitude.c` | 新增 `IsHealthy / SetKeyStand / SetCloudStand / SetCloudSit / SetNetOnline / AutonomyTick` + 阈值宏 + 静态自治状态 | `AutonomyTick` 每周期仲裁；接口常驻 |
| 2 | `Core/Src/freertos.c` `StartSensorTask` | 循环内调 `Attitude_AutonomyTick()` | 每周期执行 FSM 决策 |
| 3 | `Core/Src/freertos.c` `StartMotorTask` KEY | KEY 下降沿发站起/坐下意图（`Attitude_SetKeyStand`），由 FSM 仲裁 | 不再直切电机运行/刹车 |
| 4 | `Components/BSP/ESP/Src/esp01s.c` | 属性 `Euler_angle_open`→`balance_enable`；handler 置云意图(stand/sit)；`prop_motion_set` 加"未平衡拒绝运动"闸门 | 云端发意图、未平衡拒运动 |
| 5 | `Doc/阿里云物模型属性映射.md` | 标识符/语义更正为"意图"，角度永远上报 | — |

> 网络在线标志经 `Attitude_SetNetOnline`（P3 已从网络状态机接线）喂入 D3；D3 仅门控"意图来源"——离线则锁存云端意图、仅 KEY 驱动，不耦合平衡；姿态外环(平衡)由运动控制律独立处理（尚未实现）。

## 4. 决策权威（无旧逻辑兜底）

平衡使能的**唯一**决策权威是本地 FSM（`Attitude_AutonomyTick` 每周期仲裁）。**旧逻辑已废弃**——不再保留「KEY 直切电机运行/刹车」「云端直控 `Attitude_SetEnable`」作为可运行分支：

- **KEY**：下降沿 → `Attitude_SetKeyStand()` 发「站起/坐下意图」，由 FSM 仲裁。
- **云端**：`balance_enable` 属性 → `SetCloudStand/SetCloudSit` 发意图，由 FSM 仲裁。
- **运动指令**：`prop_motion_set` 需先经「已平衡」闸门，否则拒绝。
- **网络**：经 `Attitude_SetNetOnline`（P3 已从 `StartNetworkTask` 边沿接线）喂入 D3，仅门控云端意图来源。

> **临时门控已移除**：`ATTITUDE_AUTONOMY_ENABLED` 宏整体删除，FSM **永久接管、无兜底、无旧逻辑路径**。原 `=0` 分支从未编译/测试，保留无安全收益；运行时行为即为 FSM 仲裁结果。

## 5. 阈值表（默认保守，Phase-2 按 vofa 数据标定）

| 宏 | 默认 | 含义 |
|---|---|---|
| `ATT_TILT_OVERTURN_DEG` | 50.0 | 俯仰/横滚绝对值超此即倾覆，强制急停 |
| `ATT_TILT_STAND_READY_DEG` | 10.0 | 在此范围内才允许 ENGAGE 站起 |
| `ATT_TILT_SIT_SAFE_DEG` | 8.0 | 接近此姿态才允许优雅放下 |

`IsHealthy` 代理：`|a_mag − 1g| < 0.3` 且三轴陀螺原始计数未饱和（±32000）。

## 6. 验收标准（上线 FSM 自治的前置条件）

- [ ] KEY 手动站起后车能稳态直立 ≥ N 秒，无发散/翻转（PID 已整定）。
- [ ] vofa 曲线：pitch/roll 收敛、控制器输出无饱和震荡、原始/后滤波对比正常。
- [ ] 断网（拔 ESP-01S 或关 AP）：本地持续平衡、拒运动指令，不摔倒、不卡死。
- [ ] 误关（云端 `balance_enable=0` 中途）：进入优雅放下或保持直立，不瞬间失能摔倒。
- [ ] 倾角 > 倾覆阈值：强制急停关平衡。
- [ ] 倾角 > 站起就绪阈值：拒绝 ENGAGE，保持安全锁。
- [x] P3 接线 `Attitude_SetNetOnline`（CONNACK OK→1，掉线/超时→0，边沿幂等），D3 离线分支生效（仅门控云端意图，不耦合平衡）。

## 7. 执行顺序（框架 → 细节）

- **Phase 0**（本文）：框架合约 + 验收门槛。
- **Phase 1（现在）**：落地 #1–#5 骨架，FSM 永久接管（宏已移除），Keil 编译 0 警告。
- **Phase 2（平衡外环实现后）**：在 vofa 确认平衡稳定、数据干净后，标定阈值、esp01s 属性 `balance_enable` 联调、逐条联调 §6 验收项。注：D3 网络接线（P3）已完成且**仅门控云端意图来源**；姿态外环(平衡)尚未实现，属独立工作，不在此阶段范围内。

## 8. 风险/边界

- 当前 `Attitude_ProcessCommand` 的串口 `T1/T0` 仍直控 `SetEnable`（本地调试覆盖），不与 `g_autonomous` 联动——属预期，调试用。
- `Motor_Resume/EmergencyStop` 由 FSM 在 ENGAGE/放下时调用；电机内环运行态与平衡外环使能需同时成立，FSM 已协调。
- 改代码前**先取证** `Components/Debug/Error/`；本改动为新增框架，无同类历史事件，落盘于此文档 + 代码注释。
