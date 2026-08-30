# Components/Debug/error.md — 运行期故障导航索引

> **本文件是运行期（runtime）故障的总入口与导航索引。** 详细事件内容已按 FreeRTOS 任务
> 拆分到 `Error/` 子目录（见 `Error/README.md` 的「如何新增任务调试」规范），
> 本文件只保留层级说明、事件映射表、以及跨任务的 AI 误判诚实归档摘要。
> **通道映射/使用说明在 `README.md`；操作步骤在 `姿态外环测试指南.md` / `串级PID测试完整手册.md`；**
> 故障、根因、以及 AI 过程中犯过的判断错误只记这里与 `Error/` 子目录，**不要往测试指南重复记录**。

> **层级说明**：本文件体系是**运行期（runtime）故障**的唯一归属地——固件已编译、烧录后在板子上
> 跑出来的 IMU/I2C/UART/姿态算法/陀螺硬件/启动卡死问题，统一用 **`事件 E N`** 编号（全局连续）。
> **编译期 / 工程配置类故障**（uvprojx 打不开、Build 报错、CubeMX 生成翻倍、L6200E/L6218E 等）
> 单独归 `Doc/Keil_MDK_ARM_工程排错记录.md`（用「问题 N」编号），**绝不混入本文件体系**。
> 启动卡死 / HardFault 的**通用取证方法论**见 `Error/crash_error.md`（具体实例仍记本体系 E 事件）。
> 原 E16（编译期 uvprojx 全空白+177Error）已迁入上述排错记录「问题 4」，本文档体系不再保留副本。

---

## 事件映射表（详细内容在 Error/ 子目录）

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

> 命名约定：每个事件文件内仍以 `# 事件 E N` 为二级标题；新增事件继续全局连续编号，不按任务重置。

> **覆盖状态（8 任务全量）**：已记录运行期故障的任务 = **SENSOR / MOTOR / SCREEN / LOGGER / DIAG**（5 个），
> 对应 `Error/<TASK>_error.md` 已建。其余 **NET（ESP-01S）/ FLASH（W25Q64）/ ESP32（ESP32-S3 CAM）** 目前
> **无已记录的运行期故障**——按约定**不编造故障条目**，故不建对应 `<TASK>_error.md`；结构已预留，
> 后续实测发现异常时直接建 `Error/<TASK>_error.md`、接全局连续 E 编号即可。分类粒度采用**按任务**
> （与 `DBG_LOG_<TASK>` 一一对应、且每任务天然拥有其硬件），非按元器件，避免 MPU6050 等多处重复归档。
> 测试方案已覆盖全部 8 任务（`Test/<TASK>_test.md`）。详见 `Error/README.md` 第四节。

---

## AI 误判诚实归档（跨任务摘要）

> 完整细节见各 `Error/<TASK>_error.md` 对应事件内的「误判记录」小节；此处仅列摘要便于复盘。

- **M1（E1 相关）**：曾把卡死错判为 UART1 TX 非线程安全 → 实为 I2C 总线锁死；`?`/`` 是 I2C 坏字节，非 UART 冲突。
- **M2（E3 相关）**：`verify_dbg_frame.py` 路径算错（多一层 `dirname`）→ 改为单层 `dirname(__file__)`。
- **M3（E4 相关）**：误报 `steer` 分组错位 → 核对后 `dbg_config.h`/`README` 一致，纯误报。
- **R1（E17 相关）**：曾称"进了 main 循环跑、Error_Handler 非致命" → 实测停在死循环，非"仍在运行"。
- **H1–H4（E19 相关）**：时钟改回 480M 错乱 / CubeMX 冲掉 EXTI / TIM1 是遥测触发源 / I2C 写 INT_ENABLE 失败 → 全部证伪，真因是 PC3 上升沿 vs MPU6050 active-low 边沿反。

---

## 复测清单（用户侧，跨任务通用）

1. **静置**（板子不动）一轮：波形稳定不卡 → 验证速率降档 + 基础链路（E1/E19）。
2. **慢转**板子一轮：姿态角平滑跟随，无非法字符（E4/E6/E11）。
3. **剧烈运动**一轮：应不再卡死在 `I2C_IsErrorOccurred`（bus recovery 生效，最坏有限重试后返回 -1 而非死等）（E1/E7）。
4. **调试观测**：推理结果盯 `g_Test_results.data_is_ready` 而非瞬时 `macro_precision`（E18）；区分二进制遥测帧与文本日志（E18）。
