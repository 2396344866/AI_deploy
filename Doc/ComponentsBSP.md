# Components/BSP 目录约定（二次开发规范）

> 目的：统一 BSP 层文件风格，让后续维护者一眼知道「新文件该叫什么、放哪、怎么写头」。

## 1. 目录布局

```
BSP/
├── Inc/            # 板级外设驱动头（BSP_ 前缀）
│   ├── BSP_GPIO.h
│   ├── BSP_USART.h
│   └── BSP_W25Q64.h
├── Src/            # 板级外设驱动实现（与 Inc 一一对应）
│   ├── BSP_GPIO.c
│   ├── BSP_USART.c
│   └── BSP_W25Q64.c
├── IMU/            # IMU 子系统（算法层，无 BSP_ 前缀）
│   ├── Inc/
│   │   ├── attitude.h        # 姿态解算 + 外环 PID 控制器
│   │   ├── imu_mpu6050.h     # MPU6050 I2C 底层
│   │   ├── imu_filter.h      # 输入滤波（去极值均值 / 一阶滞后）
│   │   ├── madgwick.h        # Madgwick AHRS 融合（C 移植）
│   │   └── vofa_telemetry.h  # VOFA+ firewater 遥测
│   └── Src/  (对应 .c)
└── README.md       # 本文件
```

两种目录存在是**有意区分**，不是随意：
- `BSP/Inc|Src` 放**板级外设**（GPIO/USART/Flash），命名 `BSP_<Periph>`。
- `BSP/IMU` 放**算法子系统**（驱动+融合+控制），命名 `<module>`（无 `BSP_` 前缀）。

新增文件时：板级外设进 `BSP/Inc|Src` 用 `BSP_` 前缀；IMU/算法相关进 `BSP/IMU`，沿用现有无前缀命名。

## 2. 命名约定

| 类别 | 命名 | 示例 |
|------|------|------|
| 板级外设头/源 | `BSP_<Periph>.h/.c` | `BSP_W25Q64.h` |
| IMU/算法模块 | `<module>.h/.c` | `attitude.h`、`madgwick.c` |
| 头文件保护宏 | `_BSP_<PERIPH>_H` / `_<MODULE>_H` | `_BSP_W25Q64_H`、`_ATTITUDE_H` |
| 对外函数 | `BSP_*` / `Attitude_*` / `Madgwick_*` / `ImuFilter_*` / `MPU6050_*` / `Vofa_*` | 见各头 |

## 3. 头文件写法规范（所有 BSP 头必须遵守）

1. **保护宏**：`#ifndef _XXX_H` / `#define _XXX_H` / `#endif /* _XXX_H */`，大写、单个尾部下划线。
   - ⚠️ 历史坑：`BSP_W25Q64.h` 曾误写为 `_BSP_W24Q64_H`（24/25 错），已修正。
2. **段落文档注释**：每个头顶部用 `/* ===... */` 写清「职责 + 依赖 + 二次开发提示」，与 IMU 模块风格一致。
3. **依赖最小化**：尽量只 `#include <stdint.h>` 等标准头；确实需要 HAL 句柄时再 `#include "main.h"`（GPIO/USART 现状），IMU 模块已改为不依赖 `main.h` 的写法，新代码建议效仿。
4. **extern 句柄**：外设句柄（如 `hi2c1`）在 CubeMX 生成文件中定义，本层用 `extern` 引用，不要把定义写进 BSP（重生成零影响）。

## 4. 已纳入 Keil include path

`MDK-ARM/STM32H743VIT6.uvprojx` 已包含：
- `../Components/BSP/Inc`
- `../Components/BSP/IMU/Inc`

新增 BSP 头若放在上述两目录内，无需改工程配置即可被 `#include "xxx.h"` 找到。
若在别处新建子目录，记得同步在 uvprojx 的 C/C++ → Include Paths 追加对应相对路径。

## 5. 接口速查（IMU 外环）

- `Attitude_Init()` 初始化 MPU6050 + Madgwick + 滤波，并打印静态标定日志（重力轴应≈±16384 LSB）。
- `StartSensorTask` 由 MPU6050 data-ready 中断（EXTI3）信号量驱动，200 Hz 跑：
  `MPU6050_ReadRaw → ImuFilter_Update → Attitude_Update → Attitude_RunController → Dbg_Telemetry_Send`。
- 串口命令（在 `StartMotorTask` 解析）：`T1/T0` 开/关外环；`P<deg>` 目标俯仰角；`K<kp>,<ki>,<kd>` 在线增益。
- 遥测：**调试波形已迁至 `Components/Debug/`（UART1 firewater 30 通道固定帧，宏开关分组）**；
  UART4 留给淘晶驰串口屏。完整通道映射/宏说明/VOFA 导入见 `Components/Debug/README.md`。
- 运行期错误与 AI 误判（含 I2C 总线锁死卡死事件 E1）统一记在 **`Components/Debug/error.md`**，排查故障时先查该文件。
