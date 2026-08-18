# AI_deploy 
## 简介
STM32H743 + W25Q64 工业边缘 AI 故障诊断（铜磨矿）。DIV2IFSCN/DIT2IFSCN：
10 维 → 50 规则 → 100增加节点+50规则 → 4 类。低延迟推断 + 可维护 Flash/日志组件。


## 文档导航
logger.md | BSP_W25Q64.md | BSP_UART.md | Pinout.md |
Deployment_Guide.md | UWB_BU03_移植指南.md 


## 组件
```
Components/BSP   板级驱动（BSP_W25Q64 / BSP_USART / BSP_GPIO）
Components/Logger 统一日志（分级/时间戳/标签/环形缓冲/Flash 黑匣子）
Components/Motor 双电机闭环（TIM1 PWM + TIM3/4 编码 + TIM7 1ms 环）
Components/AI    推理数据与权重
Core/Src/freertos.c   FreeRTOS 任务 + DIV2IFSCN_Inference
Core/Src/stm32h7xx_it.c  HardFault→logger_flush_to_flash；TIM7→Motor_1ms_Handler
MDK-ARM/          Keil 工程（BSP/Logger/Motor/AI 分组）
```

## 里程碑
- W25Q64 JEDEC 0xEF4017 读写通过
- 日志 [ms][LEVEL][TAG](file:line) 正常
- SPI 40MHz（SPI123CLK=160MHz，D-Cache 已处理）
- 双电机闭环：TIM1 10kHz PWM / TIM3-4 编码器 / TIM7 1ms 环 / 增量 PI + 位置级联

## 模型与推理（代码事实）
- 结构：input10 → H0(50) → Z_final(150) → 4 类；FC1 5000MAC / FC2 600MAC
- 权重 7700 floats ≈ 30.1KB @ .dtcmram（零等待、CPU 专用）
- 加速：gamma LUT 替 powf / 预计算 inv_sigma / CMSIS-DSP f32 算子
- 精度（PC 预校验, 1605 样本, 对齐 PyTorch 94.70%）：
  `f32 94.70% | HYBRID 90.97% | FC1f32+FC2i8 91.59% | FC1i8+FC2i8 90.03% | FULL_INT8 88.72%`
  （硬件实测见 Deployment_Guide；StartInferenceTask 打印实际 acc）
- 当前 DIT2 模型：γ=0.7, p=±0.3, H0 absmax±1.83（无范围失配，FC2 int8 可行）
- 未实现：per-channel 量化 / Flash 45% 压缩

## 注意
- 自定义代码放 Components/，勿放 Core/（防 CubeMX 重生覆盖）
- 重生后检查 Keil Components 组是否被清
- 勿改时钟树（SPI 锁 160MHz）
