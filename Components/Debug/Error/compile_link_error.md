# 编译 / 链接 故障归档（Build / Link）

> 本文件归 **编译期 / 链接期** 故障（L6200E 重复符号、L6218E 未定义符号等）。
> 原 `Doc/Keil_MDK_ARM_工程排错记录.md` 的 **问题 1 / 问题 5** 迁入此处。
> 运行期板子行为/死机 → 看 `Error/<TASK>_error.md` + `Error/post_error.md`；uvprojx / CubeMX 生成类 → 看 `Error/uvprojx_error.md`。
> 全局参考（非按任务）：`Components/Debug/Ref/`（门控总表 / VOFA 映射）；启动死机取证见 `Error/crash_error.md`。

---

## 问题 1：L6200E 重复符号（Duplicate Symbol）
- **现象**：链接报大量 `L6200E: Duplicate Symbol`，同名符号来自两套 CMSIS-NN。
- **根因**：`Drivers/CMSIS/NN`（旧 API）与 `Components/EdgeImpulse/.../CMSIS/NN`（新 API）同时编入工程，函数名撞车。
- **处理**：切掉 `Drivers/CMSIS/NN` 树 19 个旧文件，保留 EdgeImpulse 自带新 API。两套 int32 ABI 兼容，Fault_Diagnosis 不检查返回值可混过。

## 问题 5：L6218E Undefined symbol ExitRun0Mode
- **现象**：`Error: L6218E: Undefined symbol ExitRun0Mode (referred from startup_stm32h743xx.o)`，`1 Error(s)`。
- **根因**：`system_stm32h7xx.h` 是 CubeMX 固件包 1.12.0+ 新版，头里 `extern void ExitRun0Mode(void)` 声明了该函数；但 `Core/Src/system_stm32h7xx.c`（登记在 `Drivers/CMSIS` 组）是**旧版**——`SystemInit` 未实现 `ExitRun0Mode`。头新源旧 → 链接找不到定义。
- **修复原则（防 CubeMX 覆盖）**：不修改 `system_stm32h7xx.c`（CubeMX 生成区，下次生成会覆盖）。**新建独立文件 `Core/Src/exit_run0_mode.c`** 提供实现，CubeMX 不动 Core/Src 下用户自建文件。
- **已生成文件** `Core/Src/exit_run0_mode.c`：
  ```c
  #include "stm32h7xx.h"
  void ExitRun0Mode(void)
  {
    #if defined (PWR_CPUCR_RUN_D3)
      CLEAR_BIT(PWR->CPUCR, PWR_CPUCR_RUN_D3);
    #endif
  }
  ```
- **加入工程**：Project 窗口展开 `Drivers/CMSIS` 组 → 右键 → Add Existing Files → 选 `Core/Src/exit_run0_mode.c` → Add（或 uvprojx 插 File 条目，须 CRLF）。
- **验证**：L6218E 消失；0 Error；`.\STM32H743VIT6\STM32H743VIT6.axf` 生成。
- **备注**：若 CubeMX 后续 GENERATE CODE 把 `system_stm32h7xx.c` 升级到含 `ExitRun0Mode` 的新版，本文件会变"重复定义"（L6200E），届时从工程移除 `exit_run0_mode.c` 即可（保留磁盘文件无害）。当前状态（2026-08-23 实测）该问题已自愈——所有 `.o` 无人引用 `ExitRun0Mode`，axf 已生成，无需额外操作（除非重新触发）。
