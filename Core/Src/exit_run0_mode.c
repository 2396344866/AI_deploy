/**
  ******************************************************************************
  * @file    exit_run0_mode.c
  * @brief   提供 ExitRun0Mode() 实现，修复新版 HAL 头 (system_stm32h7xx.h)
  *          声明但旧版 system_stm32h7xx.c 未实现导致的 L6218E 链接错误。
  *
  * 说明：
  *  - 此文件放在 Core/Src/ 下，不属于 CubeMX 生成区（GENERATE CODE 不会覆盖）。
  *  - 作用：SystemInit() 之后让 Cortex-M7 内核退出 Run0 低功耗电源模式，
  *          清除 PWR->CPUCR 的 RUN_D3 位，使 D3 域保持在正常 Run 模式。
  *  - 本实现对齐本工程 stm32h743xx.h 的 PWR_CPUCR 位定义（仅 RUN_D3 位，
  *    无 RUN_D1/RUN_D2/EXTSMPS，与该 H743 专属头一致）。
  *  - 实现参考 STM32H7 固件包 1.12.0+ 的 system_stm32h7xx.c 标准版本。
  ******************************************************************************
  */

#include "stm32h7xx.h"

/**
  * @brief  Exit from Run0 mode (keep system D3 domain in Run mode).
  * @note   Required by newer CMSIS device headers (HAL pack >= 1.12.0).
  *         Referenced from startup_stm32h743xx / SystemInit path.
  * @retval None
  */
void ExitRun0Mode(void)
{
  /* Prevent the D3 power domain from remaining in Run0 (low-power) state
     after reset. Clear the RUN_D3 bit in PWR->CPUCR so the D3 domain stays
     in normal Run mode. (Matched to this project's stm32h743xx.h bit layout.) */
#if defined (PWR_CPUCR_RUN_D3)
  CLEAR_BIT(PWR->CPUCR, PWR_CPUCR_RUN_D3);
#endif
}
