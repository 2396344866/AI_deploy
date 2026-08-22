#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H

/* =============================================================================
 * BSP_GPIO — 板级 LED GPIO 宏（CubeMX 生成引脚：RED/GREEN/BLUE）
 *   仅做引脚电平写/翻转，无初始化逻辑（GPIO 由 CubeMX 在 MX_GPIO_Init 配置）。
 *   二次开发：新增板级 GPIO 控制请沿用 BSP_<Periph>.{h,c} 命名与 _BSP_<PERIPH>_H 保护宏。
 * ============================================================================= */

#include "main.h"


#define LED_R(n)			(n?HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET))
#define LED_R_TogglePin		HAL_GPIO_TogglePin(RED_GPIO_Port,RED_Pin)	//LED_R电平翻转

#define LED_G(n)			(n?HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin,GPIO_PIN_RESET))
#define LED_G_TogglePin     HAL_GPIO_TogglePin(GREEN_GPIO_Port,GREEN_Pin)	//LED_G电平翻转

#define LED_B(n)			(n?HAL_GPIO_WritePin(BLUE_GPIO_Port,BLUE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(BLUE_GPIO_Port,BLUE_Pin,GPIO_PIN_RESET))
#define LED_B_TogglePin     HAL_GPIO_TogglePin(BLUE_GPIO_Port,BLUE_Pin)	//LED_B电平翻转




#endif
