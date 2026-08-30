#ifndef _LED_H
#define _LED_H

/* =============================================================================
 * LED — 板载 LED 指示灯驱动（RED / GREEN / BLUE 三色状态灯）
 *   仅做引脚电平写/翻转宏，无初始化逻辑（引脚由 CubeMX 在 MX_GPIO_Init 配置）。
 *   命名约定：板级模块按「实际功能」命名（如 LED / USART / W25Q64），
 *   目录 BSP/<MODULE>/{Inc,Src}，仅当裸名与 CubeMX/Core 生成头撞名时加 BSP_ 前缀。
 * ============================================================================= */

#include "main.h"


#define LED_R(n)			(n?HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET))
#define LED_R_TogglePin		HAL_GPIO_TogglePin(RED_GPIO_Port,RED_Pin)	//LED_R电平翻转

#define LED_G(n)			(n?HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin,GPIO_PIN_RESET))
#define LED_G_TogglePin     HAL_GPIO_TogglePin(GREEN_GPIO_Port,GREEN_Pin)	//LED_G电平翻转

#define LED_B(n)			(n?HAL_GPIO_WritePin(BLUE_GPIO_Port,BLUE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(BLUE_GPIO_Port,BLUE_Pin,GPIO_PIN_RESET))
#define LED_B_TogglePin     HAL_GPIO_TogglePin(BLUE_GPIO_Port,BLUE_Pin)	//LED_B电平翻转




#endif
