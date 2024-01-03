/*
 * stm32f407xx_general_functions.c
 *
 *  Created on: Dec 30, 2023
 *      Author: richard
 */

#include "stm32f407xx_general_functions.h"

uint32_t AHB_PreScaler[8] ={2,4,8,16,64,128,256,512};
uint32_t APB_PreScaler[4] = {2,4,8,16};

/*
 * Peripheral Clock speed connected to APB1
 */
uint32_t RCC_GetPCLK1Value(void) {

	uint8_t ClkSource = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3); // SWS is 2 bit long
	uint32_t ClkSourceSpeed = 0;

	if (ClkSource == SYSCLK_HSI) {
		ClkSourceSpeed = SYSCLK_HSI_SPEED;
	}
	else if (ClkSource == SYSCLK_HSE) {
		ClkSourceSpeed = SYSCLK_HSE_SPEED;
	}
	else if (ClkSource == SYSCLK_PLL) {
		ClkSourceSpeed = SYSCLK_PLL_SPEED;
	}

	// Obtain the AHB prescaler
	uint8_t PreScalerChoice = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF); // bit field is 4 bits long
	// Choice <= 8 will have PreScaler of 1
	if (PreScalerChoice >= 0x8) {
		ClkSourceSpeed /= AHB_PreScaler[PreScalerChoice - 0x8];
	}

	// obtain the APB prescaler
	PreScalerChoice = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7); // bit field is 3 bits long
	if (PreScalerChoice >= 0x4) {
		ClkSourceSpeed /= APB_PreScaler[PreScalerChoice - 0x4];
	}
	return ClkSourceSpeed;
}


/*
 * Peripheral Clock speed connected to APB2
 */
uint32_t RCC_GetPCLK2Value(void) {
	uint8_t ClkSource = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3); // SWS is 2 bit long
	uint32_t ClkSourceSpeed = 0;

	if (ClkSource == SYSCLK_HSI) {
		ClkSourceSpeed = SYSCLK_HSI_SPEED;
	}
	else if (ClkSource == SYSCLK_HSE) {
		ClkSourceSpeed = SYSCLK_HSE_SPEED;
	}
	else if (ClkSource == SYSCLK_PLL) {
		ClkSourceSpeed = SYSCLK_PLL_SPEED;
	}

	// Obtain the AHB prescaler
	uint8_t PreScalerChoice = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF); // bit field is 4 bits long
	// Choice <= 8 will have PreScaler of 1
	if (PreScalerChoice >= 0x8) {
		ClkSourceSpeed /= AHB_PreScaler[PreScalerChoice - 0x8];
	}

	// obtain the APB prescaler
	PreScalerChoice = ((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7); // bit field is 3 bits long
	if (PreScalerChoice >= 0x4) {
		ClkSourceSpeed /= APB_PreScaler[PreScalerChoice - 0x4];
	}
	return ClkSourceSpeed;
}
