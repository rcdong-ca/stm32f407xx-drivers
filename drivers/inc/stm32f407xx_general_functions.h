/*
 * stm32f407xx_general_functions.h
 *
 *  Created on: Dec 30, 2023
 *      Author: richard
 */

#ifndef STM32F407XX_GENERAL_FUNCTIONS_H_
#define STM32F407XX_GENERAL_FUNCTIONS_H_

#include "stm32f407xx.h"
/*
 * Get Clock speed of Apb1 bus
 */
uint32_t RCC_GetPCLK1Value(void);
/*
 * Get Clock speed of apb2 bus
 */
uint32_t RCC_GetPCLK2Value(void);

#endif /* STM32F407XX_GENERAL_FUNCTIONS_H_ */
