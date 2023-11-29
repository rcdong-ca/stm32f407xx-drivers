/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 22, 2023
 *      Author: richard
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * 1. Develop Handle structure for a GPIO pin
 * 	- pointer to hold the base address of the GPIO peripheral
 * 	- configuration structure as a pin can have many configurations
 *
 *
 * 	2.  Implement the prototypes listed in the header file which serve as the driver requirements
 */

/*
 * GPIO_Init() initializes the GPIO port according to the configurations found in the GPIO handler
 * which is set by the user.
 * Input:
 * 	GPIOx_Handle: User provided Handler that contains their GPIO configurations
 */
void GPIO_init(GPIO_Handle_t* GPIOx_Handle) {


	GPIO_PinConfig_t* PinConfig_ptr = GPIOx_Handle->GPIOx_PinConfig;
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handle->GPIOx_ptr;
	uint32_t temp = 0;	// registers are of 32bit but values in PinConfig_t may be less

	// 1. configure mode of GPIO pin.
	if (PinConfig_ptr->PinMode <= GPIO_MODE_ANALOG) { // not Interrupt modes
		temp = PinConfig_ptr->PinMode << (PinConfig_ptr->PinNumber * 2);
		// bitfield may already have values, clear it
		GPIOx_ptr->MODER &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
		GPIOx_ptr->MODER |= temp;
	}
	else {
		// TODO:: Do GPIOX interrupt in GPIO_INIT
	}
	// 2. Configure the output speed
	temp = PinConfig_ptr->PinOutputSpeed << (PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->OSPEEDR &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->OSPEEDR |= temp;

	// 3. configure the output type
	temp = PinConfig_ptr->PinOutputType << (PinConfig_ptr->PinNumber);
	GPIOx_ptr->OTYPER &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->OTYPER |= temp;

	// 4. configure the pupd settings
	temp = PinConfig_ptr->PinPUpPDo << (PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->PUPDR &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->PUPDR |= temp;

	// 5. configure the alt functionality, if set to alt mode
	if (PinConfig_ptr->PinMode == GPIO_MODE_ALTFUN) {
		uint32_t alt_idx = PinConfig_ptr->PinNumber / 8;
		temp = PinConfig_ptr->PinAltFunMode << ( (PinConfig_ptr->PinNumber % 8) * 4);
		GPIOx_ptr->AFR[alt_idx] &= ~(0xF << PinConfig_ptr->PinNumber * 2);
		GPIOx_ptr->AFR[alt_idx] |= temp;

	}
}

void GPIO_Reset(GPIO_RegDef_t* GPIOx_ptr) {

}

/*
 * GPIO De-Initialization: Reset the registers for a GPIO port
 */
void GPIO_DeInit(GPIO_RegDef_t* GPIOx_ptr) {

	// reset the GPIOx ports
	switch((uint32_t)GPIOx_ptr) {
		case (uint32_t)GPIOA:
			GPIOA_RESET();
			break;
		case (uint32_t)GPIOB:
				GPIOB_RESET();
				break;
		case (uint32_t)GPIOC:
				GPIOC_RESET();
				break;
		case (uint32_t)GPIOD:
				GPIOD_RESET();
				break;
		case (uint32_t)GPIOE:
				GPIOE_RESET()
				break;
		case (uint32_t)GPIOF:
				GPIOF_RESET();
				break;
		case (uint32_t)GPIOG:
				GPIOG_RESET();
				break;
		case (uint32_t)GPIOH:
				GPIOH_RESET();
				break;
		case (uint32_t)GPIOI:
				GPIOI_RESET();
				break;
	}
}

/*
 * GPIO's peripheral clock controller.
 * Input:
 * 	GPIOx_ptr: pointer address of GPIOx port.
 * 	En_Di: ENABLE or DISABLE clock
 */

void GPIO_PCLKControl(GPIO_RegDef_t* GPIOx_ptr, uint8_t En_Di) {
	GPIOx_ptr->MODER = 1;
	GPIOA_PCLK_EN();
	if (En_Di == ENABLE) {
		switch((uint32_t)GPIOx_ptr) {
			case (uint32_t)GPIOA:
					GPIOA_PCLK_EN();
					break;
			case (uint32_t)GPIOB:
					GPIOB_PCLK_EN();
					break;
			case (uint32_t)GPIOC:
					GPIOC_PCLK_EN();
					break;
			case (uint32_t)GPIOD:
					GPIOD_PCLK_EN();
					break;
			case (uint32_t)GPIOE:
					GPIOE_PCLK_EN();
					break;
			case (uint32_t)GPIOF:
					GPIOF_PCLK_EN();
					break;
			case (uint32_t)GPIOG:
					GPIOG_PCLK_EN();
					break;
			case (uint32_t)GPIOH:
					GPIOH_PCLK_EN();
					break;
			case (uint32_t)GPIOI:
					GPIOI_PCLK_EN();
					break;
		}
	}
	else if (En_Di == DISABLE) {
		switch((uint32_t)GPIOx_ptr) {
			case (uint32_t)GPIOA:
					GPIOA_PCLK_DI();
					break;
			case (uint32_t)GPIOB:
					GPIOB_PCLK_DI();
					break;
			case (uint32_t)GPIOC:
					GPIOC_PCLK_DI();
					break;
			case (uint32_t)GPIOD:
					GPIOD_PCLK_DI();
					break;
			case (uint32_t)GPIOE:
					GPIOE_PCLK_DI();
					break;
			case (uint32_t)GPIOF:
					GPIOF_PCLK_DI();
					break;
			case (uint32_t)GPIOG:
					GPIOG_PCLK_DI();
					break;
			case (uint32_t)GPIOH:
					GPIOH_PCLK_DI();
					break;
			case (uint32_t)GPIOI:
					GPIOI_PCLK_DI();
					break;
		}
	}
	else {
		// INVALID Insert Logging
		// TODO:: add Log for incorrect GPIO address
	}

}
