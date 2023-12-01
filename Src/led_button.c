/*
 * led_button.c
 *
 *  Created on: Nov 30, 2023
 *      Author: richard
 */


#include <stdio.h>
#include <time.h>

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#define BUTTON_PRESSED 1

void delay(void) {
	for (uint32_t i =0; i < 500000/2; i++) {}
}

int main(void) {

	GPIO_Handle_t Gpio_Led;
	Gpio_Led.GPIOx_ptr = GPIOD;		// LED is attached to PD12, which is GPIO port D pin 12 (refer to schematic)
	Gpio_Led.GPIOx_PinConfig.PinNumber = 12;
	Gpio_Led.GPIOx_PinConfig.PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIOx_PinConfig.PinOutputSpeed = GPIO_OSPEED_MED;
	Gpio_Led.GPIOx_PinConfig.PinOutputType = GPIO_OTYPE_PP;
	Gpio_Led.GPIOx_PinConfig.PinPUpPDo = GPIO_PUPD_NO;		// In Push pull. doesn't need PuPd resistors
	GPIO_PCLKControl(&Gpio_Led, ENABLE);
	GPIO_Init(&Gpio_Led);

	GPIO_Handle_t Gpio_Button;
	Gpio_Button.GPIOx_ptr = GPIOA;
	Gpio_Button.GPIOx_PinConfig.PinNumber = 0;
	Gpio_Button.GPIOx_PinConfig.PinMode = GPIO_MODE_IN;
	Gpio_Button.GPIOx_PinConfig.PinPUpPDo = GPIO_PUPD_NO;	// External Pull Down exists as seen in schematic. No need for internal
	GPIO_PCLKControl(&Gpio_Button, ENABLE);
	GPIO_Init(&Gpio_Button);


	while(1) {
		if (GPIO_ReadFromInputPin(&Gpio_Button) == BUTTON_PRESSED) {
			delay();		// debouncing
			GPIO_ToggleOutputPin(&Gpio_Led);
		}
	}
}



