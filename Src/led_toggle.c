/*
 * led_toggle.c
 *
 *  Created on: Nov 29, 2023
 *      Author: richard
 *
 * Led application to test GPIO driver correctness
 */

#include <stdio.h>
#include <time.h>

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void) {
	for (uint32_t i =0; i < 100000; i++) {}
}

int main(void) {

	GPIO_Handle_t Gpio_Led;
	Gpio_Led.GPIOx_ptr = GPIOD;		// LED is attached to PD12, which is GPIO port D pin 12 (refer to schematic)
	Gpio_Led.GPIOx_PinConfig.PinNumber = 12;
	Gpio_Led.GPIOx_PinConfig.PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIOx_PinConfig.PinOutputSpeed = GPIO_OSPEED_HIGH;
	Gpio_Led.GPIOx_PinConfig.PinOutputType = GPIO_OTYPE_PP;
	Gpio_Led.GPIOx_PinConfig.PinPUpPDo = GPIO_PUPD_NO;		// In Push pull. doesn't need PuPd resistors
//	GPIO_PCLKControl(&Gpio_Led, ENABLE);
	GPIO_Init(&Gpio_Led);


	while(1) {
		GPIO_ToggleOutputPin(&Gpio_Led);
		delay();
	}
}


