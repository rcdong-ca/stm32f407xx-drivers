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
	for (uint32_t i =0; i < 1000000; i++) {}
}

int main(void) {

	GPIO_Handle_t Gpio_Led;
//	Gpio_Led.GPIOx_ptr = GPIOB;
//	uint32_t s = AHB1_BASE_ADDR + 0x0400;
//	GPIO_RegDef_t* my_ptr = (GPIO_RegDef_t*)s;
//	if (my_ptr == GPIOB)
//		printf("lol");
//	Gpio_Led.GPIOx_ptr = GPIOD;
//
	Gpio_Led.GPIOx_ptr = GPIOD;		// LED is attached to PD12, which is GPIO port D pin 12 (refer to schematic)
	Gpio_Led.GPIOx_PinConfig.PinNumber = 12;
	Gpio_Led.GPIOx_PinConfig.PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIOx_PinConfig.PinOutputSpeed = GPIO_OSPEED_MED;
	Gpio_Led.GPIOx_PinConfig.PinOutputType = GPIO_OTYPE_PP;
	Gpio_Led.GPIOx_PinConfig.PinPUpPDo = GPIO_PUPD_NO;		// In Push pull. doesn't need PuPd resistors

	GPIO_PCLKControl(&Gpio_Led, ENABLE);
	GPIO_Init(&Gpio_Led);
//	GPIO_WriteToOutputPin(&Gpio_Led, (uint8_t)1);
//
//	int s = 0;
//	printf("hellow wolrd\n");

	while(1) {
		GPIO_ToggleOutputPin(&Gpio_Led);
		delay();
	}
}


