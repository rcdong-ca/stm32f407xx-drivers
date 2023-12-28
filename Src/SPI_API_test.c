/*
 * SPI_API_test.c
 *
 *  Created on: Dec 9, 2023
 *      Author: richard
 *
 *      Small Application to test the Senddata api
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#include <string.h>
#include <stdio.h>

/*
 * SPI2 uses GPIOB.
 * NSS: Pin12
 * SCLK: Pin13
 * MISO: Pin14
 * MOSI: Pin15
 * GPIO pins uses Alternating Functions 5
 */

void SPI2_GpioInits() {

	GPIO_Handle_t Gpio_SpiPins;
	Gpio_SpiPins.GPIOx_ptr = GPIOB;
	Gpio_SpiPins.GPIOx_PinConfig.PinMode = GPIO_MODE_ALTFUN;
	Gpio_SpiPins.GPIOx_PinConfig.PinAltFunMode = 5;
	Gpio_SpiPins.GPIOx_PinConfig.PinOutputSpeed = GPIO_OSPEED_HIGH;
	Gpio_SpiPins.GPIOx_PinConfig.PinOutputType = GPIO_OTYPE_PP;
	Gpio_SpiPins.GPIOx_PinConfig.PinPUpPDo = GPIO_PUPD_NO;

	// Configure the SCLK pin
	Gpio_SpiPins.GPIOx_PinConfig.PinNumber = 13;
	GPIO_Init(&Gpio_SpiPins);

//	// Configure NSS pin
//	Gpio_SpiPins.GPIOx_PinConfig.PinNumber = 12;
//	GPIO_Init(&Gpio_SpiPins);

//	// Configure MISO Pin
//	Gpio_SpiPins.GPIOx_PinConfig.PinNumber = 14;
//	GPIO_Init(&Gpio_SpiPins);

	// Configure MOSI Pin
	Gpio_SpiPins.GPIOx_PinConfig.PinNumber = 15;
	GPIO_Init(&Gpio_SpiPins);
}



int main() {



	char user_data[] = "Hello world";

		//this function is used to initialize the GPIO pins to behave as SPI2 pins
		SPI2_GpioInits();

		//This function is used to initialize the SPI2 peripheral parameters
		SPI_Handle_t SPI2handle;

			SPI2handle.SPIx_ptr = SPI2;
			SPI2handle.SPIx_Config.BusConfig = SPI_BUS_FULL_DUPLEX;
			SPI2handle.SPIx_Config.DeviceMode = SPI_DEVICE_MODE_MASTER;
			SPI2handle.SPIx_Config.BR = SPI_SCLK_BR_DIV2;//generates sclk of 8MHz
			SPI2handle.SPIx_Config.DFF = SPI_DFF_8BITS;
			SPI2handle.SPIx_Config.CPOL = SPI_CPOL_HIGH;
			SPI2handle.SPIx_Config.CPHA = SPI_CPHA_LOW;
			SPI2handle.SPIx_Config.SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

			SPI_Init(&SPI2handle);

		//this makes NSS signal internally high and avoids MODF error
		SPI_SSIConfig(SPI2,ENABLE);

		//enable the SPI2 peripheral
		SPI_PControl(SPI2,ENABLE);

		//to send data
		SPI_Send(&SPI2handle,(uint8_t*)user_data,strlen(user_data));


		while(1);

	return 0;
}

