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

void SPI_GpioInit() {

	GPIO_Handle_t Gpio_SpiPins;
	Gpio_SpiPins.GPIOx_ptr = GPIOB;
	Gpio_SpiPins.GPIOx_PinConfig.PinMode = GPIO_MODE_ALTFUN;
	Gpio_SpiPins.GPIOx_PinConfig.PinAltFunMode = 5;
	Gpio_SpiPins.GPIOx_PinConfig.PinOutputSpeed = GPIO_OSPEED_MED;
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

	char TxBuffer[] = "Hello World!";

	SPI_GpioInit();

	// SPI Initialization
	SPI_Handle_t SPI2Handle;
	SPI2Handle.SPIx_ptr = SPI2;
	SPI2Handle.SPIx_Config.BR = SPI_SCLK_BR_DIV2; // 8MHz
	SPI2Handle.SPIx_Config.BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2Handle.SPIx_Config.CPHA = SPI_CHPA_LOW;
	SPI2Handle.SPIx_Config.CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIx_Config.SSM = SPI_SSM_EN;
	SPI2Handle.SPIx_Config.DFF = SPI_DFF_16BITS;
	SPI2Handle.SPIx_Config.DeviceMode = SPI_DEVICE_MODE_MASTER;

	SPI_Init(&SPI2Handle);
	SPI_SSIConfig(SPI2Handle.SPIx_ptr, ENABLE);

	SPI_PControl(SPI2Handle.SPIx_ptr, ENABLE);

	SPI_Send(&SPI2Handle, (uint8_t*)TxBuffer, strlen(TxBuffer));


	while(1);

	return 0;
}

