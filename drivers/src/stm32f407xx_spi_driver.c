/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Dec 6, 2023
 *      Author: richard
 */
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

/*
 * Get Status register info
 * Input:
 * 	SPIx_ptr: address of SPIx peripheral
 * 	StatusField: Status field bit. Refer to @SPI_STATUS_FIELD
 * Output:
 * 	bit: check bit at StatusField is SET or NOT_SET
 */
uint8_t SPI_GetStatus(SPI_RegDef_t* SPIx_ptr, uint8_t StatusField) {
	return (SPIx_ptr->DR >> StatusField) & 1;
}

/*
 * Initiates SPI_Peripheral
 */
void SPI_Init(SPI_Handle_t* SPIx_Handler) {

	// Initiate Peripheral Clock
	SPI_PCLKControl(SPIx_Handler, ENABLE);

	// clear SPI_CR1 Register
	SPIx_Handler->SPIx_ptr->CR1 = 0;

	// Set up Device mode
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.DeviceMode << 2);

	//Set up Bus Configuration.
	if (SPIx_Handler->SPIx_Config.BusConfig == SPI_BUS_FULL_DUPLEX) {
		SPIx_Handler->SPIx_ptr->CR1 |= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (SPIx_Handler->SPIx_Config.BusConfig == SPI_BUS_HALF_DUPLEX) {
		SPIx_Handler->SPIx_ptr->CR1 |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (SPIx_Handler->SPIx_Config.BusConfig == SPI_BUS_SIMPLEX_RXONLY) {
		SPIx_Handler->SPIx_ptr->CR1 |= ~(1 << SPI_CR1_BIDIMODE);
		SPIx_Handler->SPIx_ptr->CR1 |= (1 << SPI_CR1_RXONLY);
	}
	// Set up Data frame format
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.DFF << SPI_CR1_DFF);

	// Set Clock Phase
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.CPHA << SPI_CR1_CPHA);
	// Set Clock Polarity
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.CPOL << SPI_CR1_CPOL);
	// Set SSM
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.SSM << SPI_CR1_SSM);
	// Set Baud Rate
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.BR << SPI_CR1_BR);
}

void SPI_DeInit(SPI_Handle_t* SPIx_Handler) {
	// Disable Peripheral Clock
	SPI_PCLKControl(SPIx_Handler, DISABLE);

	if (SPIx_Handler->SPIx_ptr == SPI1) {
		SPI1_RESET();
	}
	else if (SPIx_Handler->SPIx_ptr == SPI2) {
		SPI2_RESET();
	}
	else if (SPIx_Handler->SPIx_ptr == SPI3) {
			SPI3_RESET();
		}
}

/*
 * Send and Receive Data
 */
void SPI_Send(SPI_Handle_t* SPIx_Handler, uint8_t* TxBuffer_ptr, uint32_t DataSize) {
	uint32_t CurrStep = 0;
	while (CurrStep < DataSize) {
		// wait for tx buffer to be empty
		while ( SPI_GetStatus(SPIx_Handler->SPIx_ptr, SPI_SR_TXE) == NOT_SET);

		// Check Data Format
		if (SPIx_Handler->SPIx_Config.DFF == SPI_DFF_8BITS) {
			// Write in to Data Register
			SPIx_Handler->SPIx_ptr->DR = TxBuffer_ptr[CurrStep];
			CurrStep++;

		}
		else if (SPIx_Handler->SPIx_Config.DFF == SPI_DFF_16BITS) {
			if (CurrStep + 1 < DataSize) {
				// Combine two uint8_t into uint16_t
				SPIx_Handler->SPIx_ptr->DR = ((uint16_t)TxBuffer_ptr[CurrStep] << 8) | TxBuffer_ptr[CurrStep + 1] ;
				CurrStep += 2;
			}
			else {
				// Only 1 uint8_t left
				SPIx_Handler->SPIx_ptr->DR = TxBuffer_ptr[CurrStep];
				CurrStep++;
			}
		}

	}
}
void SPI_Receive(SPI_Handle_t* SPIx_Handler, uint8_t* RxBuffer_ptr, uint32_t DataSize);

/*
 * IRQ (Interrupt request) configuration and ISR (Interrupt Signal Response) handling
 */

void SPI_IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI);	// Interrupt requests configuration
void SPI_IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t IRQPriority);
//void SPI_IRQHandling(uint8_t PinNumber);


/*
 * SPI Peripheral Clock controller
 */

void SPI_PCLKControl(SPI_Handle_t* SPIx_Handler, uint8_t En_Di) {
	if (SPIx_Handler->SPIx_ptr == SPI1) {
		if (En_Di == ENABLE)
			SPI1_PCLK_EN();
		else if (En_Di == DISABLE)
			SPI1_PCLK_DI();
	}
	else if (SPIx_Handler->SPIx_ptr == SPI2) {
		if (En_Di == ENABLE)
			SPI2_PCLK_EN();
		else if (En_Di == DISABLE)
			SPI2_PCLK_DI();
	}
	else if (SPIx_Handler->SPIx_ptr == SPI3) {
		if (En_Di == ENABLE)
			SPI3_PCLK_EN();
		else if (En_Di == DISABLE)
			SPI3_PCLK_DI();
	}
}

/*
 * SPI periphera Control
 */
void SPI_PControl(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di) {
	if (En_Di == ENABLE) {
		SPIx_ptr->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		SPIx_ptr->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

