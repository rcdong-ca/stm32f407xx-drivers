/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Dec 6, 2023
 *      Author: richard
 */

#include <stdio.h>
#include <stddef.h>

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
/*
 * Spi Interrupt handle functions @SPI_INTERUPPT_FX
 */
static void Spi_Txe_Interrupt_Handle();
static void Spi_Rxne_Interrupt_Handle();
static void Spi_Ovr_Interrupt_Handle();



/*
 * Get Status register info
 * Input:
 * 	SPIx_ptr: address of SPIx peripheral
 * 	StatusField: Status field bit. Refer to @SPI_STATUS_FIELD
 * Output:
 * 	bit: check bit at StatusField is SET or NOT_SET
 */
uint8_t SPI_GetStatus(SPI_RegDef_t* SPIx_ptr, uint8_t StatusField) {
	uint8_t status = (SPIx_ptr->SR >> StatusField) & 1;
	return status;
}

/*
 * Initiates SPI_Peripheral
 */
void SPI_Init(SPI_Handle_t* SPIx_Handler) {

	// Initiate Peripheral Clock
	SPI_PCLKControl(SPIx_Handler, ENABLE);

//	 clear SPI_CR1 Register
	SPIx_Handler->SPIx_ptr->CR1 = 0;

	// Set up Device mode
	SPIx_Handler->SPIx_ptr->CR1 |= (SPIx_Handler->SPIx_Config.DeviceMode << SPI_CR1_MSTR);

	//Set up Bus Configuration.
	if (SPIx_Handler->SPIx_Config.BusConfig == SPI_BUS_FULL_DUPLEX) {
		SPIx_Handler->SPIx_ptr->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
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
		while ( SPI_GetStatus(SPIx_Handler->SPIx_ptr, SPI_SR_TXE) == NOT_SET); // BUg this line

		// 8 bit data format
		if ( ((SPIx_Handler->SPIx_ptr->CR1 >> SPI_CR1_DFF) & 0x1 ) == SPI_DFF_8BITS) {
			// Write in to Data Register
			SPIx_Handler->SPIx_ptr->DR = *TxBuffer_ptr;
			TxBuffer_ptr++;
			CurrStep++;

		}
		else {
			// Combine two uint8_t into uint16_t
			SPIx_Handler->SPIx_ptr->DR = *(uint16_t*)TxBuffer_ptr;
			(uint16_t*)TxBuffer_ptr++;
			CurrStep += 2;
		}

	}
}
void SPI_Receive(SPI_Handle_t* SPIx_Handler, uint8_t* RxBuffer_ptr, uint32_t DataSize) {
	uint32_t CurrStep = 0;
		while (CurrStep < DataSize) {
			// wait for tx buffer to be empty
			while ( SPI_GetStatus(SPIx_Handler->SPIx_ptr, SPI_SR_RXNE) == NOT_SET);

			// 8 bit data format
			if ( ((SPIx_Handler->SPIx_ptr->CR1 >> SPI_CR1_DFF) & 0x1 ) == SPI_DFF_8BITS) {
				// Write in to Data Register
				RxBuffer_ptr[CurrStep] = (uint8_t)SPIx_Handler->SPIx_ptr->DR;
				CurrStep++;

			}
			else {
				if (CurrStep + 1 < DataSize) {
					// Combine two uint8_t into uint16_t
					*((uint16_t*)(RxBuffer_ptr + CurrStep)) = SPIx_Handler->SPIx_ptr->DR;
					CurrStep += 2;
				}
			}

		}
}

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
 * SPI peripheral Control
 */
void SPI_PControl(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di) {
	if (En_Di == ENABLE) {
		SPIx_ptr->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		SPIx_ptr->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/*
 * Configure the SSI bit. This influences NSS when SSM is enabled.
 * If NSS is pulled to GND. This means another master is using the SPI bus
 * If NSS is pulled up, it is using the SPI BUS.
 */
void SPI_SSIConfig(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di)  {
	if (En_Di == ENABLE) {
		SPIx_ptr->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		SPIx_ptr->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di)  {
	if (En_Di == ENABLE) {
		SPIx_ptr->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else {
		SPIx_ptr->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * Configure the NVIC registers to accepts interrupt from EXTIx Line
 */

void SPI_IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI) {
	if (EN_DI == ENABLE) {

		// MCU only supports up to 81 IRQ numbers, so we only need NVIC_ISER0 - NVIC_ISER2 registers
		if (IRQNumber < 32) {
			NVIC_Ctrl->ISER->REG[0] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 64) {
			NVIC_Ctrl->ISER->REG[1] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 96) {
			NVIC_Ctrl->ISER->REG[2] |= (1 << (IRQNumber % 32) );
		}
		else {
			// LOG:: INVALID IRQ NUMBER
		}
	}
	else {
		if (IRQNumber < 32) {
			NVIC_Ctrl->ICER->REG[0] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 64) {
			NVIC_Ctrl->ICER->REG[1] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 96) {
			NVIC_Ctrl->ICER->REG[2] |= (1 << (IRQNumber % 32) );
		}
	}
}

/*
 * SPI Interrupt Functions
 */

uint8_t SPI_SendDataInterrupt(SPI_Handle_t* SpiHandler_ptr, uint8_t* TxBuffer_ptr, uint32_t TxLen) {
	uint8_t state = SpiHandler_ptr->TxState;
	// SPI is Not busy in Tx operation
	if (state != SPI_STATE_BUSY_TX) {
		// Save buffer address and len in Handler which will be read by the interrupt handler
		SpiHandler_ptr->TxBufferLen = TxLen;
		SpiHandler_ptr->TxBuffer_ptr = TxBuffer_ptr;
		// Set SPI state to Tx busy so it will be locked until transmissino is over
		SpiHandler_ptr->TxState = SPI_STATE_BUSY_TX;
		// Enable the TXEIE control bit. IRQ generated when TXE bit in SR is set
		SpiHandler_ptr->SPIx_ptr->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataInterrupt(SPI_Handle_t* SpiHandler_ptr, uint8_t* RxBuffer_ptr, uint32_t DataSize) {
	uint8_t state = SpiHandler_ptr->RxState;
	// check if SPI is busy from Rx operation
	if (state != SPI_STATE_BUSY_RX) {
		SpiHandler_ptr->RxBuffer_ptr = RxBuffer_ptr;
		SpiHandler_ptr->RxBufferLen = DataSize;
		SpiHandler_ptr->RxState = SPI_STATE_BUSY_RX;
		SpiHandler_ptr->SPIx_ptr->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}


void SPI_IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t IRQPriority) {

	// IPR has 4 8bit priority fields
	uint8_t TargetReg = IRQNumber / 4;
	uint8_t TargetField = IRQNumber % 4;

	// MCU support 16 levels; 4 bits of interrupt priority are used. This means only the 4 most significant bits
	// in the field will be used, as stated by the processor manual
	uint8_t ShiftAmount = ( TargetField * 8) + (NVIC_IPR_FIELD_SIZE - NVIC_MCU_PR_BITS) ;
	NVIC_Ctrl->IPR->REG[TargetReg] |= (IRQPriority << ShiftAmount);
}

void SPI_IRQHandling(SPI_Handle_t* SpiHandler_ptr) {

	// 1. Identify the Interrupt by checking the Status Register
	// Transfer Data interrupt
	if (SpiHandler_ptr->SPIx_ptr->SR & (1 << SPI_SR_TXE) && SpiHandler_ptr->SPIx_ptr->CR2 & (1 << SPI_CR2_TXEIE)) {
		Spi_Txe_Interrupt_Handle();
	}
	// Receive Data interrupt
	else if (SpiHandler_ptr->SPIx_ptr->SR & (1<<SPI_SR_RXNE) && SpiHandler_ptr->SPIx_ptr->CR2 & (1 << SPI_CR2_RXNEIE)) {
		Spi_Rxne_Interrupt_Handle();
	}
	// Overrun Error.
	else if (SpiHandler_ptr->SPIx_ptr->SR & (1<< SPI_SR_OVR) && SpiHandler_ptr->SPIx_ptr->CR2 & (1 << SPI_CR2_ERRIE)) {
		Spi_Ovr_Interrupt_Handle();
	}
	// TODO:: Implement other Error handlers that may occur
}

/*
 * Close Spi Reception/Transmission
 */
void SPI_CloseTransmission(SPI_Handle_t* SpiHandler_ptr) {
	SpiHandler_ptr->SPIx_ptr->CR2 &= ~(1 << SPI_CR2_TXEIE);
	SpiHandler_ptr->TxBuffer_ptr = NULL;
	SpiHandler_ptr->TxState = SPI_STATE_READY;
}
void SPI_CloseReception(SPI_Handle_t* SpiHandler_ptr) {
	// disable the interrupt bit to stop Rxne interrupts
	SpiHandler_ptr->SPIx_ptr->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	SpiHandler_ptr->RxBuffer_ptr = NULL;
	SpiHandler_ptr->RxState = SPI_STATE_READY;
}

void SPI_ClearOVRFlag(SPI_Handle_t* SpiHandler_ptr) {
	uint8_t temp;
	temp = SpiHandler_ptr->SPIx_ptr->DR;
	temp = SpiHandler_ptr->SPIx_ptr->SR;
	(void)temp; // remove unused variable warning
}

/*
 * @INTERRUPT_HANDLE_FX
 * Interrupt Helper functions
 */

static void Spi_Txe_Interrupt_Handle(SPI_Handle_t* SpiHandler_ptr) {

	// 8 bit data format
	if ( ((SpiHandler_ptr->SPIx_ptr->CR1 >> SPI_CR1_DFF) & 0x1 ) == SPI_DFF_8BITS) {
		// Write in to Data Register
		SpiHandler_ptr->SPIx_ptr->DR = *(SpiHandler_ptr->TxBuffer_ptr);
		SpiHandler_ptr->TxBufferLen--;
		SpiHandler_ptr->TxBuffer_ptr++; // move the pointer by 8 bits

	}
	// 16 bit data format
	else {
		SpiHandler_ptr->SPIx_ptr->DR = *(uint16_t*)SpiHandler_ptr->TxBuffer_ptr;
		SpiHandler_ptr->TxBufferLen -= 2;
		(uint16_t*)SpiHandler_ptr->TxBuffer_ptr++; // move the pointer by 16 bits
	}
	// If all data has been sent, close the Spi Txe transmission
	if (SpiHandler_ptr->TxBufferLen == 0) {
		// disable the interrupt bit to stop Txe interrupts
		SPI_CloseTransmission(SpiHandler_ptr);
		SPI_ApplicationEventCallBack(SpiHandler_ptr, SPI_EVENT_TX_COMPLETE); // TODO:: user application ahdnles this
	}

}
static void Spi_Rxne_Interrupt_Handle(SPI_Handle_t* SpiHandler_ptr) {

	if ( ((SpiHandler_ptr->SPIx_ptr->CR1 >> SPI_CR1_DFF) & 0x01) == SPI_DFF_8BITS) {
		// store DR data into buffer
		*SpiHandler_ptr->RxBuffer_ptr = *(uint8_t*)SpiHandler_ptr->SPIx_ptr->DR;
		SpiHandler_ptr->RxBufferLen--;
		SpiHandler_ptr->SPIx_ptr++; // increment pointer by 8 bit to store next value
	}
	// 16 bit DFF
	else {
		*SpiHandler_ptr->RxBuffer_ptr = *(uint8_t*)SpiHandler_ptr->SPIx_ptr->DR;
		SpiHandler_ptr->RxBufferLen -= 2;
		(uint16_t*)SpiHandler_ptr->RxBuffer_ptr++;
	}
	// No more elements to receive
	if (SpiHandler_ptr->RxBufferLen  == 0) {
		SPI_CloseReception(SpiHandler_ptr);
		SPI_ApplicationEventCallBack(SpiHandler_ptr, SPI_EVENT_RX_COMPLETE); // TODO::have user pass function into this
	}

}
/*
 * Basic function that terminates the SPI transmission gracefully.
 * User should decide how to handle the overrun error through their application
 *
 */
static void Spi_Ovr_Interrupt_Handle(SPI_Handle_t* SpiHandler) {

	// This error is specific to data reception, thus action will only be taking during Rx time
	if (SpiHandler->TxState != SPI_STATE_BUSY_TX) {
		SPI_ClearOVRFlag(SpiHandler);
		SPI_ApplicationEventCallBack(SpiHandler, SPI_EVENT_ERROR_OVR);
	}
}

__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t* SpiHandle, uint8_t AppEvent) {
	// Weak Implementation. User should override this function

}
