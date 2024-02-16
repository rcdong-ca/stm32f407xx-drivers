/*
 * stm32f407xx_uart_driver.c
 *
 *  Created on: Jan 27, 2024
 *      Author: richard
 *
 *
 *
 *      NOTE: WHILE THIS DRIVER API ONLY SUPPORTS UART; WE DO NOT TAKE CLOCK INTO CONSIDERATION
 */


#include "stm32f407xx_uart_driver.h"
#include <stddef.h>

void USART_PeripheralControl(USART_RegDef_t* USARTx_ptr, uint8_t En_Di) {
	if (En_Di == ENABLE) {
		USARTx_ptr->CR1 |= (1 << USART_CR1_UE);
	}
	else if (En_Di == DISABLE) {
		USARTx_ptr->CR1 |= ~(1 << USART_CR1_UE);
	}
}

void USART_PClockControl(USART_RegDef_t* UARTx_ptr, uint8_t En_Di) {
	if (En_Di == ENABLE) {
		if (UARTx_ptr == USART1) {
			USART1_PCLK_EN();
		}
		else if (UARTx_ptr == USART2) {
			USART2_PCLK_EN();
		}
		else if (UARTx_ptr == USART3) {
				USART3_PCLK_EN();
		}
		else if (UARTx_ptr == UART4) {
			UART4_PCLK_EN();
		}
		else if (UARTx_ptr == UART5) {
			UART5_PCLK_EN();
		}
		else if (UARTx_ptr == USART6) {
			USART6_PCLK_EN();
		}
	}
	else if (En_Di == DISABLE) {
		if (UARTx_ptr == USART1) {
			USART1_PCLK_DI();
		}
		else if (UARTx_ptr == USART2) {
			USART2_PCLK_DI();
		}
		else if (UARTx_ptr == USART3) {
			USART3_PCLK_DI();
		}
		else if (UARTx_ptr == UART4) {
			UART4_PCLK_DI();
		}
		else if (UARTx_ptr == UART5) {
			UART5_PCLK_DI();
		}
		else if (UARTx_ptr == USART6) {
			USART6_PCLK_DI();
		}
	}
}

/*
 * Baud Rate control. Only Supports Standard Mode
 */
void USART_SetBaudRate(USART_RegDef_t* USARTx_ptr, uint32_t BaudRate) {

	uint16_t Mantissa = 0;  // Mantissa is the whole number part. Only 12 bits will be used
	uint8_t Fraction = 0;  // Fractino of USART_DIV
	uint8_t OverSampling = (USARTx_ptr->CR1 >> USART_CR1_OVER8) & 0x1;

	// USART peripherals can be found on APB1 and APB2. Determine which clock to use
	uint32_t ClockSpeed = RCC_GetPCLK1Value();
	if (USARTx_ptr == USART1 || USARTx_ptr == USART6) {
		ClockSpeed = RCC_GetPCLK2Value();
	}

	// Equation: USART_DIV = fclk / (8 * (2-OVER8) * TxRxBaudRate). Multiply by 100 to get fraction
	uint32_t UsartDiv = (ClockSpeed * 100) / (8 * (2 - OverSampling) * BaudRate);

	Mantissa = UsartDiv / 100;
	Fraction = (UsartDiv - Mantissa*100);

	uint32_t TempReg = 0;
	TempReg |= (Mantissa << USART_BRR_MANTISSA);
	TempReg |= ((Fraction << USART_BRR_FRACTION) & 0xF); // 4 bit long

	USARTx_ptr->BRR = TempReg;
}


void USART_Init(USART_Handle_t* USART_Handler) {

	uint32_t tempReg = 0;
	// Enable Peripheral Clock
	USART_PClockControl(USART_Handler->USARTx_ptr, ENABLE);

	// Configure CR1 register

	// Set Word length
	tempReg |= (USART_Handler->USART_Config.WordLength << USART_CR1_M);

	// Set the Parity control
	if (USART_Handler->USART_Config.ParityControl == ENABLE) {
		tempReg |= (1 << USART_CR1_PCE);
		tempReg |= (USART_Handler->USART_Config.ParitySelection << USART_CR1_PS);
	}


	// Conifgure the device mode: Receiver or Transmitter
	if (USART_Handler->USART_Config.Mode == USART_DEVICE_MODE_RX) {
		tempReg |= (1 << USART_CR1_RE);
	}
	else if (USART_Handler->USART_Config.Mode == USART_DEVICE_MODE_RX) {
		tempReg |= (1 << USART_CR1_TE);
	}
	else if (USART_Handler->USART_Config.Mode == USART_DEVICE_MODE_TXRX) {
		tempReg |= (1 << USART_CR1_TE);
		tempReg |= (1 << USART_CR1_RE);
	}

	// Set the CR1 Register
	USART_Handler->USARTx_ptr->CR1 = tempReg;


	tempReg = 0;
	// Configure Cr2 Register
	// Set the stop bits
	tempReg |= (USART_Handler->USART_Config.NumOfStopBits << USART_CR2_STOP);
	USART_Handler->USARTx_ptr->CR2 = tempReg;

	tempReg =0;
	// Configure CR3 Register
	if (USART_Handler->USART_Config.HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		tempReg = (1 << USART_CR3_CTSE);
	}
	else if (USART_Handler->USART_Config.HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		tempReg = (1 << USART_CR3_RTSE);
	}
	else if (USART_Handler->USART_Config.HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		tempReg = (1 << USART_CR3_RTSE);
		tempReg = (1 << USART_CR3_CTSE);
	}

	USART_SetBaudRate(USART_Handler->USARTx_ptr, USART_Handler->USART_Config.BaudRate);
}
void USART_DeInit(USART_Handle_t* USART_Handler) {
}


uint8_t USART_GetStatus(USART_RegDef_t *USARTx_ptr, uint8_t StatusField) {
	return (USARTx_ptr->SR >> StatusField) & 0x1;
}

void USART_ClearStatus(USART_RegDef_t* USARTx_ptr, uint8_t StatusField) {
	if (StatusField == USART_SR_TC) {
		// read SR and write 0 to DR
		(void)USARTx_ptr->SR;
		USARTx_ptr->DR = 0;
	}
}


/*
 * USART Send and Receive. Blocking calls
 */
void USART_SendData(USART_Handle_t* USART_Handler, uint8_t* TxBuffer, uint32_t Len) {

	while (Len > 0 ) {
		// wait for data register to be empty
		while (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_TXE) == NOT_SET);
		if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_8) {
			// Parity Enabled will not affect how we increment buffer.
			USART_Handler->USARTx_ptr->DR = *TxBuffer;
			TxBuffer++;
		}
		else if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_9) {
			// Parity Enabled will affect how we increment buffer
			if (USART_Handler->USART_Config.ParityControl == ENABLE) {
				// Send 8 bits of data
				USART_Handler->USARTx_ptr->DR = *TxBuffer;
				TxBuffer++;
			}
			else if (USART_Handler->USART_Config.ParityControl == DISABLE) {
				// Send 9 bits of data
				USART_Handler->USARTx_ptr->DR = *(uint16_t*)TxBuffer &0x1FF;
				TxBuffer += 2;
			}
		}
		Len--;
	}

	// Wait for TC status to be set
	while (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_TC) == NOT_SET);
	// Clear the TC status
	USART_ClearStatus(USART_Handler->USARTx_ptr, USART_SR_TC);
}

void USART_ReceiveData(USART_Handle_t* USART_Handler, uint8_t* RxBuffer, uint32_t Len) {

	while (Len > 0) {

		// wait for RXNE bit to be set. Data has arrived in DR
		while (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_RXNE) == NOT_SET);

		if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_8) {
			// Data can go directly into buffer
			*RxBuffer = USART_Handler->USARTx_ptr->DR;
		}
		else if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_9) {
			if (USART_Handler->USART_Config.ParityControl == ENABLE) {
				// data can go directly into buffer
				*RxBuffer = USART_Handler->USARTx_ptr->DR;
			}
			else {
				// 9 bits of data
				*(uint16_t*)RxBuffer = USART_Handler->USARTx_ptr->DR;
				RxBuffer+=2;
			}
		}
		Len--;
	}
}


/******************************** USART Interrupt API calls *************************************/

uint8_t USART_SendDataIT(USART_Handle_t* USART_Handler, uint8_t *TxBuffer, uint32_t Len) {

		// Check if it is already in TxState
		if (USART_Handler->TxState != USART_STATE_TX_BUSY) {
			USART_Handler->TxBuffer = TxBuffer;
			USART_Handler->TxLen = Len;
			USART_Handler->TxState = USART_STATE_TX_BUSY;

			// Enable Send Interrupts
			USART_Handler->USARTx_ptr->CR1 |= (1 << USART_CR1_TXEIE);
			USART_Handler->USARTx_ptr->CR1 |= (1 << USART_CR1_TCIE);
			return 1;
		}
		return 0;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t* USART_Handler, uint8_t *RxBuffer, uint32_t Len) {

		// Check if it is already in TxState
		if (USART_Handler->RxState != USART_STATE_RX_BUSY) {
			USART_Handler->RxBuffer = RxBuffer;
			USART_Handler->RxLen = Len;
			USART_Handler->RxState = USART_STATE_RX_BUSY;

			// Enable Send Interrupts
			USART_Handler->USARTx_ptr->CR1 |= (1 << USART_CR1_RXNEIE);
			return 1;
		}
		return 0;
}

static void USART_TC_EV_IT_Handle(USART_Handle_t* USART_Handler) {
	// close the Tramission

	// Clear the TC Flag
	(void)USART_Handler->USARTx_ptr->SR;
	// double check all data has been sent

	if (USART_Handler->TxLen != 0)
		return;

	// Clear TCIE control bit
	USART_Handler->USARTx_ptr->CR1 &= ~(1 << USART_CR1_TCIE);

	// Reset Peripheral state
	USART_Handler->TxState = USART_STATE_READY;

	// reset buffer and length
	USART_Handler->TxBuffer = NULL;
	USART_Handler->TxLen = 0;
	// Notify application of Transmission completion
	USART_ApplicationEventCallBack(USART_Handler, USART_ITEV_TX_CMPLT);
}

static void USART_TXE_EV_IT_Handle(USART_Handle_t* USART_Handler) {
	if (USART_Handler->TxLen > 0 ) {
			// wait for data register to be empty
			if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_8) {
				// Parity Enabled will not affect how we increment buffer.
				USART_Handler->USARTx_ptr->DR = *USART_Handler->TxBuffer;
				USART_Handler->TxBuffer++;
			}
			else if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_9) {
				// Parity Enabled will affect how we increment buffer
				if (USART_Handler->USART_Config.ParityControl == ENABLE) {
					// Send 8 bits of data
					USART_Handler->USARTx_ptr->DR = *USART_Handler->TxBuffer;
					USART_Handler->TxBuffer++;
				}
				else if (USART_Handler->USART_Config.ParityControl == DISABLE) {
					// Send 9 bits of data
					USART_Handler->USARTx_ptr->DR = *(uint16_t*)USART_Handler->TxBuffer & 0x1FF;
					USART_Handler->TxBuffer += 2;
				}
			}
			USART_Handler->TxLen--;
		}
}

void USART_RXNE_EV_IT_Handle(USART_Handle_t* USART_Handler) {
	if (USART_Handler->RxLen > 0) {

			if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_8) {
				// Data can go directly into buffer
				*USART_Handler->RxBuffer = USART_Handler->USARTx_ptr->DR;
			}
			else if (USART_Handler->USART_Config.WordLength == USART_WORDLEN_9) {
				if (USART_Handler->USART_Config.ParityControl == ENABLE) {
					// data can go directly into buffer
					*USART_Handler->RxBuffer = USART_Handler->USARTx_ptr->DR;
				}
				else {
					// 9 bits of data
					*USART_Handler->RxBuffer = USART_Handler->USARTx_ptr->DR;
					USART_Handler->RxBuffer+=2;
				}
			}
			USART_Handler->RxLen--;
		}
}

void USART_EVIRQHandling(USART_Handle_t* USART_Handler) {

	// TXE event
	if (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_TXE) == SET) {

		// Check if Transmission has been completed
		if (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_TC) == SET) {
			USART_TC_EV_IT_Handle(USART_Handler);
		}
		else {  // Send the remaining data
			USART_TXE_EV_IT_Handle(USART_Handler);
		}
	}
	// RXNE event
	if (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_RXNE) == SET) {
		USART_RXNE_EV_IT_Handle(USART_Handler);
	}

	// CTS Event
	if (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_CTS) == SET) {

	}

	// Idle Character event
	if (USART_GetStatus(USART_Handler->USARTx_ptr, USART_SR_IDLE) == SET) {
	}
}
void USART_ERRIRQHandling(USART_Handle_t* USART_Handler) {

}


__attribute__((weak)) void USART_ApplicationEventCallBack(USART_Handle_t* USART_Handler, uint8_t AppEvent) {
	// Weak Implementation. User should override this function

}





