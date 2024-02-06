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


void USART_Init(USART_Handle_t* USART_Handler) {

	uint32_t tempReg = 0;
	// Enable Peripheral Clock
	USART_PClockControl(USART_Handler->USARTx_ptr, ENABLE);

	// Configure CR1 register

	// Set Word length
	tempReg |= (USART_Handler->USART_Config.WordLength << USART_CR1_M);

	// Set the Parity control
	tempReg |= (1 << USART_CR1_PCE);
	tempReg |= (USART_Handler->USART_Config.ParitySelection << USART_CR1_PS);


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

	USART_SetBaudRate(USART_Handler);
}
void USART_DeInit(USART_Handle_t* USART_Handler) {
}


uint8_t USART_GetStatus(USART_RegDef_t *USARTx_ptr, uint8_t StatusField) {
	return (USARTx_ptr->SR >> StatusField) & 0x1;
}


void USART_IRQHandling(USART_Handle_t* USART_Handler) {

}




