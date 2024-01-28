/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Jan 27, 2024
 *      Author: richard
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t Mode;
	uint32_t BaudRate;
	uint8_t NumOfStopBits;
	uint8_t WordLength;
	uint8_t ParityControl;
	uint8_t HWFlowControl;
}USART_Config_t;

typedef struct {
	USART_RegDef_t* USARTx_ptr;
	USART_Config_t USART_Config;
}USART_Handle_t;


/***************************** USART API CALLS *********************************/


/*
 * Peripheral Clock control: Enable or Disable
 */
void USART_PClockControl(USART_RegDef_t *USARTx_ptr, uint8_t En_Di);

/*
 * USART Initialization and De-Initialization
 */
void USART_Init(USART_Handle_t* USART_Handler);
void USART_DeInit(USART_Handle_t* USART_Handler);

/*
 * Send And Receive Blocking function
 */
void USART_SendData(USART_Handle_t* USART_Handler, uint8_t *TxBuffer, uint32_t Len);
void  USART_ReceiveData(USART_Handle_t* USART_Handler, uint8_t* RxBuffer, uint32_t Len);

/*
 * Send and Receive Non-blocking function
 */
uint8_t USART_SendDataIT(USART_Handle_t* USART_Handler, uint8_t *TxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t* USART_Handler, uint8_t* RxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_Di);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t* USART_Handler);


/*
 * Other Peripheral Control APIs
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *USARTx_ptr, uint8_t StatusField);
void USART_ClearFlag(USART_RegDef_t* USARTx_ptr, uint16_t StatusField);
void USART_PeripheralControl(USART_RegDef_t* USARTx_ptr, uint8_t En_Di);
void USART_SetBaudRate(USART_RegDef_t* USARTx_ptr, uint32_t BaudRate);


/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t* USART_Handler, uint8_t AppEvent);


#endif /* INC_STM32F407XX_UART_DRIVER_H_ */
