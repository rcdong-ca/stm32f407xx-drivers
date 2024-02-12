/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Jan 27, 2024
 *      Author: richard
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_

#include "stm32f407xx.h"
#include "stm32f407xx_rcc_driver.h"
#include "arm_cortexM4.h"

typedef struct {
	uint8_t Mode;
	uint32_t BaudRate;
	uint8_t NumOfStopBits;
	uint8_t WordLength;
	uint8_t ParityControl;		// Enable Parity or Disable Parity
	uint8_t ParitySelection; 	// Odd or even parity @USART_PARITY
	uint8_t HWFlowControl;
}USART_Config_t;

typedef struct {
	USART_RegDef_t* USARTx_ptr;
	USART_Config_t USART_Config;

	uint8_t TxState;				// Tx State: based off @USART_STATE_BUSY
	uint8_t *TxBuffer;
	uint32_t TxLen;

	uint8_t RxState;
	uint8_t* RxBuffer;
	uint32_t RxLen;

}USART_Handle_t;


/***************************** USART API CALLS *********************************/


/*
 * Peripheral Clock control: Enable or Disable
 */
void USART_PClockControl(USART_RegDef_t *USARTx_ptr, uint8_t En_Di);

/*
 * USART Initialization and De-Initialization
 * Note:: Only Supports UART for time being. No use of clock for now
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

void USART_IRQHandling(USART_Handle_t* USART_Handler);


/*
 * Other Peripheral Control APIs
 */

uint8_t USART_GetStatus(USART_RegDef_t *USARTx_ptr, uint8_t StatusField);
void USART_ClearStatus(USART_RegDef_t* USARTx_ptr, uint8_t StatusField);
void USART_PeripheralControl(USART_RegDef_t* USARTx_ptr, uint8_t En_Di);
void USART_SetBaudRate(USART_RegDef_t* USARTx_ptr, uint32_t BaudRate);


/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t* USART_Handler, uint8_t AppEvent);


/************ USART MACROS AND OTHERS****************/

/*
 * @USART_DEVICE_MODE
 */
#define USART_DEVICE_MODE_TX					0
#define USART_DEVICE_MODE_RX					1
#define USART_DEVICE_MODE_TXRX					2

/*
 * @USART_STATE_BUSY
 */
#define USART_STATE_READY						0
#define USART_STATE_RX_BUSY						1
#define USART_STATE_TX_BUSY						2

/*
 * @USART_OVERSAMPLING. Used for buad rate calculation
 */
#define USART_OVERSAMPLE_8						1
#define USART_OVERAMPLE_16						0

/*
 * @USART_PARITY selection
 */
#define USART_PARITY_EVEN						0
#define USART_PARITY_ODD						1

/*
 * @USART_NStopBits. Number of Stop Bits transmitted after every character
 */
#define USART_STOPBITS_1						0  // default
#define USART_STOPBITS_0_5						1  // Receiving Data in Smart Mode
#define USART_STOPBITS_2						2  // USART, single-wire and modem modes ONLY
#define USART_STOPBITS_1_5						3  // Tx and Rx data in Smart Mode

/*
 * @USART_WordLen. Number of bits to be sent as 1 word
 */
#define USART_WORDLEN_8							0
#define USART_WORDLEN_9							1

/*
 * @USARt_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE					0
#define USART_HW_FLOW_CTRL_CTS					1
#define USART_HW_FLOW_CTRL_RTS					2
#define USART_HW_FLOW_CTRL_CTS_RTS				3

/*
 * @USART_BAUD
 * Supported Baud rates. More baud rates supported, please refer to RefManual
 */
#define USART_BAUD_1200							1200
#define USART_BAUD_2400							2400
#define USART_BAUD_9600							9600
#define USART_BAUD_19200						19200



#endif /* INC_STM32F407XX_UART_DRIVER_H_ */
