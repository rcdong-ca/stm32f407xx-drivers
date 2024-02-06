/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Dec 28, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "arm_cortexM4.h"
#include "stm32f407xx.h"
#include "stm32f407xx_general_functions.h"

// Configurable parameters for I2C
typedef struct {
	uint32_t SCLSpeed;			// Serial Clock Speeds from @I2C_SCL
	uint8_t DeviceAddr;
	uint8_t ACKControl;
	uint8_t FMDutyCycle;
}I2C_Config_t;

typedef struct {
	I2C_Config_t I2Cx_Config;
	I2C_RegDef_t* I2Cx_ptr;

	// Used for interrupts
	uint8_t* TxBuffer;
	uint32_t TxLen;
	uint8_t* RxBuffer;
	uint32_t RxLen;
	uint8_t State;					// State of I2C: ready, Busy in Rx/Tx @I2C_APP_STATE
	uint8_t DeviceAddr;				// Slave/device address
	uint8_t RS;						// repeated Start enable/disable
	uint32_t RxSize;
}I2C_Handle_t;

/*
 * @I2C_DEVICE_MODE. Corresponds to the MSL bit in SR2
 */
#define I2C_DEVICE_MODE_MASTER		1
#define I2C_DEVICE_MODE_SLAVE		0

/*
 *@I2C_APP_State I2c Application state
 */
#define I2C_STATE_READY				0
#define I2C_STATE_TX_BUSY			1
#define I2C_STATE_RX_BUSY			2

/*
 * @I2C_SCL available serial clock speeds
 */
#define I2C_SCL_SPEED_SM	100000		// 100 kHz
#define I2C_SCL_SPEED_FM2k	200000
#define I2C_SCL_SPEED_FM4k	400000

/*
 * @I2C_FMDutyCycke
 */
#define I2C_FM_DUTY_2					0  // Tlow = 2*Thigh
#define I2C_FM_DUTY_16_9				1  // Tlow = 1.8*Thigh


/*
 * @I2C Interrupt Event Application Completion
 */
#define I2C_ITEV_STOP_CMPLT				0
#define I2C_ITEV_TX_CMPLT				1
#define I2C_ITEV_RX_CMPLT				2
// Slave Event Flags
#define I2C_ITEV_SLAVE_TXE				3
#define I2C_ITEV_SLAVE_RXNE				4


/*
 * @I2C error flags. From SR1
 */
#define I2C_ERR_BERR					1
#define I2C_ERR_AF						2
#define I2C_ERR_ARLO					3
#define I2C_ERR_OVR						4

/* ************************************ I2C API Functions ********************************/

void I2C_Init(I2C_Handle_t* I2Cx_Handler);
void I2C_DeInit(I2C_Handle_t* I2Cx_Handler);

/*
 * I2C Peripheral Control.
 */
void I2C_PControl(I2C_RegDef_t* I2Cx_ptr, uint8_t En_Di);

/*
 * I2C Blocking Communication API
 */
void I2C_MasterReceiveData(I2C_Handle_t* I2Cx_Handler, uint8_t* RxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterSendData(I2C_Handle_t* I2Cx_Handler, uint8_t* TxBuffer_ptr, uint32_t TxLen, uint8_t SlaveAddr);

/*
 * Get I2C status register info.
 * Return:
 * 	SET (1) or NOT_SET(0)
 */
uint8_t I2C_GetStatus1(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField);  // SR1
uint8_t I2C_GetStatus2(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField);  // SR2

/*
 * I2C Non Blocking Master receive
 * Input:
 * 	I2C_Handler
 * 	RxBuffer: receive buffer
 * 	Len: Length of data being received
 * 	SlaveAddr: Address of slave device that is transmitting data to master
 * 	RS: Repeat Start Enable/Disable
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* I2Cx_Handler, uint8_t* RxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS);
/*
 * I2C Non Blocking Master Transmit
 * Input:
 * 	I2C_Handler
 * 	TxBuffer: Transmit Buffer
 * 	Len: Length of data being transmitted
 * 	SlaveAddr: Address of slave device that is transmitting data to master
 * 	RS: Repeat Start Enable/Disable
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* I2Cx_Handler, uint8_t* TxBuffer_ptr, uint32_t TxLen, uint8_t SlaveAddr, uint8_t RS);

/*
 * I2C Interrupt Event Handle
 */
void I2C_EV_IRQHandling(I2C_Handle_t* I2Cx_Handler);
/*
 * I2C Error Interrupt Handle
 */
void I2C_ERR_IRQHandling(I2C_Handle_t* I2Cx_Handler);

/*
 * I2C Slave Send Data and Receive Data
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* I2Cx_ptr);
void I2C_SlaveSendData(I2C_RegDef_t* I2Cx_ptr, uint8_t data);

/*
 * User application Call back for interrupt event. Can be overwritten
 */
void I2C_ApplicationEventCallBack(I2C_Handle_t* I2Cx_Handler, uint8_t AppEvent);

/*
 * Peripheral Clock controller
 */

void I2C_PCLKControl(I2C_Handle_t* I2Cx_Handler, uint8_t En_Di);  // Peripheral Clock controller


/*
 * Close Tranmissions and Receive call
 */
void I2C_Close_Rx(I2C_Handle_t *I2Cx_Handler);
void I2C_Close_Tx(I2C_Handle_t *I2Cx_Handler);

/*
 * Reset I2C Peripheral to default state
 */
#define I2C1_RESET()				{RCC->APB1RSTR |= (1 << 5); RCC->APB1RSTR &= ~(1 << 5); }
#define I2C2_RESET()				{RCC->APB1RSTR |= (1 << 6); RCC->APB1RSTR &= ~(1 << 6); }
#define I2C3_RESET()				{RCC->APB1RSTR |= (1 << 7); RCC->APB1RSTR &= ~(1 << 7); }


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
