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
}I2C_Handle_t;

/*
 * @I2C_SCL available serial clock speeds
 */
#define I2C_SCL_SPEED_SM	100000		// 100 kHz
#define I2C_SCL_SPEED_FM2k	200000
#define I2C_SCL_SPEED_FM4k	400000

/*
 * @I2C_ACK_CONTROL
 */
#define I2C_ACK_ENABLE					1
#define I2C_ACK_DISABLE					0

/*
 * @I2C_FMDutyCycke
 */
#define I2C_FM_DUTY_2					0  // Tlow = 2*Thigh
#define I2C_FM_DUTY_16_9				1  // Tlow = 1.8*Thigh

/*
 * @I2C_AckControl
 */
#define I2C_ACK_CTRL_ENABLE				1
#define I2C_ACK_CTRL_DISABLE			0


/*
 * I2C API
 */

void I2C_Init(I2C_Handle_t* I2Cx_Handler);
void I2C_DeInit(I2C_Handle_t* I2Cx_Handler);

/*
 * I2C Peripheral Control.
 */
void I2C_PControl(I2C_RegDef_t* I2Cx_ptr, uint8_t En_Di);

/*
 * Get I2C status register info.
 * Return:
 * 	SET (1) or NOT_SET(0)
 */
uint8_t I2C_GetStatus1(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField);  // SR1
uint8_t I2C_GetStatus2(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField);  // SR2

/*
 * IRQ (Interrupt request) configuration and ISR (Interrupt Signal Response) handling
 */

void I2C_IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI);	// Interrupt requests configuration
void I2C_IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Peripheral Clock controller
 */

void I2C_PCLKControl(I2C_Handle_t* I2Cx_Handler, uint8_t En_Di);  // Peripheral Clock controller

/*
 * Reset I2C Peripheral to default state
 */
#define I2C1_RESET()				{RCC->APB1RSTR |= (1 << 5); RCC->APB1RSTR &= ~(1 << 5); }
#define I2C2_RESET()				{RCC->APB1RSTR |= (1 << 6); RCC->APB1RSTR &= ~(1 << 6); }
#define I2C3_RESET()				{RCC->APB1RSTR |= (1 << 7); RCC->APB1RSTR &= ~(1 << 7); }

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
