/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 22, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* Pin configuration structure for GPIO*/
typedef struct {
	uint8_t PinNumber;  // Port x has up to 16 pins. 8 bits is enough to represent 0-15
	uint8_t PinMode;   // I/O direction mode for pins. 4 modes: 0-Input, 1-Ouput, 2-AltFun, 3-Analog
	uint8_t PinOutputSpeed;
	uint8_t PinOutputType;
	uint8_t PinPUpPDo;		// Pull up or Pull Down state
	uint8_t PinAltFunMode;	// Alternate function written for pins 0-15

}GPIO_PinConfig_t;

/* GPIO handler for users to configure GPIO */
typedef struct {
	GPIO_RegDef_t* GPIOx_ptr;			// holds the base address of GPIO port
	GPIO_PinConfig* GIOx_PinConfig;


}GPIO_Handle_t;

/*
 * APIs supported by GPIO driver
 */

void GPIO_Init(GPIO_Handle_t* GPIOx_Handler);
void GPIO_DeInit(GPIO_RegDef_t* GPIxO_ptr);  // reset all registers for GPIOx port
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* GPIOx_ptr, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* GPIOx_ptr);  // 16 pins so 16 bit int
void GPIO_WriteToOutputPin(GPIO_RegDef_t* GPIOx_ptr, uint8_t PinNumber, uint8 Val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* GPIOx_ptr, uint16_t OutputData);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* GPIOx_ptr, uint8_t PinNUmber, uint8_t EN_DI); //


/*
 * IRQ (Interrupt request) configuration and ISR (Interrupt Signal Response) handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t EN_DI);	// Interrupt requests configuration

void GPIO_IRQHandling(GPIO_RegDef_t* GPIOx_ptr, uint8_t PinNumber);

/*
 * Input:
 * 		GPIOx_ptr: Pointer to a GPIO port
 * 		En_Di: ENABLE or DISABLE the Peripheral clock for GPIOx port
 */
void GPIO_PCLKControl(GPIO_RegDef_t* GPIOx_ptr, uint8_t En_Di);  // Peripheral Clock controller

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
