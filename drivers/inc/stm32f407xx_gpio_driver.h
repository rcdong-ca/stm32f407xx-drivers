/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 22, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"
#include "arm_cortexM4.h"

/* Pin configuration structure for GPIO*/
typedef struct {
	uint8_t PinNumber;  			// Port x has up to 16 pins: 0-15
	uint8_t PinMode;   				// Possible Pin modes: @GPIO_PIN_MODES
	uint8_t PinOutputSpeed;			// Pin Output Speeds: @GPIO_OSPEED
	uint8_t PinOutputType;			// Pin Output Types: @GPIO_OTYPE
	uint8_t PinPUpPDo;				// Pull up and Pull down options: @GPIO_PUPD
	uint8_t PinAltFunMode;			// Alternate function written for pins 0-15

}GPIO_PinConfig_t;

/* GPIO handler for users to configure GPIO */
typedef struct {
	GPIO_RegDef_t* GPIOx_ptr;			// holds the base address of GPIO port
	GPIO_PinConfig_t GPIOx_PinConfig;


}GPIO_Handle_t;

/*
 * APIs supported by GPIO driver
 */

void GPIO_Init(GPIO_Handle_t* GPIOx_Handler);
void GPIO_DeInit(GPIO_Handle_t* GPIOx_Handler);

uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t* GPIOx_Handler);
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t* GPIOx_Handler);

uint16_t GPIO_ReadFromOutputPort(GPIO_Handle_t* GPIOx_Handler);
void GPIO_WriteToOutputPin(GPIO_Handle_t* GPIOx_Handler, uint8_t Val);
void GPIO_WriteToOutputPort(GPIO_Handle_t* GPIOx_Handler, uint16_t OutputData);
void GPIO_ToggleOutputPin(GPIO_Handle_t* GPIOx_Handler);

void GPIO_ToggleOutputPinDirect(GPIO_RegDef_t* GpioPort, uint8_t PinNumber);


/*
 * IRQ (Interrupt request) configuration and ISR (Interrupt Signal Response) handling
 */

void GPIO_IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI);	// Interrupt requests configuration
void GPIO_IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * Peripheral Clock controller for GPIO
 */

void GPIO_PCLKControl(GPIO_Handle_t* GPIOx_Handler, uint8_t En_Di);  // Peripheral Clock controller


/* @GPIO_PIN_MODES
 * GPIO Pin Modes
 */
#define GPIO_MODE_IN 			0		// Input Mode
#define GPIO_MODE_OUT 			1		// Output Mode
#define GPIO_MODE_ALTFUN		2		// Alternating functions mode
#define GPIO_MODE_ANALOG		3		// Analog mode
// GPIOs can be configured to detect interrupts that will be sent to processor
//TODO:: Re-evaluate if GPIO Interrupt triggers should  be separate modes
#define GPIO_MODE_IN_RT			4		// External Interrupt: Rising Trigger
#define GPIO_MODE_IN_FT			5		// External Interrupt: Falling Trigger
#define GPIO_MODE_IN_RFT		6		// External Interrupt: Rising and Falling Trigger

/* @GPIO_OTYPES
 * GPIO Output Types
 */
#define GPIO_OTYPE_PP			0		// PUSH PULL (reset state)
#define GPIO_OTYPE_OD			1		// OPEN DRAIN

/* @GPIO_OSPEED
 * GPIO Output speed
 */
#define GPIO_OSPEED_LOW			0		// Low speed
#define GPIO_OSPEED_MED			1		// Medium Speed
#define GPIO_OSPEED_HIGH		2		// High speed
#define GPIO_SPEED_VHIGH		3		// Very high speed

/* @GPIO_PUPD
 * GPIO port Pull Down Pull Up mode
 */
#define GPIO_PUPD_NO			0		// No Pull up and Pull Down is used
#define GPIO_PU					1		// Pull up only
#define GPIO_PD					2		// Pull Down only

/* @GPIO_RESET
 * Reset GPIO peripherals (all pins and registers)
 */
#define GPIOA_RESET()			{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); }
#define GPIOB_RESET()			{RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); }
#define GPIOC_RESET()			{RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); }
#define GPIOD_RESET()			{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); }
#define GPIOE_RESET()			{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); }
#define GPIOF_RESET()			{RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); }
#define GPIOG_RESET()			{RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); }
#define GPIOH_RESET()			{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); }
#define GPIOI_RESET()			{RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); }


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
