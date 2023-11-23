/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 22, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


/*
 * APIs supported by GPIO driver
 */

void GPIO_Init(void);
void GPIO_DeInit(void);
void GPIO_Read_From_Input_Pin(void);
void GPIO_Read_From_Input_Port(void);
void GPIO_Write_To_Output_Pin(void);
void GPIO_Write_To_Output_Port(void);
void GPIO_Toggle_Output_Pin(void);
void GPIO_IRQ_Config(void);	// Interrupt requests configuration
void GPIO_IRQ_Handling(void);
void GPIO_PERI_CLK_Control(void);  // Peripheral Clock controller

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
