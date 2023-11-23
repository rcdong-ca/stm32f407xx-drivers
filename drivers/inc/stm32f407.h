/*
 * stm32f407.h
 *
 *  Created on: Nov 21, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_



/* base address of flash and sram memory (based off stm32f4 ref man)*/
#define FLASH_BASE_ADDR 	0x08000000UL // where cod is stored
#define SRAM1_BASE_ADDR 	0x20000000UL // 112kB capacity
#define SRAM2_BASE_ADDR 	0x2001C000UL // 16kB capacity
#define ROM_BASE_ADDR		0x1FFF0000UL // system memory


/* base address of AHBx and APBx Peripherals */

#define AHB1_BASE_ADDR		0x4002 0000UL
#define AHB2_BASE_ADDR		0x5000 0000UL
#define AHB3_BASE_ADDR		0x6000 0000UL

#define APB1_BASE_ADDR		0x4000 0000UL
#define APB2_BASE_ADDR		0x4001 0000UL


/*
 * TODO::
 * 1. Define macros for other APB1 Peripherals, as we will be writing drivers for
 * GPIO, I2C, UART, SPI, etc.
 *
 * 2. write the GPIO general structure! Note, these are registers, maybe consider which
 * registers are volatile.
 *
 * 3. Create macros to access GPIO A -> GPIO I.
 * 4. Do this for the RCC and periperhal clocks
 * 5. Write Macro for clock enables for GPIO peripherals, I2C clock peripheral, etc.
 * 6. Write macro for clock disables, do this for our various peripherals
 *
 */

#endif /* INC_STM32F407_H_ */
