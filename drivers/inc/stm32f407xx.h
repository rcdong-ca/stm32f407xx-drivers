/*
 * stm32f407.h
 *
 *  Created on: Nov 21, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_



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

/* base address of GPIO peripherals (Connected to AHB1 bus) */
#define GPIOA_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 0)	// each GPIO address is of 0x400 size
#define GPIOB_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 1)
#define GPIOC_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 2)
#define GPIOD_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 3)
#define GPIOE_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 4)
#define GPIOF_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 5)
#define GPIOG_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 6)
#define GPIOH_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 7)
#define GPIOI_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 8)
#define GPIOJ_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 9)
#define GPIOK_BASE_ADDR		AHB1_BASE_ADDR + (0x0400UL * 10)

/* Base address of I2C Peripherals (Connected to APB1 Bus) */
#define I2C1_BASE_ADDR		APB1_BASE_ADDR + 0x5400UL
#define I2C2_BASE_ADDR		APB1_BASE_ADDR + 0x5800UL
#define I2C3_BASE_ADDR		APB1_BASE_ADDR + 0x5C00UL

/* Base address of UART Peripherlas (Connected to APB1 Bus). Does not include all UART address. Add yourself */
#define UART4		APB1_BASE_ADDR + 0x4C00UL
#define UART5		APB1_BASE_ADDR + 0x5000UL


/* Base address for SPI peripherals */
#define SPI2		APB1_BASE_ADDR + 0x3800UL
#define SPI3		APB1_BASE_ADDR + 0x3C00UL

#define SPI1		APB2_BASE_ADDR + 0x3000UL

/* Base address for USART peripherals */
#define USART2		APB1_BASE_ADDR + 0x4400UL
#define USART3		APB1_BASE_ADDR + 0x4800UL

#define USART1		APB2_BASE_ADDR + 0x1000UL
#define USART6		APB2_BASE_ADDR + 0x1400UL

/* Base address for EXTI Peripheral */
#define EXTI		APB2_BASE_ADDR + 0x3C00UL

/* Base address for SYSCFG peripheral */
#define SYSCFG_BASE_ADDR		APB2_BASE_ADDR + 0x4001 3800UL


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

#endif /* INC_STM32F407XX_H_ */
