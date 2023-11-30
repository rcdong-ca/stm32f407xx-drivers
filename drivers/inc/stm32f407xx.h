/*
 * stm32f407.h
 *
 *  Created on: Nov 21, 2023
 *      Author: richard
 *
 *
 * Memory mapping of st32f407CGDISC-1 board
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile  // Acronym for volatile

/* base address of flash and sram memory (based off stm32f4 ref man)*/
#define FLASH_BASE_ADDR 		0x08000000U // where cod is stored
#define SRAM1_BASE_ADDR 		0x20000000U // 112kB capacity
#define SRAM2_BASE_ADDR 		0x2001C000U // 16kB capacity
#define ROM_BASE_ADDR			0x1FFF0000U // system memory


/* base address of AHBx and APBx Peripherals */

#define AHB1_BASE_ADDR			0x40020000U
#define AHB2_BASE_ADDR			0x50000000U
#define AHB3_BASE_ADDR			0x60000000U

#define APB1_BASE_ADDR			0x40000000U
#define APB2_BASE_ADDR			0x40010000U

/* base address of GPIO peripherals (Connected to AHB1 bus) */
#define GPIOA_BASE_ADDR			(AHB1_BASE_ADDR)		// each GPIO address is of 0x400 size
#define GPIOB_BASE_ADDR			(AHB1_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR			(AHB1_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR			(AHB1_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR			(AHB1_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR			(AHB1_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR			(AHB1_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR			(AHB1_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR			(AHB1_BASE_ADDR + 0x2000)

/* Base address of I2C Peripherals (Connected to APB1 Bus) */
#define I2C1_BASE_ADDR			(APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR			(APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR			(APB1_BASE_ADDR + 0x5C00)

/* Base address of UART Peripherlas (Connected to APB1 Bus). Does not include all UART address. Add yourself */
#define UART4_BASE_ADDR			(APB1_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR			(APB1_BASE_ADDR + 0x5000U)


/* Base address for SPI peripherals */
#define SPI2_BASE_ADDR			(APB1_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR			(APB1_BASE_ADDR + 0x3C00)

#define SPI1_BASE_ADDR			(APB2_BASE_ADDR + 0x3000)

/* Base address for USART peripherals */
#define USART2_BASE_ADDR		(APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR		(APB1_BASE_ADDR + 0x4800)

#define USART1_BASE_ADDR		(APB2_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR		(APB2_BASE_ADDR + 0x1400)

/* Base address for EXTI Peripheral */
#define EXTI_BASE_ADDR			(APB2_BASE_ADDR + 0x3C00)

/* Base address for SYSCFG peripheral */
#define SYSCFG_BASE_ADDR		(APB2_BASE_ADDR + 0x3800)


/* Peripheral register structure for RCC */
typedef struct {
	__vo uint32_t CR;  // RCC control register. Offset 0x00
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;		// reserved section of the register
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2]; // reserved section of the register
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVERD6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2CFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/* Peripheral Register structure of GPIO. Available Values are found in stm32f407xx_gpio_drivers.h */

typedef struct {
	__vo uint32_t MODER;			// Mode Register: Available @GPIO_PIN_MODES
	__vo uint32_t OTYPER;			// Output type Register: Available @GPIO_OTYPES
	__vo uint32_t OSPEEDR;			// Output Speed: Available @GPIO_OSPEED
	__vo uint32_t PUPDR;			// Output Speed: Available @GPIO_OSPEED
	__vo uint32_t IDR;				// Input data Register
	__vo uint32_t ODR;				// Output data register
	__vo uint32_t BSRR;				//
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];  // Low is first, high is second
}GPIO_RegDef_t;

/* Reset and Control Clock Registers (RCC reg) and Peripheral Clocks */
#define RCC_BASE_ADDR			(AHB1_BASE_ADDR + 0x3800U)

// Note: Dereferencing random areas in memory may cause segmentation fault, which is why we () again
#define RCC						((RCC_RegDef_t*)RCC_BASE_ADDR)

/* Peripheral Definitions type-casted to xxx_RegDef_t */
#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

/*
 * Clock Enable/Disable macros for GPIOx Peripherals. Note: GPIO are connected to AHB1 BUS.
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= (0 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= (0 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= (0 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= (0 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= (0 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= (0 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= (0 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= (0 << 7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= (0 << 8))

/*
 * Clock Enable/Disable macros for I2Cx Periphrals.
 */
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= (0 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= (0 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= (0 << 23))

/*
 * Clock Enable/Disable macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1 << 15))
#define SPI1_PCLK_DI()					(RCC->APB2ENR &= (0 << 12))
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= (0 << 14))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= (0 << 15))

/*
 * Clock Enable/Disable Macros for UARTx Peripherals
 */
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1 << 20))
#define UART4_PCLK_DI()					(RCC->APB1ENR &= (0 << 19))
#define UART5_PCLK_DI()					(RCC->APB1ENR &= (0 << 20))

/*
 * Clock Enable/Disable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN()				(RCC->APB2ER |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->APB1ER |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ER |= (1 << 18))
#define USART6_PCLK_EN()				(RCC->APB2ER |= (1 << 5))
#define USART1_PCLK_DI()				(RCC->APB2ER &= (0 << 4))
#define USART2_PCLK_DI()				(RCC->APB1ER &= (0 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ER &= (0 << 18))
#define USART6_PCLK_DI()				(RCC->APB2ER &= (0 << 5))

/*
 * Clock Enable/Disable Macro for System Configuration
 */
#define SYSCFEN_PCLK_EN()				(RCC->APB2ER |= (1 << 14))
#define SYSCFEN_PCLK_DI()				(RCC->APB2ER &= (0 << 14))



/* Other macros and Enumerations */
#define	DISABLE							((uint8_t)0)
#define ENABLE							((uint8_t)1)
#define RESET							(DISABLE)
#define SET								(ENABLE)


#endif /* INC_STM32F407XX_H_ */
