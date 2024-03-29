/*
 * stm32f407.h
 *
 *  Created on: Nov 21, 2023
 *      Author: richard
 *
 *
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile  // Acronym for volatile


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

/* EXTI Register Structure Peripheral */

typedef struct {
	uint32_t IMR;				// Interrupt Mask Register. 0 to mask a line. 1 to unmask a line
	uint32_t EMR;				// Event Mask Register. 0 to mask, 1 to unmask
	uint32_t RTSR;				// Rising Trigger Selection Reg. 0: Disable line x, 1: enable line x
	uint32_t FTSR;				// Falling Trigger Selection Reg. Same as RTSR
	__vo uint32_t SWIER;		// Software Interrupt event Reg. 1: Interrupt Request Generation. Cleared by writing 1 to corresponding bit in PR reg
	__vo uint32_t PR;			// Pending Request. 1: Selected trigger request occured.0: no trigger request occured
}EXTI_RegDef_t;

/* SYSCFG Register Structure Peripheral */
typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];		// External Interrupt Configuration Reg. Each covers 4 pins
	__vo uint32_t CMPCR;		// Compensation Cell Reg. 0:I/O not ready. 1: O ready. 8 and 0 bit only
}SYSCFG_RegDef_t;

/*
 * SPI Peripheral Register structure
 */
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2CCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * I2C Peripheral Register Structure
 */
typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;  // Own Address Register. Used by slave
	__vo uint32_t OAR2;  // Own Address Register. Used by slave
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;  // Clock control register
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;  // Noise filter register
}I2C_RegDef_t;


/*
 * USART Peripheral Registers structure
 */
typedef struct {
	__vo uint32_t SR;			// Status Register
	__vo uint32_t DR;			// Data Register
	uint32_t BRR;				// Baud Rate Register
	uint32_t CR1;				// Control Registers 1->3
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;				// Guard Time and Prescaler Register
}USART_RegDef_t;


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

#define EXTI					((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASE_ADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASE_ADDR)

/*
 * USART: Universal Sync/Async ReceiveTransmit
 */
#define USART1					((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2					((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3					((USART_RegDef_t*)USART3_BASE_ADDR)
#define USART6					((USART_RegDef_t*)USART6_BASE_ADDR)
/*
 * UART: Universal Async Receive Transmit. These will share the same registers as USART, but lacking
 * the Synchronous feature
 */
#define UART4					((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5					((USART_RegDef_t*)UART5_BASE_ADDR)


/*
 * IRQ (Interrupt Requests) Interrupt Position/Number
 */
// External Interrupt Controller lines
#define IRQ_NO_EXTI0						6
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40

// SPI IRQ Positions
#define IRQ_NO_SPI1							35
#define IRQ_NO_SPI2							36
#define IRQ_NO_SPI3							51

// I2C IRQ Positions
#define IRQ_NO_I2C1_EV						31
#define IRQ_NO_I2C1_ER						32
#define IRQ_NO_I2C2_EV						33
#define IRQ_NO_I2C2_ER						34
#define IRQ_NO_I2C3_EV						79
#define IRQ_NO_I2C3_ER						80

// USART IRQ positions
#define IRQ_NO_USART1						37
#define IRQ_NO_USART2						38
#define IRQ_NO_USART3						39
#define IRQ_NO_UART4						52
#define IRQ_NO_UART5						53
#define IRQ_NO_USART6						71

/*
 * IRQ Priority Levels/Value. 16 Programmable levels used for NVIC_IPR_REG
 */
#define NVIC_IRQ_PRI0					0
#define NVIC_IRQ_PRI1					1
#define NVIC_IRQ_PRI2					2
#define NVIC_IRQ_PRI3					3
#define NVIC_IRQ_PRI4					4
#define NVIC_IRQ_PRI5					5
#define NVIC_IRQ_PRI6					6
#define NVIC_IRQ_PRI7					7
#define NVIC_IRQ_PRI8					8
#define NVIC_IRQ_PRI9					9
#define NVIC_IRQ_PRI10					10
#define NVIC_IRQ_PRI11					11
#define NVIC_IRQ_PRI12					12
#define NVIC_IRQ_PRI13					13
#define NVIC_IRQ_PRI14					14
#define NVIC_IRQ_PRI15					15

/*
 * @NVIC_MCU_PR_BITS supports 16 programmable priority levels; 4 bits of interrupt priority are implemented.
 * As NVIC PRI reg provides 8 bit priority field, only 4 high order bits will be used,
 */
#define NVIC_MCU_PR_BITS				4


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
#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))

#define USART1_PCLK_DI()				(RCC->APB2ENR &= (0 << 4))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= (0 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= (0 << 18))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= (0 << 5))

/*
 * Clock Enable/Disable Macro for System Configuration
 */
#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= (0 << 14))


/*
 * RCC_CFGR register bitfields
 */
#define RCC_CFGR_SWS					2  // 2 bit long field. System Clock switch status; find clk source.
#define RCC_CFGR_HPRE					4  // 4 bit long field. AHB prescaler
#define RCC_CFGR_PPRE1					10 // 3 bit long field. APB1 low speed Prescaler
#define RCC_CFGR_PPRE2					13 // 3 bit long field. APB2 high speed Prescaler

/*
 * System Clock Source Macro
 */
#define SYSCLK_HSI						0 // High Speed Internal Clock
#define SYSCLK_HSE						1 // High speed External Clock
#define SYSCLK_PLL						2 // Phase locked loop


/*
 *  System clock and their speeds. Note PLL clock speed will have to be calculated
 */
#define SYSCLK_HSI_SPEED				16000000 // 16MHz
#define SYSCLK_HSE_SPEED				8000000  //8 MHz
#define SYSCLK_PLL_SPEED				0  // TODO:: create calculate PLL speed function

/*
 * SPI Control Register 1 Fields
 */
#define SPI_CR1_BIDIMODE				15	// Bidirectional Mode
#define SPI_CR1_BIDIOE					14  // Output enable in bidirectional mode
#define SPI_CR1_DFF						11  // Data Frame Format
#define SPI_CR1_RXONLY					10  // Receive Only
#define SPI_CR1_SSM						9	// Software Slave management
#define SPI_CR1_SSI						8	// Internal Slave select. Effect only when SSM is set
#define SPI_CR1_LSBFIRST				7	// Frame Format
#define SPI_CR1_SPE						6	// SPI enable
#define SPI_CR1_BR						5	// Baud Rate Control
#define SPI_CR1_MSTR					2	// Device Selection: (Master/Slave)
#define SPI_CR1_CPOL					1	// Clock Polarity
#define SPI_CR1_CPHA					0	// Clock Phase

/*
 * SPI Control Register 2 Fields
 */
#define SPI_CR2_TXEIE					7  // Tx Buffer empty interrupt Enable. 0: Masked, 1: Unmasked
#define SPI_CR2_RXNEIE					6  // Rx buffer NOT empty interrupt enable
#define SPI_CR2_ERRIE					5  // Error Interrupt Enable
#define SPI_CR2_FRF						4  // Frame Format. 0: SPI Motorola Mode, 1: SPI TI Mode
#define SPI_CR2_SSOE					2  // Slave Select output enable
#define SPI_CR2_TXDNAEN					1  // Tx buffer Direct Memory Access (DMA) enable
#define SPI_CR2_RXDMAEN					0  // Rx buffer DMA enable

/* @SPI_STATUS_FIELD
 * SPI Status Register: Status are set by by the MCU
 */
#define SPI_SR_FRE						8  // Frame Format error. 0: No error. 1: FF error occurred
#define SPI_SR_BSY						7  // 0: SPI not busy. 1: SPI is busy or Tx is not empty
#define SPI_SR_OVR						6  // Overrun
#define SPI_SR_MODF						5  // Mode fault
#define SPI_SR_CRCERR					4  // CRC Error flag
#define SPI_SR_UDR						3  // Underrun flag
#define SPI_SR_CHSIDE					2  // Channel Side
#define SPI_SR_TXE						1  // Tx buffer 0: not empty. 1: empty
#define SPI_SR_RXNE						0  // Rx Buffer 0: not empty. 1: empty

/*
 * @SPI_EVENT
 * SPI Event State for Transmission and Reception
 */
#define SPI_EVENT_RX_COMPLETE			0
#define SPI_EVENT_TX_COMPLETE			1
#define SPI_EVENT_ERROR_OVR				2



/*
 * Bit fields of I2C CR1 (Control Register)
 */
#define I2C_CR1_SWRST					15	// Software Reset. 0: Not under Reset state, 1: Under Reset state
#define I2C_CR1_ALERT					13  //SMBus alert
#define I2C_CR1_PEC						12
#define I2C_CR1_POS						11
#define I2C_CR1_ACK						10
#define I2C_CR1_STOP					9
#define I2C_CR1_START					8
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_ENGC					6
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENARP					4
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_SMBUS					1
#define I2C_CR1_PE						0  // Peripheral Enable

/*
 * I2C CR2 Bit Fields
 */
#define I2C_CR2_LAST					12 // DMA last transfer: Master receiver mode to permit generation of NACK on last received data
#define I2C_CR2_DMAEN					11 // DMA Request Enable: (TxE = 1 or RxNE = 1 will enable this bit)
#define I2C_CR2_ITBUFEN					10 // Buffer Interrupt Enable, caused be TxE or RxNE
#define I2C_CR2_ITEVTEN					9  // Event Interrupt Enable
#define I2C_CR2_ITERREN					8  // Error Interrupt Enable
#define I2C_CR2_FREQ					0  // Peripheral Clock Frequency, 5:0; 5 bits long

/*
 * I2C OAR1 Bit field
 */
#define I2C_OAR1_ADD					1	// Address Interface field 10* bit long

/*
 * I2C Clock Control Register bit field
 */
#define I2C_CCR_FS						15  // I2C Master mode selection
#define I2C_CCR_DUTY					14  // Fm mode Duty Cycle
#define I2C_CCR_CCR						0   // Control SCL in master mode

/*
 * I2C Status Register1 Bit fields
 */
#define I2C_SR1_SMBALERT				15
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_PECERR					12  // Packet Error Checking in reception
#define I2C_SR1_OVR						11  // Overrun/Underrun
#define I2C_SR1_AF						10  // Ack failure
#define I2C_SR1_ARLO					9  // Arbitration lost (Master Mode)
#define I2C_SR1_BERR					8  // Bus Error caused by misplaced start/stop condition
#define I2C_SR1_TxE						7  // Transmitter Data register empty
#define I2C_SR1_RxNE					6  // Receive Data Register empty
#define I2C_SR1_STOPF					4  // Stop detection (slave mode)
#define I2C_SR1_ADD10					3  // 10 bit header sent (Master Mode)
#define I2C_SR1_BTF						2  // Byte transfer finished
#define I2C_SR1_ADDR					1  // Address between master and slave match
#define I2C_SR1_SB						0  // Start Condition (Master Mode)

/*
 * I2C Status Register 2 Bit Fields
 */
#define I2C_SR2_PEC						15  // Packet Error Checking Register. 7 bits long
#define I2C_SR2_DUALF					7   // Dual Flag (slave mode)
#define I2C_SR2_SMBHOST					6   // SMBus Host header (slave mode)
#define I2C_SR2_DEFAULT					5   // SMBus device default address (slave mode)
#define I2C_SR2_GENCALL					4	// General Call address (Slave mode)
#define I2C_SR2_TRA						2   // Number of bytes transmitted/received
#define I2C_SR2_BUSY					1   // I2C bus is busy
#define I2C_SR2_MSL						0	// Device in Master/Slave mode



/*
 * USART Status Register Bit Fields
 */
#define USART_SR_CTS					9	// CTS flag. Not available for UART4 and USART 5
#define USART_SR_LBD					8	// Line break detection flag
#define USART_SR_TXE					7	// Transmit Data Register Empty
#define USART_SR_TC						6	// Transmission complete. Dependent that TXE is set
#define USART_SR_RXNE					5	// Read Data register is not empty
#define USART_SR_IDLE					4	// Idle line is detected
#define USART_SR_ORE					3	// Overrun Error
#define USART_SR_NF						2	// Noise Detected Flag
#define USART_SR_FE						1	// Framing Error
#define USART_SR_PE						0	// Parity Error


/*
 *USART CR1 Register Bit Fields
 */
#define USART_CR1_OVER8					15	// Oversampling mode. 0: By 16. 1: by 0
#define USART_CR1_UE					13  // USART Enable
#define USART_CR1_M						12  // Word Length. 0: 8 Data bits. 1: 9 Data Bits
#define USART_CR1_WAKE					11  // Wake up Method. 0: Idle Line, 1: Address Mark
#define USART_CR1_PCE					10  // Parity Control Enable. 0: Disabled, 1: Enabled
#define USART_CR1_PS					9	// Parity Selection 0: Enable, 1: Odd
#define USART_CR1_PEIE					8
#define USART_CR1_TXEIE					7
#define USART_CR1_TCIE					6
#define USART_CR1_RXNEIE				5
#define USART_CR1_IDLEIE				4
#define USART_CR1_TE					3  // Transmitter Enable
#define USART_CR1_RE					2  // Receive Enable
#define USART_CR1_RWU					1  // Receiver Wake up
#define USART_CR1_SBK					0  // Send Break

/*
 * USART BRR Register bit Fields
 */
#define USART_BRR_MANTISSA				4	// 12 bit long. Start at Bit field 4
#define USART_BRR_FRACTION				0	// 4 bit long. Start at Bit Field 0

/*
 * USART CR2 Register Bit fields
 */
#define USART_CR2_LINEN					14
#define USART_CR2_STOP					12

/*
 * USART CR# Register bit Fields
 */
#define USART_CR3_CTSE					9
#define USART_CR3_RTSE					8



/* @OTHER_MACROS
 * Other macros and Enumerations */
#define	DISABLE							(0)
#define ENABLE							(1)
#define RESET							(DISABLE)
#define SET								(ENABLE)
#define NOT_SET							(DISABLE)


#endif /* INC_STM32F407XX_H_ */
