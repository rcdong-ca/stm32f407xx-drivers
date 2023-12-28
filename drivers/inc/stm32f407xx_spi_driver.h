/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 6, 2023
 *      Author: richard
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
#include "arm_cortexM4.h"

/*
 * SPIx device configuration structure
 */
typedef struct {
	uint8_t DeviceMode;				// Master or Slave
	uint8_t BusConfig;				// Full Duplex, Half Duplex, single Duplex
	uint8_t DFF;			// Data frame format. Send x number of bits
	uint8_t CPHA;					// Clock Phase, 0: first clk edge, 1: second clk edge sampled
	uint8_t CPOL;			// Clock Polarity. 0: Low, 1: High
	uint8_t SSM;					// Slave Selection Management. Hardware or Software.
	uint8_t BR;				// Serial Clock speed. Baud rate
}SPI_Config_t;

typedef struct {
	SPI_Config_t SPIx_Config;
	SPI_RegDef_t* SPIx_ptr;
	// Full duplex support
	uint8_t* TxBuffer_ptr;
	uint32_t TxBufferLen;
	uint8_t TxState;

	uint8_t* RxBuffer_ptr;
	uint32_t RxBufferLen;
	uint8_t RxState;
}SPI_Handle_t;

/*
 * @SPI_DEVICE_MODE
 * Possible mode a device can be set to
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * @SPI_BUS_CONFIG
 */
#define SPI_BUS_FULL_DUPLEX				0
#define SPI_BUS_HALF_DUPLEX				1
#define SPI_BUS_SIMPLEX_RXONLY			2	// Simplex with M:Tx only is FUll DUplex but with MISO line deactivated


/*
 * @CPHA	Clock Phase
 */
#define	SPI_CPHA_LOW					0	// First (Rising/Falling) Edge Sampled
#define SPI_CPHA_HIGH					1	// Second Edge Sampled

/*
 * @CPOL	Clock Polarity
 */
#define SPI_CPOL_LOW					0	// Signal Starts Low
#define SPI_CPOL_HIGH					1	//Signal Starts High

/*
 * @SSM		Slave Select Manager
 */
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * @SPI_SPEED	Baud Rate of Serial Clock
 */
#define SPI_SCLK_BR_DIV2				0
#define SPI_SCLK_BR_DIV4				1
#define SPI_SCLK_BR_DIV8				2
#define SPI_SCLK_BR_DIV16			3
#define SPI_SCLK_BR_DIV32			4
#define SPI_SCLK_BR_DIV64			5
#define SPI_SCLK_BR_DIV128			6
#define SPI_SCLK_BR_DIV256			7

/*
 * @ SPI_DFF		Data frame format.
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * Reset SPIx Peripherals
 */
#define SPI1_RESET()				{RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); }
#define SPI2_RESET()				{RCC->APB1RSTR |= (1 << 14); RCC->APB2RSTR &= ~(1 << 14); }
#define SPI3_RESET()				{RCC->APB1RSTR |= (1 << 15); RCC->APB2RSTR &= ~(1 << 15); }


/*
 * @SPI_STATE SPI Application States
 */
#define SPI_STATE_READY					0
#define SPI_STATE_BUSY_RX				1	// In process of Receiving data
#define SPI_STATE_BUSY_TX				2   // In process of Sending Data


/***************************** SPI API Start **************************************/

/*
 * Initialize SPI peripherals with user configuration fron SPIx Handler
 */
void SPI_Init(SPI_Handle_t* SPIx_Handler);
void SPI_DeInit(SPI_Handle_t* SPIx_Handler);

/*
 * SPI Peripheral Control.
 */
void SPI_PControl(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di);

/*
 * Send and Receive Data
 */
void SPI_Send(SPI_Handle_t* SPIx_Handler, uint8_t* TxBuffer_ptr, uint32_t DataSize);
void SPI_Receive(SPI_Handle_t* SPIx_Handler, uint8_t* RxBuffer_ptr, uint32_t DataSize);

/*
 * Send and Receive Data with Interrupt Mode ( Non blocking )
 */
void SPI_Send_Int(SPI_Handle_t* SPIx_Handler, uint8_t* TxBuffer_ptr, uint32_t DataSize);
void SPI_Receive_Int(SPI_Handle_t* SPIx_Handler, uint8_t* RxBuffer_ptr, uint32_t DataSize);


/*
 * Get SPI status register info.
 * Return:
 * 	SET (1) or NOT_SET(0)
 */
uint8_t SPI_GetStatus(SPI_RegDef_t* SPIx_ptr, uint8_t StatusField);

/*
 * IRQ (Interrupt request) configuration and ISR (Interrupt Signal Response) handling
 */

void SPI_IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI);	// Interrupt requests configuration
void SPI_IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* SpiHandler_ptr);

/*
 * Peripheral Clock controller
 */

void SPI_PCLKControl(SPI_Handle_t* SPIx_Handler, uint8_t En_Di);  // Peripheral Clock controller

/*
 * SSI bit Configuration (Used when @SSM is set; Software NSS management)
 */
void SPI_SSIConfig(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di);

/*
 * SSOE (Slave select output enable) bit. Used
 */
void SPI_SSOIConfig(SPI_RegDef_t* SPIx_ptr, uint8_t En_Di);

/*
 * Overrun Flag handling
 */
void SPI_ClearOVRFlag(SPI_Handle_t* SpiHandler);
void SPI_CloseTransmission(SPI_Handle_t* SpiHandler);
void SPI_CloseReception(SPI_Handle_t* SpiHandler);

/*
 * User application Call back for interrupt event. Can be overwritten
 */
void SPI_ApplicationEventCallBack(SPI_Handle_t* SpiHandle, uint8_t AppEvent);

/***************************** SPI API End **************************************/


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
