/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Dec 28, 2023
 *      Author: richard
 */

#include "stm32f407xx_i2c_driver.h"
/*
 * I2C Peripheral clock controller. Enables/Disables the peripheral Clock
 * Input:
 * 	I2Cx_Handler: the I2C handler provided by the user
 * 	En_Di: ENABLE or DISABLE, value found in @OTHER_MACROS
 */

uint8_t I2C_GetStatus1(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField) {
	return (I2Cx_ptr->SR1 >> StatusField) & 0x1;
}
uint8_t I2C_GetStatus2(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField) {
	return (I2Cx_ptr->SR2 >> StatusField) & 0x1;
}

void I2C_PCLKControl(I2C_Handle_t* I2Cx_Handler, uint8_t En_Di) {
	if (En_Di == ENABLE) {
		if (I2Cx_Handler->I2Cx_ptr == I2C1) {
			I2C1_PCLK_EN();
		}
		else if (I2Cx_Handler->I2Cx_ptr == I2C2) {
			I2C2_PCLK_EN();
		}
		else if (I2Cx_Handler->I2Cx_ptr == I2C3) {
			I2C3_PCLK_EN();
		}
	}
	else {
		if (I2Cx_Handler->I2Cx_ptr == I2C1) {
			I2C1_PCLK_DI();
		}
		else if (I2Cx_Handler->I2Cx_ptr == I2C2) {
			I2C2_PCLK_DI();
		}
		else if (I2Cx_Handler->I2Cx_ptr == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/*
 * Initialize I2C Peripheral. Only supports 7 bit address for the time being
 */
void I2C_Init(I2C_Handle_t* I2Cx_Handler) {

	// we want to reset the register
	uint32_t TempReg = 0;

	// Enable the Acking
	TempReg |= (1 << I2C_CR1_ACK);
	I2Cx_Handler->I2Cx_ptr->CR1 = TempReg;

	TempReg = 0;
	// Configure the peripheral clock frequency
	TempReg = (RCC_GetPCLK1Value() / 1000000);
	I2Cx_Handler->I2Cx_ptr->CR2 = (TempReg << I2C_CR2_FREQ);

	// add device own address
	TempReg = 0;
	TempReg |= (I2Cx_Handler->I2Cx_Config.DeviceAddr << I2C_OAR1_ADD);
	// reserved bit in reg that has to be software set to 1
	TempReg |= (1 << 14);
	I2Cx_Handler->I2Cx_ptr->OAR1 = TempReg;

	// Calculate CCR Value to set control SCL frequency
	TempReg = 0;
	uint16_t CCRValue = 0;
	if (I2Cx_Handler->I2Cx_Config.SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard Mode
		// Time(SCLK) = 2 * CCR * Time(Peripheral clock)
		CCRValue = RCC_GetPCLK1Value() / 2 /I2Cx_Handler->I2Cx_Config.SCLSpeed;
	}
	else {
		// Fast Mode
		TempReg |= (1 << I2C_CCR_FS);
		// Set Duty Mode
		TempReg |= (I2Cx_Handler->I2Cx_Config.FMDutyCycle << I2C_CCR_DUTY);

		if (I2Cx_Handler->I2Cx_Config.FMDutyCycle == I2C_FM_DUTY_2) {
			// CCR value for Duty = 0: Time(SCLK) = 3*CCR*Time(PCLK)
			CCRValue = RCC_GetPCLK1Value() / 3 / I2Cx_Handler->I2Cx_Config.SCLSpeed;
		}
		else if (I2Cx_Handler->I2Cx_Config.FMDutyCycle == I2C_FM_DUTY_16_9) {
			// CCR for Duty = 1: Time(SCLK) = 25 * CCR * Time(PCLK)
			CCRValue = RCC_GetPCLK1Value() / 25 / I2Cx_Handler->I2Cx_Config.SCLSpeed;
		}
	}
	TempReg |= ( (CCRValue & 0xFFF) << I2C_CCR_CCR);  // Keep only 12 bits of CCR value

	// Configure the TRISE register
}

void I2C_DeInit(I2C_Handle_t* I2Cx_Handler) {
	// Disable the Peripheral Clock
	I2C_PCLKControl(I2Cx_Handler, DISABLE);

	// Set I2C into reset state
	if (I2Cx_Handler->I2Cx_ptr == I2C1) {
		I2C1_RESET();
	}
	else if (I2Cx_Handler->I2Cx_ptr == I2C2) {
		I2C2_RESET();
	}
	else if (I2Cx_Handler->I2Cx_ptr == I2C3) {
		I2C3_RESET();
	}
}

/*
 * I2C Master Send (blocking all)
 */
void I2C_MasterSendData(I2C_Handle_t* I2Cx_Handler, uint8_t* TxBuffer_ptr, uint32_t TxLen, uint8_t SlaveAddr) {

	// Generate Start Condition
	I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_START);

	// wait for SR SB bit to be set to see if start generation is complete.. TODO:: Stuck waiting forever, add a timer?
	while ( I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_SB) == NOT_SET );

	// Address Phase: Write slave address to data register
	SlaveAddr  = SlaveAddr << 1;  // Shift left 1 bit for r/w bit
	SlaveAddr &= ~(0x1);  // r: 1. w: 0
	I2Cx_Handler->I2Cx_ptr->DR = SlaveAddr;

	// Wait for ACK/NACK response from slave to confirm address phase is complete
	while ( I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_ADDR) == NOT_SET );

	// Clear the Addr bit by reading SR1 and Sr2
	(void)I2Cx_Handler->I2Cx_ptr->SR1;
	(void)I2Cx_Handler->I2Cx_ptr->SR2;

	// Data Phase: Send Data
	while (TxLen > 0) {
		// wait for Data Register to be empty
		while (I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_TxE) == NOT_SET);
		I2Cx_Handler->I2Cx_ptr->DR = *TxBuffer_ptr;
		TxBuffer_ptr++;
		TxLen--;
	}

	// Wait for TxE=1 and BTF=1 (Program Stop). SCL is pulled low
	while (I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_TxE) == NOT_SET);
	while (I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_BTF) == NOT_SET);

	// Generate Stop Condition
	I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_STOP);
}



