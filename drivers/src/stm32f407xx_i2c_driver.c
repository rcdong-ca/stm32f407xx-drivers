/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Dec 28, 2023
 *      Author: richard
 */



#include "stm32f407xx_i2c_driver.h"
#include <stddef.h>


/*
 * I2C Helper functions
 */

static uint8_t I2C_Get_Status1(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField) {
	return (I2Cx_ptr->SR1 >> StatusField) & 0x1;
}
static uint8_t I2C_Get_Status2(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField) {
	return (I2Cx_ptr->SR2 >> StatusField) & 0x1;
}

// Get the Interrupt flag bit
static uint8_t I2C_Get_IT_Flag(I2C_RegDef_t* I2Cx_ptr, uint8_t StatusField) {
	return (I2Cx_ptr->CR2 >> StatusField) & 0x1;
}

// Handle Ack
static void I2C_Ack_Control(I2C_RegDef_t* I2Cx_ptr, uint8_t En_Di) {
	if (En_Di == ENABLE) {
		I2Cx_ptr->CR1 |= (1 << I2C_CR1_ACK);
	}
	else if (En_Di == DISABLE) {
		I2Cx_ptr->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_Close_Rx(I2C_Handle_t *I2Cx_Handler)
{
	// Disable ITBUFEN Bit in CR2
	I2Cx_Handler->I2Cx_ptr->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Disable ITEVFEN Bit in CR2
	I2Cx_Handler->I2Cx_ptr->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	I2Cx_Handler->State = I2C_STATE_READY;
	I2Cx_Handler->RxBuffer = NULL;
	I2Cx_Handler->RxLen = 0;
	I2Cx_Handler->RxSize = 0;

	// Re-enable ACK if enabled
	if(I2Cx_Handler->I2Cx_Config.ACKControl == ENABLE)
	{
		I2C_Ack_Control(I2Cx_Handler->I2Cx_ptr, ENABLE);
	}

}

void I2C_Close_Tx(I2C_Handle_t *I2Cx_Handler)
{
	// Disable ITBUFEN in CR2
	I2Cx_Handler->I2Cx_ptr->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	// Implement the code to disable ITEVFEN Control Bit
	I2Cx_Handler->I2Cx_ptr->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
	I2Cx_Handler->State = I2C_STATE_READY;
	I2Cx_Handler->TxBuffer = NULL;
	I2Cx_Handler->TxLen = 0;
}

void I2C_ApplicationEventCallBack(I2C_Handle_t* I2Cx_Handler, uint8_t AppEvent) {

}

/*
 *
 * If Device in Master RxMode and RcSize == 1, we will have to disable the Ack
 * before clearing the Addr flag.
 * Otherwise, Clear the addr flag right away
 */
static void I2C_Clear_Addr_Flag(I2C_Handle_t* I2Cx_Handler) {

	// Master Rx Mode. with RxSize == 1 case
	if (I2C_GetStatus2(I2Cx_Handler->I2Cx_ptr, I2C_SR2_MSL) == I2C_DEVICE_MODE_MASTER) {
		if (I2Cx_Handler->RxSize == 1) {
			// Disable the Ack Bit
			I2C_Ack_Control(I2Cx_Handler->I2Cx_ptr, DISABLE);
		}
	}
	// Clear flag by reading SR1 and SR2 registers
	(void)I2Cx_Handler->I2Cx_ptr->SR1;
	(void)I2Cx_Handler->I2Cx_ptr->SR2;
}

static void I2C_Master_RXNE_ITEV_Handle(I2C_Handle_t* I2Cx_Handler) {

	if (I2C_Get_Status2(I2Cx_Handler->I2Cx_ptr, I2C_SR2_MSL) == I2C_DEVICE_MODE_MASTER) {
		if (I2Cx_Handler->RxLen > 0) {

			if (I2Cx_Handler->RxLen == 2) {
				// Disable Ack
				I2Cx_Handler->I2Cx_ptr->CR1 &= ~(1 << I2C_CR1_ACK);
			}
			// Read the Data into buffer
			*I2Cx_Handler->RxBuffer = (uint8_t)(I2Cx_Handler->I2Cx_ptr->DR);
			I2Cx_Handler->RxBuffer++;
			I2Cx_Handler->RxLen--;
		}
		if (I2Cx_Handler->RxLen == 0) {
			// Generate Stop Condition if repeated start is not set
			if (I2Cx_Handler->RS == NOT_SET)
				I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_STOP);

			// End the Rx operation
			I2C_Close_Rx(I2Cx_Handler);

			// Notify Application of Rx Completion
			I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ITEV_RX_CMPLT);

		}
	}
}

static void I2C_Master_TXE_ITEV_Handle(I2C_Handle_t* I2Cx_Handler) {

	if (I2Cx_Handler->State == I2C_STATE_TX_BUSY) {
		if (I2Cx_Handler->TxLen > 0) {
			// Place data from Tx Buffer into DR
			I2Cx_Handler->I2Cx_ptr->DR = *(I2Cx_Handler->TxBuffer);
			I2Cx_Handler->TxBuffer++;
			I2Cx_Handler->TxLen--;
		}
	}
}

/*
 * Method of sending and receiving is quite similar between the nonblocking and blocking send/receive.
 */
static void I2c_Address_Phase(I2C_Handle_t* I2Cx_Handler, uint8_t SlaveAddr) {
	SlaveAddr  = SlaveAddr << 1;  // Shift left 1 bit for r/w bit
	if (I2Cx_Handler->State == I2C_STATE_RX_BUSY) {
		SlaveAddr |= (0x1);	// read bit: Set to 1
		I2Cx_Handler->I2Cx_ptr->DR = SlaveAddr;
	}
	else if (I2Cx_Handler->State == I2C_STATE_TX_BUSY) {
		SlaveAddr &= ~(0x1);  // Write bit: Set to 0
		I2Cx_Handler->I2Cx_ptr->DR = SlaveAddr;
	}
}




/******************* END ****************************/

/*
 * I2C Peripheral clock controller. Enables/Disables the peripheral Clock
 * Input:
 * 	I2Cx_Handler: the I2C handler provided by the user
 * 	En_Di: ENABLE or DISABLE, value found in @OTHER_MACROS
 */

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
	// init I2C peripheral clock
	I2C_PCLKControl(I2Cx_Handler, ENABLE);

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
	// reserved bit in position 14 in reg that has to be software set to 1
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
	I2Cx_Handler->I2Cx_ptr->CCR = TempReg;

	// Configure the TRISE register
	if (I2Cx_Handler->I2Cx_Config.SCLSpeed <= I2C_SCL_SPEED_SM) {
		TempReg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else {
		TempReg = RCC_GetPCLK1Value() * 300 / 1000000000U + 1;
	}
	TempReg |= ( (CCRValue & 0x3F) );
	I2Cx_Handler->I2Cx_ptr->TRISE = TempReg;
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
	while ( I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_SB) == NOT_SET );

	// Address Phase: Write slave address to data register
	I2c_Address_Phase(I2Cx_Handler, SlaveAddr);

	// Wait for ACK/NACK response from slave to confirm address phase is complete
	while ( I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_ADDR) == NOT_SET );

	// Clear the Addr bit by reading SR1 and Sr2
	I2C_Clear_Addr_Flag(I2Cx_Handler);

	// Data Phase: Send Data
	while (TxLen > 0) {
		// wait for Data Register to be empty
		while (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_TxE) == NOT_SET);
		I2Cx_Handler->I2Cx_ptr->DR = *I2Cx_Handler->TxBuffer;
		I2Cx_Handler->TxBuffer++;
		I2Cx_Handler->TxLen--;
	}

	// Wait for TxE=1 and BTF=1 (Program Stop). SCL is pulled low
	while (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_TxE) == NOT_SET);
	while (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_BTF) == NOT_SET);

	// Generate Stop Condition
	I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_STOP);
}

/*
 * Master Receive Mode. Blocking function
 * Input:
 * 	I2Cx_Handler
 * 	RxBuffer: 8bit int array use to store the receiving data
 * 	Len: Length of the receiving data
 * 	SlaveAddr: Slave address of slave device
 */
void I2C_MasterReceiveData(I2C_Handle_t* I2Cx_Handler, uint8_t* RxBuffer, uint32_t Len, uint8_t SlaveAddr) {
	// Start Condition Generation. Confirm generation though SB bit in SR1
	I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_START);

	// Wait for Start Condition to be generated
	while ( I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_SB) == NOT_SET );

	// Start Address Phase
	I2c_Address_Phase(I2Cx_Handler, SlaveAddr);

	// check addr flag in SR1 for addr phase completion
	while (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_ADDR)== NOT_SET)


	// 1 Byte Data reception.
	if (Len == 1) {
		// Ack bit has to be disabled before Clearing the Addr flag
		I2Cx_Handler->I2Cx_ptr->CR1 &= ~(1 << I2C_CR1_ACK);

		// Clear ADDR flag
		I2C_Clear_Addr_Flag(I2Cx_Handler);

		// wait for RXNE bit to be set
		while (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_RxNE) != NOT_SET);
		// generate stop condition
		I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_STOP);
		// Read the data
		*RxBuffer = (uint8_t)(I2Cx_Handler->I2Cx_ptr->DR);
		RxBuffer++;
		Len--;
	}
	// 1 < x Byte Data Reception
	else if (Len > 1) {
		// Clear ADDR flag
		I2C_Clear_Addr_Flag(I2Cx_Handler);

		while (Len > 0) {
			// wait for RXNE bit to be set (DR has data)
			while (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_RxNE) != NOT_SET);
			// Second last Byte: Initiate Stop communication
			if (Len == 2) {
				// Disable Ack bit to send NACK
				I2C_Ack_Control(I2Cx_Handler->I2Cx_ptr, DISABLE);
				// Generate Stop Condition
				I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_STOP);
			}
			// Read data clears RxNE
			*RxBuffer = (uint8_t)(I2Cx_Handler->I2Cx_ptr->DR);
			RxBuffer++;
			Len--;
		}
	}
	// Set the Ack bit
	if (I2Cx_Handler->I2Cx_Config.ACKControl == ENABLE) {
		I2C_Ack_Control(I2Cx_Handler->I2Cx_ptr, ENABLE);
	}
}

/*********************** I2C Interrupt API's *****************************/

/*
 * I2C Master Send Interrupt (Non Blocking Call
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* I2Cx_Handler, uint8_t* TxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS) {
	uint8_t State = I2Cx_Handler->State;
	if (State != I2C_STATE_READY) {
		// Set the handler with the necessary information to send data
		I2Cx_Handler->TxBuffer = TxBuffer;
		I2Cx_Handler->RS = RS;
		I2Cx_Handler->State = I2C_STATE_TX_BUSY;
		I2Cx_Handler->TxLen = Len;
		I2Cx_Handler->DeviceAddr = SlaveAddr;

		// Generate Start Condition
		I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_START);

		// Set the interrupt bits in CR2
		I2Cx_Handler->I2Cx_ptr->CR2 |= (1 << I2C_CR2_ITBUFEN);
		I2Cx_Handler->I2Cx_ptr->CR2 |= (1 << I2C_CR2_ITEVTEN);
		I2Cx_Handler->I2Cx_ptr->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return State;
}
/*
 * i2C Master Receive Interrupt (Non-blocking Call)
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* I2Cx_Handler, uint8_t* RxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RS) {

	uint8_t State = I2Cx_Handler->State;
	if (State == I2C_STATE_READY) {
		// Config the Handler with the necessary information to receive data
		I2Cx_Handler->RxBuffer = RxBuffer;
		I2Cx_Handler->RxLen = Len;
		I2Cx_Handler->RxSize = Len;  // Used in ISR to manage data reception
		I2Cx_Handler->DeviceAddr = SlaveAddr;
		I2Cx_Handler->RS = RS;
		I2Cx_Handler->State = I2C_STATE_RX_BUSY;

		// Generate start condition
		I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_START);

		// set the interrupt bits in CR2
		I2Cx_Handler->I2Cx_ptr->CR2 |= (1 << I2C_CR2_ITBUFEN);
		I2Cx_Handler->I2Cx_ptr->CR2 |= (1 << I2C_CR2_ITEVTEN);
		I2Cx_Handler->I2Cx_ptr->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return State;
}

/*
 * I2C Slave Operations: Send Data and Receive Data
 */
void I2C_SlaveSendData(I2C_RegDef_t* I2Cx_ptr, uint8_t data) {
	I2Cx_ptr->DR =data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* I2Cx_ptr) {
	return (uint8_t)I2Cx_ptr->DR;
}

/*
 * I2C Event Interrupt Handling. Will only get triggered if the ITEVTEN bit is set
 */
 void I2C_EV_IRQHandling(I2C_Handle_t* I2Cx_Handler) {

	 // SB flag event
	 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_SB) == SET) {
		 // Initiate Address Phase
		 I2c_Address_Phase(I2Cx_Handler, I2Cx_Handler->DeviceAddr);
	 }
	 // ADDR flag event. Acknowledges Address Phase Completion
	 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_ADDR) == SET) {
		 I2C_Clear_Addr_Flag(I2Cx_Handler);
	 }

	 // Buffer event has to be enabled
	 if (I2C_Get_IT_Flag(I2Cx_Handler->I2Cx_ptr, I2C_CR2_ITBUFEN) == SET) {

		 // RxNE Flag: Data has arrived in DR
		 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_RxNE) == SET) {

			 // Master Response to RXNE
			 if (I2C_GetStatus2(I2Cx_Handler->I2Cx_ptr, I2C_SR2_MSL) == I2C_DEVICE_MODE_MASTER) {
				 I2C_Master_RXNE_ITEV_Handle(I2Cx_Handler);
			 }
			 // Slave response to RXNE
			 else if (I2C_Get_Status2(I2Cx_Handler->I2Cx_ptr, I2C_SR2_MSL) == I2C_DEVICE_MODE_SLAVE) {

				 // check if slave is busy in Txe state
				 if (I2Cx_Handler->State == I2C_STATE_TX_BUSY) {
					 // Notify Application of RxNE flag set. Slave send Data
					 I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ITEV_SLAVE_RXNE);
				 }
			 }
		 }
		 // TxE flag event
		 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_TxE) == SET) {
			 // Master and Slave will have different responses to flag
			 uint8_t DevMode = I2C_GetStatus2(I2Cx_Handler->I2Cx_ptr, I2C_SR2_MSL);
			 // Master Txe Flag response:
			 if (DevMode == I2C_DEVICE_MODE_MASTER) {
				 I2C_Master_TXE_ITEV_Handle(I2Cx_Handler);
			 }
			 else if (DevMode == I2C_DEVICE_MODE_SLAVE) {
				 // Check if slave is busy in Rx state
				 if (I2Cx_Handler->State == I2C_STATE_RX_BUSY) {
					 // Notify Application TxE event. Slave transmit Data
					 I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ITEV_SLAVE_TXE);
				 }
			 }
		 }
	 }
	 // STOPF Flag event.
	 // This can only be triggered by slave when it receive the Stop Condition from Master
	 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_STOPF) == SET) {
		 // Clear the Stop flag by reading Sr1 writing to Cr1
		 (void)I2Cx_Handler->I2Cx_ptr->SR1;
		 I2Cx_Handler->I2Cx_ptr->CR1 |= (0x00000000);

		 // Notify application that of STOP event
		 I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ITEV_STOP_CMPLT);
	 }
	 // BTF Flag event.
	 // Just used by Master to close transmission if certain conditions are met
	 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_BTF) == SET) {

		 // Transmission: we only care about using this as Stop Generation.
		 if (I2Cx_Handler->State == I2C_STATE_TX_BUSY) {
			 // BTF = Txe = 1 and TxLen = 0 means no more bytes need to be transmitted,

			 if (I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_TxE) && I2Cx_Handler->TxLen == 0) {
				 if (I2Cx_Handler->RS != DISABLE) {
					 // Generate Stop Condition only if repeated start is disabled
					 I2Cx_Handler->I2Cx_ptr->CR1 |= (1 << I2C_CR1_STOP);
				 }
				 // Reset all the handle structure elements.
					I2C_Close_Tx(I2Cx_Handler);
					I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ITEV_TX_CMPLT);
			 }
		 }
	 }
 }



 void I2C_ERR_IRQHandling(I2C_Handle_t* I2Cx_Handler) {

	 // Buss Error
	 if (I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_BERR) == SET) {

		 // Data line is not released. Up to user to what should happen
		 // Clear BERR flag
		 I2Cx_Handler->I2Cx_ptr->SR1 &= ~(1 << I2C_SR1_BERR);

		// Notify User to handle Error
		I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ERR_BERR);
	 }
	 // Acknowledge Failure.
	 if (I2C_GetStatus1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_AF) == SET) {
		 // Clear AF flag
		 I2Cx_Handler->I2Cx_ptr->SR1 &= ~(1 << I2C_SR1_AF);
		 // Notify User. User should decide to generate Stop or Repeated Start
		 // This is also received by slave do end I2C communication
		 I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ERR_AF);
	 }

	 // Arbitration Lost
	 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_ARLO) == SET) {
		 // Clear the ARLO flag
		 I2Cx_Handler->I2Cx_ptr->SR1 &= ~(1 << I2C_SR1_ARLO);
		 // Notify the user
		 I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ERR_ARLO);
	 }

	 // Overrun/Underrun Error
	 if (I2C_Get_Status1(I2Cx_Handler->I2Cx_ptr, I2C_SR1_OVR)) {
		 // Clear the the flag
		 I2Cx_Handler->I2Cx_ptr->SR1 &= ~(1<<I2C_SR1_OVR);
		 // Notify the User
		 I2C_ApplicationEventCallBack(I2Cx_Handler, I2C_ERR_OVR);
	 }
 }



