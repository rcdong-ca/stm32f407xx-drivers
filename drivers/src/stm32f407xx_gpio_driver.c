/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 22, 2023
 *      Author: richard
 */

#include "arm_cortexM4.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdio.h>


uint8_t GPIO_PortToNumber(GPIO_RegDef_t* GPIOx_ptr) {
	if (GPIOx_ptr == GPIOA) {
		return 0;
	}
	else if (GPIOx_ptr == GPIOB) {
		return 1;
	}
	else if (GPIOx_ptr == GPIOC) {
			return 2;
		}
	else if (GPIOx_ptr == GPIOD) {
			return 3;
		}
	else if (GPIOx_ptr == GPIOE) {
			return 4;
		}
	else if (GPIOx_ptr == GPIOF) {
			return 5;
		}
	else if (GPIOx_ptr == GPIOG) {
			return 6;
		}
	else if (GPIOx_ptr == GPIOH) {
			return 7;
		}
	else if (GPIOx_ptr == GPIOI) {
			return 8;
	}
	// TODO:: ADD LOGGING FOR INCORRECT GPIO PORT ADDRESS
	return 0;
}

/*
 * GPIO_Init() initializes the GPIO port according to the configurations found in the GPIO handler
 * which is set by the user.
 * Input:
 * 	GPIOx_Handle: User provided Handler that contains their GPIO configurations
 */
void GPIO_Init(GPIO_Handle_t* GPIOx_Handle) {

	// Enable Peripheral Clock
	GPIO_PCLKControl(GPIOx_Handle, ENABLE);

	GPIO_PinConfig_t* PinConfig_ptr = &(GPIOx_Handle->GPIOx_PinConfig);
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handle->GPIOx_ptr;
	uint32_t temp = 0;	// registers are of 32bit but values in PinConfig_t may be less

	// 1. configure mode of GPIO pin.
	if (PinConfig_ptr->PinMode <= GPIO_MODE_ANALOG) { // not Interrupt modes
		temp = PinConfig_ptr->PinMode << (PinConfig_ptr->PinNumber * 2);
//		// bitfield may already have values, clear it
		GPIOx_ptr->MODER &= ~(0x3 << (PinConfig_ptr->PinNumber * 2) );
		GPIOx_ptr->MODER |= temp;

	}
	else {
		// Interrupt Mode still require pins to be in Input mode
		temp = GPIO_MODE_IN << (PinConfig_ptr->PinNumber * 2);
		GPIOx_ptr->MODER &= ~(0x3 << (PinConfig_ptr->PinNumber * 2) );
		GPIOx_ptr->MODER |= temp;

		if (PinConfig_ptr->PinMode == GPIO_MODE_IN_FT) {
			// clear the RT so its only FT
			EXTI->RTSR &= (1 << PinConfig_ptr->PinNumber);
			EXTI->FTSR |= (1 << PinConfig_ptr->PinNumber);

		}
		else if (PinConfig_ptr->PinMode == GPIO_MODE_IN_RT) {
			EXTI->FTSR &= (1 << PinConfig_ptr->PinNumber);
			EXTI->RTSR |= (1 << PinConfig_ptr->PinNumber);

		}
		else if (PinConfig_ptr->PinMode == GPIO_MODE_IN_RFT) {
			EXTI->RTSR |= (1 << PinConfig_ptr->PinNumber);
			EXTI->FTSR |= (1 << PinConfig_ptr->PinNumber);
		}

		// configure the EXTI control on which pin the interrupt will be coming from
		uint8_t RegNum = PinConfig_ptr->PinNumber / 4;		// Each Reg accounts for 4 pins
		uint8_t GPIO_PortNum = GPIO_PortToNumber(GPIOx_ptr);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[RegNum] &= ~(GPIO_PortNum << (GPIO_PortNum % 4) );
		SYSCFG->EXTICR[RegNum] |= (GPIO_PortNum << (GPIO_PortNum % 4) );

		// Unmask the line so processor can see the interrupt
		EXTI->IMR |=  (1 << PinConfig_ptr->PinNumber);

	}
	// Configure the output speed
	temp = PinConfig_ptr->PinOutputSpeed << (PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->OSPEEDR &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->OSPEEDR |= temp;

	// configure the output type
	temp = PinConfig_ptr->PinOutputType << (PinConfig_ptr->PinNumber);
	GPIOx_ptr->OTYPER &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->OTYPER |= temp;

	// Configure the pupd settings
	temp = PinConfig_ptr->PinPUpPDo << (PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->PUPDR &= ~(0x3 << PinConfig_ptr->PinNumber * 2);
	GPIOx_ptr->PUPDR |= temp;

	// 5. configure the alt functionality, if set to alt mode
	if (PinConfig_ptr->PinMode == GPIO_MODE_ALTFUN) {
		uint32_t alt_idx = PinConfig_ptr->PinNumber / 8;
		temp = PinConfig_ptr->PinAltFunMode << ( (PinConfig_ptr->PinNumber % 8) * 4);
		GPIOx_ptr->AFR[alt_idx] &= ~(0xF << PinConfig_ptr->PinNumber * 2);
		GPIOx_ptr->AFR[alt_idx] |= temp;

	}
}

/*
 * GPIO De-Initialization: Reset the registers for a GPIO port by placing it in reset state and not reset state
 */
void GPIO_DeInit(GPIO_Handle_t* GPIOx_Handler) {

	GPIO_PCLKControl(GPIOx_Handler, DISABLE);

	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handler->GPIOx_ptr;
	if (GPIOx_ptr == GPIOA) {
		GPIOA_RESET();
	}
	else if (GPIOx_ptr == GPIOB) {
		GPIOB_RESET();
	}
	else if (GPIOx_ptr == GPIOC) {
		GPIOC_RESET();
	}
	else if (GPIOx_ptr == GPIOD) {
		GPIOD_RESET();
	}
	else if (GPIOx_ptr == GPIOE) {
		GPIOE_RESET();
	}
	else if (GPIOx_ptr == GPIOF) {
		GPIOF_RESET();
	}
	else if (GPIOx_ptr == GPIOG) {
		GPIOG_RESET();
	}
	else if (GPIOx_ptr == GPIOH) {
		GPIOH_RESET();
	}
	else if (GPIOx_ptr == GPIOI) {
		GPIOI_RESET();
	}
	else {
		// No Valid GPIO port
	}
}

/*
 * Read a single pin from Input data register
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t* GPIOx_Handler) {
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handler->GPIOx_ptr;
	uint8_t PinNumber = GPIOx_Handler->GPIOx_PinConfig.PinNumber;
	uint8_t data = (GPIOx_ptr->IDR >> PinNumber) & 0x1;
	return data;

}

/*
 * Read from GPIO Input data Register
 * Input:
 * 	GPIOx_Handler:
 * Return:
 * 	Data (uint16_t) data
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t* GPIOx_Handler) {
	return (uint16_t)GPIOx_Handler->GPIOx_ptr->IDR;
}

/*
 * Write to Output Pin
 */
void GPIO_WriteToOutputPin(GPIO_Handle_t* GPIOx_Handler, uint8_t Val) {
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handler->GPIOx_ptr;
	uint8_t PinNumber = GPIOx_Handler->GPIOx_PinConfig.PinNumber;
	// clear the the bits by setting it to 0
	GPIOx_ptr->ODR &= ~(0x1 << PinNumber);
	GPIOx_ptr->ODR |= (Val << PinNumber);
}

/*
 * Write to all Output Pins
 */
void GPIO_WriteToOutputPort(GPIO_Handle_t* GPIOx_Handler, uint16_t OutputData) {
	GPIOx_Handler->GPIOx_ptr->ODR = OutputData;
}

/*
 * Toggle Output pin. Switch from 1 -> 0 or 0 -> 1.
 */
void GPIO_ToggleOutputPin(GPIO_Handle_t* GPIOx_Handler) {
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handler->GPIOx_ptr;
	uint8_t PinNumber = GPIOx_Handler->GPIOx_PinConfig.PinNumber;
	GPIOx_ptr->ODR ^= (1 << PinNumber);
}
void GPIO_ToggleOutputPinDirect(GPIO_RegDef_t* GpioPort, uint8_t PinNumber) {
	GpioPort->ODR ^= (1 << PinNumber);
}
/*
 * Get Output Pin's current value
 * Input:
 * 	GPIOx_Handler: Corresponding Handler to GPIOx port
 * Output:
 * 	data: can be 1 or 0
 */
uint8_t GPIO_ReadFromOutputPin(GPIO_Handle_t* GPIOx_Handler) {
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handler->GPIOx_ptr;
	uint8_t PinNumber = GPIOx_Handler->GPIOx_PinConfig.PinNumber;
	uint8_t data = (GPIOx_ptr->ODR >> PinNumber) & 0x1;
	return data;
}


/*
 * GPIO's peripheral clock controller.
 * Input:
 * 	GPIOx_ptr: pointer address of GPIOx port.
 * 	En_Di: ENABLE or DISABLE clock
 */

void GPIO_PCLKControl(GPIO_Handle_t* GPIOx_Handler, uint8_t En_Di) {
	GPIO_RegDef_t* GPIOx_ptr = GPIOx_Handler->GPIOx_ptr;
	if (En_Di == ENABLE) {
		if (GPIOx_ptr == GPIOA)
			GPIOA_PCLK_EN();
		else if (GPIOx_ptr == GPIOB)
			GPIOB_PCLK_EN();
		else if (GPIOx_ptr == GPIOC)
			GPIOC_PCLK_EN();
		else if (GPIOx_ptr == GPIOD)
			GPIOD_PCLK_EN();
		else if (GPIOx_ptr == GPIOE)
			GPIOE_PCLK_EN();
		else if (GPIOx_ptr == GPIOF)
			GPIOF_PCLK_EN();
		else if (GPIOx_ptr == GPIOG)
			GPIOG_PCLK_EN();
		else if (GPIOx_ptr == GPIOH)
			GPIOH_PCLK_EN();
		else if (GPIOx_ptr == GPIOI)
			GPIOI_PCLK_EN();
		else {
			// Invalid GPIO Address!
		}
	}
	else if (En_Di == DISABLE) {
		if (GPIOx_ptr == GPIOA)
			GPIOA_PCLK_DI();
		else if (GPIOx_ptr == GPIOB)
			GPIOB_PCLK_DI();
		else if (GPIOx_ptr == GPIOC)
			GPIOC_PCLK_DI();
		else if (GPIOx_ptr == GPIOD)
			GPIOD_PCLK_DI();
		else if (GPIOx_ptr == GPIOE)
			GPIOE_PCLK_DI();
		else if (GPIOx_ptr == GPIOF)
			GPIOF_PCLK_DI();
		else if (GPIOx_ptr == GPIOG)
			GPIOG_PCLK_DI();
		else if (GPIOx_ptr == GPIOH)
			GPIOH_PCLK_DI();
		else if (GPIOx_ptr == GPIOI)
			GPIOI_PCLK_DI();
		else {
			// Invalid GPIO Address!
		}
	}
	else {
		// INVALID Insert Logging
		// TODO:: add Log for incorrect GPIO address
	}

}

/*
 * Configure the NVIC registers to accepts interrupt from EXTIx Line
 */

void GPIO_IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI) {
	if (EN_DI == ENABLE) {

		// MCU only supports up to 81 IRQ numbers, so we only need NVIC_ISER0 - NVIC_ISER2 registers
		if (IRQNumber < 32) {
			NVIC_Ctrl->ISER->REG[0] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 64) {
			NVIC_Ctrl->ISER->REG[1] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 96) {
			NVIC_Ctrl->ISER->REG[2] |= (1 << (IRQNumber % 32) );
		}
		else {
			// LOG:: INVALID IRQ NUMBER
		}
	}
	else {
		if (IRQNumber < 32) {
			NVIC_Ctrl->ICER->REG[0] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 64) {
			NVIC_Ctrl->ICER->REG[1] |= (1 << (IRQNumber % 32) );
		}
		else if (IRQNumber < 96) {
			NVIC_Ctrl->ICER->REG[2] |= (1 << (IRQNumber % 32) );
		}
		else {
			// LOG:: INVALID IRQ NUMBER
		}
	}
}

void GPIO_IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t IRQPriority) {

	// IPR has 4 8bit priority fields
	uint8_t TargetReg = IRQNumber / 4;
	uint8_t TargetField = IRQNumber % 4;

	// MCU support 16 levels; 4 bits of interrupt priority are used. This means only the 4 most significant bits
	// in the field will be used, as stated by the processor manual
	uint8_t ShiftAmount = ( TargetField * 8) + (NVIC_IPR_FIELD_SIZE - NVIC_MCU_PR_BITS) ;
	NVIC_Ctrl->IPR->REG[TargetReg] |= (IRQPriority << ShiftAmount);
}

void GPIO_IRQHandling(uint8_t PinNumber) {

	// clear the EXTI PR reg corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1<< PinNumber);
	}
}


