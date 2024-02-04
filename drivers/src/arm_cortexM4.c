/*
 * arm_cortexM4.c
 *
 *  Created on: Dec 3, 2023
 *      Author: richard
 */


#include "arm_cortexM4.h"

void NVIC_CtrlInit(NVIC_RegDef_t* NVIC_Ctrl) {
	NVIC_Ctrl->ISER = (EightArr_Regs*)NVIC_ISER_BASEADDR;
	NVIC_Ctrl->ICER = (EightArr_Regs*)NVIC_ICER_BASEADDR;
	NVIC_Ctrl->ISPR = (EightArr_Regs*)NVIC_ISPR_BASEADDR;
	NVIC_Ctrl->ICPR = (EightArr_Regs*)NVIC_ICPR_BASEADDR;
	NVIC_Ctrl->IPR = (IPR_Regs*)NVIC_IPR_BASEADDR;
	NVIC_Ctrl->STIR = (uint32_t*)NVIC_STIR_BASEADDR;
}

void IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI) {	// Interrupt requests configuration
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
		}
}
void IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint32_t IRQPriority, uint8_t NVIC_PrBits) {
	// IPR has 4 8bit priority field
	uint8_t TargetReg = IRQNumber / 4;
	uint8_t TargetField = IRQNumber % 4;

	// MCU support 16 levels; 4 bits of interrupt priority are used. This means only the 4 most significant bits
	// in the field will be used, as stated by the processor manual
	uint8_t ShiftAmount = ( TargetField * 8) + (NVIC_IPR_FIELD_SIZE - NVIC_PrBits) ;
	NVIC_Ctrl->IPR->REG[TargetReg] |= (IRQPriority << ShiftAmount);
}
