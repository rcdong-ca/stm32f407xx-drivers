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
