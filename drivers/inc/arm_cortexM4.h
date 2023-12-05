/*
 * arm_cortexM4.h
 *
 *  Created on: Dec 3, 2023
 *      Author: richard
 *
 *      ARM Cortex M4 Processor Macros
 */

#ifndef INC_ARM_CORTEXM4_H_
#define INC_ARM_CORTEXM4_H_

#include <stdint.h>

#define __vo volatile


#define NVIC_ISER_BASEADDR				(0xE000E100U)
#define NVIC_ICER_BASEADDR				(0XE000E180U)
#define NVIC_ISPR_BASEADDR				(0XE000E200U)
#define NVIC_ICPR_BASEADDR				(0XE000E280U)
#define NVIC_IABR_BASEADDR				(0xE000E300U)
#define NVIC_IPR_BASEADDR				(0xE000E400U)
#define NVIC_STIR_BASEADDR				(0xE000EF00U)


/*
 * NVIC ISERx Registers	(Interrupt Set-enable) IRQNum: 1-240 interrupts
 */
typedef struct {
	__vo uint32_t REG[8];
}EightArr_Regs;

typedef struct {
	__vo uint32_t REG[60];
}IPR_Regs;


/*
 * structure definition of NVIC controller. Must be initiated by NVIC_Ctrl_Init
 */
typedef struct {
	EightArr_Regs* ISER;
	EightArr_Regs* ICER;
	EightArr_Regs* ISPR;
	EightArr_Regs* ICPR;
	IPR_Regs* IPR;
	uint32_t* STIR;
}NVIC_RegDef_t;

#define NVIC_IPR_FIELD_SIZE				8

void NVIC_CtrlInit(NVIC_RegDef_t* NVIC_Ctrl);
/* Interrupt Priority Register */

#endif /* INC_ARM_CORTEXM4_H_ */
