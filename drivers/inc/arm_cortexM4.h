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

#define NVIC_IPR_FIELD_SIZE				8	// IPR register is composed of 4 fields, 8 bits long

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
	EightArr_Regs* ISER;	// Interrupt Set Enable; Enables the interrupt
	EightArr_Regs* ICER;	// Interrupt Clear Enable; Disable the interuppt
	EightArr_Regs* ISPR;	// Interrupt set Pending; Set interrupt to pending state
	EightArr_Regs* ICPR;	// Intterupt Clear Pending;	Remove Pending state from interrupt
	IPR_Regs* IPR;			// Interrupt Priority Register
	uint32_t* STIR;
}NVIC_RegDef_t;

void IRQInterruptConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint8_t EN_DI);
/*
 * NVIC_PrBits is MCU specific. Refer to MCU on how many priority will be supported.
 * STM32f4: refer to @NVIC_MCU_PR_BITS in stm32f407xx.h
 */
void IRQPriorityConfig(NVIC_RegDef_t* NVIC_Ctrl, uint8_t IRQNumber, uint32_t IRQPriority, uint8_t NVIC_PrBits);
void NVIC_CtrlInit(NVIC_RegDef_t* NVIC_Ctrl);


#define	DISABLE							(0)
#define ENABLE							(1)

#endif /* INC_ARM_CORTEXM4_H_ */
