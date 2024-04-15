/*
 * cm7.h
 *
 *  Created on: Apr 14, 2024
 *      Author: HoJoon
 */

#ifndef INC_CM7_H_
#define INC_CM7_H_

#include <stdint.h>

typedef struct
{
	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;
} SysTick_t;

typedef struct
{
	volatile uint32_t ISER[8];
	uint32_t RESERVED0[24];
	volatile uint32_t ICER[8];
	uint32_t RESERVED1[24];
	volatile uint32_t ISPR[8];
	uint32_t RESERVED2[24];
	volatile uint32_t ICPR[8];
	uint32_t RESERVED3[24];
	volatile uint32_t IABR[8];
	uint32_t RESERVED4[56];
	volatile uint8_t IP[240];
	uint32_t RESERVED5[644];
	volatile uint32_t STIR;
} NVIC_t;

typedef struct
{
	volatile  uint32_t CPUID;
	volatile uint32_t ICSR;
	volatile uint32_t VTOR;
	volatile uint32_t AIRCR;
	volatile uint32_t SCR;
	volatile uint32_t CCR;
	volatile uint8_t  SHPR[12U];
	volatile uint32_t SHCSR;
	volatile uint32_t CFSR;
	volatile uint32_t HFSR;
	volatile uint32_t DFSR;
	volatile uint32_t MMFAR;
	volatile uint32_t BFAR;
	volatile uint32_t AFSR;
	volatile  uint32_t ID_PFR[2U];
	volatile  uint32_t ID_DFR;
	volatile  uint32_t ID_AFR;
	volatile  uint32_t ID_MFR[4U];
	volatile  uint32_t ID_ISAR[5U];
	uint32_t RESERVED0[1U];
	volatile  uint32_t CLIDR;
	volatile  uint32_t CTR;
	volatile  uint32_t CCSIDR;
	volatile uint32_t CSSELR;
	volatile uint32_t CPACR;
	uint32_t RESERVED3[93U];
	volatile  uint32_t STIR;
	uint32_t RESERVED4[15U];
	volatile  uint32_t MVFR0;
	volatile  uint32_t MVFR1;
	volatile  uint32_t MVFR2;
	uint32_t RESERVED5[1U];
	volatile  uint32_t ICIALLU;
	uint32_t RESERVED6[1U];
	volatile  uint32_t ICIMVAU;
	volatile  uint32_t DCIMVAC;
	volatile  uint32_t DCISW;
	volatile  uint32_t DCCMVAU;
	volatile  uint32_t DCCMVAC;
	volatile  uint32_t DCCSW;
	volatile  uint32_t DCCIMVAC;
	volatile  uint32_t DCCISW;
	uint32_t RESERVED7[6U];
	volatile uint32_t ITCMCR;
	volatile uint32_t DTCMCR;
	volatile uint32_t AHBPCR;
	volatile uint32_t CACR;
	volatile uint32_t AHBSCR;
	uint32_t RESERVED8[1U];
	volatile uint32_t ABFSR;
} SCB_t;

/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos         16U                                            /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos          2U                                            /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk         (1UL << SysTick_CTRL_CLKSOURCE_Pos)            /*!< SysTick CTRL: CLKSOURCE Mask */

#define SysTick_CTRL_TICKINT_Pos            1U                                            /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk           (1UL << SysTick_CTRL_TICKINT_Pos)              /*!< SysTick CTRL: TICKINT Mask */

#define SysTick_CTRL_ENABLE_Pos             0U                                            /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1UL /*<< SysTick_CTRL_ENABLE_Pos*/)           /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0U                                            /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/)    /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos             0U                                            /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/)    /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31U
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)

#define SysTick_CALIB_SKEW_Pos             30U
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)

#define SysTick_CALIB_TENMS_Pos             0U
#define SysTick_CALIB_TENMS_Msk            	(0xFFFFFFUL)

#define SCB_AIRCR_PRIOGROUP_Pos				8U
#define SCB_AIRCR_PRIOGROUP_Msk				(7UL << SCB_AIRCR_PRIOGROUP_Pos)

#define SCB_AIRCR_VECTKEY_Pos				16U
#define SCB_AIRCR_VECTKEY_Msk				(0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)

#define SCS_BASE_ADDRESS            		(0xE000E000UL)
#define SYSTICK_BASE_ADDRESS				(SCS_BASE_ADDRESS + 0x0010UL)
#define NVIC_BASE_ADDRESS					(SCS_BASE_ADDRESS + 0x0100UL)
#define SCB_BASE_ADDRESS					(SCS_BASE_ADDRESS + 0x0D00UL)

#define SYSTICK							((SysTick_t*)SYSTICK_BASE_ADDRESS)
#define NVIC							((NVIC_t*)NVIC_BASE_ADDRESS)
#define SCB								((SCB_t*)SCB_BASE_ADDRESS)

static inline uint32_t SysTick_config(uint32_t ticks)
{
	if((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
	{
		return (1UL);
	}

	SYSTICK->LOAD = (uint32_t)(ticks - 1UL);
	// TODO: NVIC SET PRIORITY
	SYSTICK->VAL = 0UL;
	SYSTICK->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

	return (0UL);
}

static inline void NVIC_set_priority_grouping(uint32_t PriorityGroup)
{
	uint32_t reg_value;
	uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);

	reg_value = SCB->AIRCR;
	reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIOGROUP_Msk));
	reg_value = (reg_value | ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (PriorityGroupTmp << SCB_AIRCR_PRIOGROUP_Pos));
	SCB->AIRCR =  reg_value;
}

static inline uint32_t NVIC_get_priority_grouping(void)
{
	return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIOGROUP_Msk) >> SCB_AIRCR_PRIOGROUP_Pos));
}

static inline void NVIC_set_priority(int32_t IRQNumber, uint32_t priority)
{
	if(IRQNumber >= 0)
	{
		NVIC->IP[((uint32_t)IRQNumber)] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
	}
	else
	{
		SCB->SHPR[(((uint32_t)IRQNumber) & 0xFUL) - 4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
	}
}

static inline uint32_t NVIC_encode_priority(uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
	uint32_t PriorityGroupTemp = (PriorityGroup & (uint32_t)0x07UL);
	uint32_t PreemptPriorityBits;
	uint32_t SubPriorityBits;

	PreemptPriorityBits = ((7UL - PriorityGroupTemp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTemp);
	SubPriorityBits = ((PriorityGroupTemp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTemp - 7UL) + (uint32_t)(4U));
	return (
			((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
			((SubPriority & (uint32_t)((1UL << (SubPriorityBits)) - 1UL))));
}

#endif /* INC_CM7_H_ */
