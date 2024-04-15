/*
 * stm32f76xxx_rcc.h
 *
 *  Created on: Apr 14, 2024
 *      Author: HoJoon
 */

#ifndef INC_STM32F76XXX_RCC_H_
#define INC_STM32F76XXX_RCC_H_

#include "stm32f76xxx.h"

typedef struct
{
	uint32_t PLLState;
	uint32_t PLLSource;
	uint32_t PLLM;
	uint32_t PLLN;
	uint32_t PLLP;
	uint32_t PLLQ;
	uint32_t PLLR;

} RCC_PLLInitTypeDef;

typedef struct
{
	uint32_t OscillatorType;
	uint32_t HSEState;
	uint32_t LSEState;
	uint32_t HSIState;
	uint32_t HSICalibrationValue;
	uint32_t LSIState;
	RCC_PLLInitTypeDef PLL;
} RCC_OscInit_t;

typedef struct
{
	uint32_t ClockType;
	uint32_t SYSCLKSource;
	uint32_t AHBCLKDivider;
	uint32_t APB1CLKDivider;
	uint32_t APB2CLKDivider;
} RCC_CLKInit_t;

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT		((uint32_t)100U)
#endif

#define RCC_OSCILLATORTYPE_NONE				((uint32_t)0x0U)
#define RCC_OSCILLATORTYPE_HSE				((uint32_t)0x1U)
#define RCC_OSCILLATORTYPE_HSI				((uint32_t)0x2U)
#define RCC_OSCILLATORTYPE_LSE				((uint32_t)0x4U)
#define RCC_OSCILLATORTYPE_LSI				((uint32_t)0x8U)

#define RCC_HSE_OFF							((uint32_t)0x0U)
#define RCC_HSE_ON							RCC_CR_HSEON
#define RCC_HSE_BYPASS						((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))

#define RCC_PLL_NONE						((uint32_t)0x0U)
#define RCC_PLL_OFF							((uint32_t)0x1U)
#define RCC_PLL_ON							((uint32_t)0x2U)

#define RCC_PLLSOURCE_HSI					RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE					RCC_PLLCFGR_PLLSRC_HSE

#define RCC_CLOCKTYPE_SYSCLK				((uint32_t)0x1U)
#define RCC_CLOCKTYPE_HCLK					((uint32_t)0x2U)
#define RCC_CLOCKTYPE_PCLK1					((uint32_t)0x4U)
#define RCC_CLOCKTYPE_PCLK2					((uint32_t)0x8U)

#define RCC_SYSCLKSOURCE_HSI				RCC_CFGR_SW_HSI

#define RCC_PLLP_DIV2						((uint32_t)0x2U)
#define RCC_PLLP_DIV4						((uint32_t)0x4U)
#define RCC_PLLP_DIV6						((uint32_t)0x6U)
#define RCC_PLLP_DIV8						((uint32_t)0x8U)

#define RCC_SYSCLKSOURCE_HSI				RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE				RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLLCLK				RCC_CFGR_SW_PLL

#define RCC_SYSCLKSOURCE_STATUS_HSI			RCC_CFGR_SWS_HSI
#define RCC_SYSCLKSOURCE_STATUS_HSE			RCC_CFGR_SWS_HSE
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK		RCC_CFGR_SWS_PLL

#define RCC_SYSCLK_DIV1					RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2					RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4					RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8					RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16				RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64				RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128				RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256				RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512				RCC_CFGR_HPRE_DIV512

#define RCC_HCLK_DIV1					RCC_CFGR_PPRE1_DIV1
#define RCC_HCLK_DIV2					RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4					RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV8					RCC_CFGR_PPRE1_DIV8
#define RCC_HCLK_DIV16					RCC_CFGR_PPRE1_DIV16

#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21U)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31U)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39U)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3BU)
#define RCC_FLAG_PLLSAIRDY               ((uint8_t)0x3CU)

#define RCC_FLAG_LSERDY                  ((uint8_t)0x41U)

#define RCC_SYSCLK_CONFIG(__RCC_SYSCLKSOURCE__)	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__RCC_SYSCLKSOURCE__))
#define RCC_GET_SYSCLK_SOURCE()					(RCC->CFGR & RCC_CFGR_SWS)
#define RCC_GET_PLL_OSCSOURCE() 				((uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC))

#define RCC_FLAG_MASK  ((uint8_t)0x1F)
#define RCC_GET_FLAG(__FLAG__) (((((((__FLAG__) >> 5) == 1)? RCC->CR :((((__FLAG__) >> 5) == 2) ? RCC->BDCR :((((__FLAG__) >> 5) == 3)? RCC->CSR :RCC->CIR))) & ((uint32_t)1 << ((__FLAG__) & RCC_FLAG_MASK)))!= 0)? 1 : 0)

#define RCC_HSE_CONFIG(__STATE__)                       \
		do {                                        	\
			if ((__STATE__) == RCC_HSE_ON)            	\
			{                                         	\
				SET_BIT(RCC->CR, RCC_CR_HSEON);         \
			}                                         	\
			else if ((__STATE__) == RCC_HSE_OFF)      	\
			{                                         	\
				CLEAR_BIT(RCC->CR, RCC_CR_HSEON);       \
				CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);      \
			}                                         	\
			else if ((__STATE__) == RCC_HSE_BYPASS)   	\
			{                                         	\
				SET_BIT(RCC->CR, RCC_CR_HSEBYP);        \
				SET_BIT(RCC->CR, RCC_CR_HSEON);         \
			}                                         	\
			else                                      	\
			{                                         	\
				CLEAR_BIT(RCC->CR, RCC_CR_HSEON);       \
				CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);      \
			}                                         	\
		} while(0)

#define RCC_PLL_ENABLE()		SET_BIT(RCC->CR, RCC_CR_PLLON)
#define RCC_PLL_DISABLE()		CLEAR_BIT(RCC->CR, RCC_CR_PLLON)

#define HSE_TIMEOUT_VALUE			HSE_STARTUP_TIMEOUT
#define PLL_TIMEOUT_VALUE			((uint32_t)2)
#define CLOCKSWITCH_TIMEOUT_VALUE	((uint32_t)5000)

void RCC_Osc_config(RCC_OscInit_t* pRCCOsc);
void RCC_Clock_config(RCC_CLKInit_t* pRCCClk, uint32_t FLatency);
uint32_t RCC_Get_SysClock_freq(void);

#endif /* INC_STM32F76XXX_RCC_H_ */
