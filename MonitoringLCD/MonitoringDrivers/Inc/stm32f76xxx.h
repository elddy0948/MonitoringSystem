/*
 * stm32f76xxx.h
 *
 *  Created on: Apr 10, 2024
 *      Author: HoJoon
 */

#ifndef STM32F76XXX_H_
#define STM32F76XXX_H_

#include <stdint.h>

#if !defined(HSE_VALUE)
#define HSE_VALUE		((uint32_t)8000000U)
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE		((uint32_t)16000000U)
#endif

/* Block 0 & Block 1  */
#define FLASH_AXIM_BASE_ADDRESS		0x08000000U
#define FLASH_ICTM_BASE_ADDRESS		0x00200000U
#define SYSTEM_MEMORY_BASE_ADDRESS	0x00100000U
#define SRAM1_BASE_ADDRESS			0x20020000U
#define SRAM2_BASE_ADDRESS			0x2007C000U
#define SRAM						SRAM1_BASE_ADDRESS

/* Peripherals */
#define PERIPHERAL_BASE_ADDRESS				0x40000000U
#define APB1_BASE_ADDRESS					PERIPHERAL_BASE_ADDRESS
#define APB2_BASE_ADDRESS					0x40010000U
#define AHB1_BASE_ADDRESS					0x40020000U
#define AHB2_BASE_ADDRESS					0x50000000U
#define AHB3_BASE_ADDRESS					0x60000000U
#define INTERNAL_PERIPHERAL_BASE_ADDRESS	0xE0000000

/* Peripherals register boundary addresses */
#define RCC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x3800UL)
#define CRC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x3000UL)
#define FLASH_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x3C00UL)

#define GPIOB_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x0400UL)
#define GPIOC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x0800UL)

#define SYSCFG_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x3800UL)
#define EXTI_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x3C00UL)

/* Register definition structures */
typedef struct
{
	volatile uint32_t MODER;		// 0x00
	volatile uint32_t OTYPER;		// 0x04
	volatile uint32_t OSPEEDR;		// 0x08
	volatile uint32_t PUPDR;		// 0x0C
	volatile uint32_t IDR;			// 0x10
	volatile uint32_t ODR;			// 0x14
	volatile uint32_t BSRR;			// 0x18
	volatile uint32_t LCKR;			// 0x1C
	volatile uint32_t AFR[2];		// 0x20 ~ 0x24
} GPIO_RegisterDefinition_t;

typedef struct
{
	volatile uint32_t CR;			// 0x00
	volatile uint32_t PLLCFGR;		// 0x04
	volatile uint32_t CFGR;			// 0x08
	volatile uint32_t CIR;			// 0x0C
	volatile uint32_t AHB1RSTR;		// 0x10
	volatile uint32_t AHB2RSTR;		// 0x14
	volatile uint32_t AHB3RSTR;		// 0x18
	volatile uint32_t Reserved1;	// 0x1C
	volatile uint32_t APB1RSTR;		// 0x20
	volatile uint32_t APB2RSTR;		// 0x24
	volatile uint32_t Reserved2[2];	// 0x28 ~ 0x2C
	volatile uint32_t AHB1ENR;		// 0x30
	volatile uint32_t AHB2ENR;		// 0x34
	volatile uint32_t AHB3ENR;		// 0x38
	volatile uint32_t Reserved3;	// 0x3C
	volatile uint32_t APB1ENR;		// 0x40
	volatile uint32_t APB2ENR;		// 0x44
	volatile uint32_t Reserved4[2];	// 0x48 ~ 0x4C
	volatile uint32_t AHB1LPENR;	// 0x50
	volatile uint32_t AHB2LPENR;	// 0x54
	volatile uint32_t AHB3LPENR;	// 0x58
	volatile uint32_t Reserved5;	// 0x5C
	volatile uint32_t APB1LPENR;	// 0x60
	volatile uint32_t APB2LPENR;	// 0x64
	volatile uint32_t Reserved6[2];	// 0x68 ~ 0x6C
	volatile uint32_t BDCR;			// 0x70
	volatile uint32_t CSR;			// 0x74
	volatile uint32_t Reserved7[2];	// 0x78 ~ 0x7C
	volatile uint32_t SSCGR;		// 0x80
	volatile uint32_t PLLI2SCFGR;	// 0x84
	volatile uint32_t PLLSAICFGR;	// 0x88
	volatile uint32_t DCKCFGR1;		// 0x8C
	volatile uint32_t DCKCFGR2;		// 0x90
} RCC_RegisterDefinition_t;

typedef struct
{
	volatile uint32_t MEMRMP;		// 0x00
	volatile uint32_t PMC;			// 0x04
	volatile uint32_t EXTICR[4];	// 0x08 ~ 0x14
	volatile uint32_t CBR;			// 0x1C
	volatile uint32_t CMPCR;		// 0x20
} SYSCFG_RegisterDefinition_t;

typedef struct
{
	volatile uint32_t IMR;			// 0x00
	volatile uint32_t EMR;			// 0x04
	volatile uint32_t RTSR;			// 0x08
	volatile uint32_t FTSR;			// 0x0C
	volatile uint32_t SWIER;		// 0x10
	volatile uint32_t PR;			// 0x14
} EXTI_RegisterDefinition_t;

typedef struct
{
	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t OPTCR;
	volatile uint32_t OPTCR1;
} FLASH_RegisterDefinition_t;

/* Peripheral specifics */
#define GPIOB		( (GPIO_RegisterDefinition_t*)GPIOB_BASE_ADDRESS )
#define GPIOC		( (GPIO_RegisterDefinition_t*)GPIOC_BASE_ADDRESS )

#define RCC			( (RCC_RegisterDefinition_t*)RCC_BASE_ADDRESS )
#define FLASH		( (FLASH_RegisterDefinition_t*)FLASH_BASE_ADDRESS )
#define SYSCFG		( (SYSCFG_RegisterDefinition_t*)SYSCFG_BASE_ADDRESS )
#define EXTI		( (EXTI_RegisterDefinition_t*)EXTI_BASE_ADDRESS )

/* CLK Enable macros*/
#define GPIOB_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 2))

#define SYSCFG_PCLK_ENABLE()	(RCC->APB2ENR |= (1 << 14))

/* CLK Disable macros */
#define GPIOB_PCLK_DISABLE()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DISABLE()	(RCC->AHB1ENR &= ~(1 << 2))

#define SYSCFG_PCLK_DISABLE()	(RCC->APB2ENR &= ~(1 << 14))

/* Reset Peripherals macros */
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)

#define SYSCFG_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 14); (RCC->APB2RSTR &= ~(1 << 14)); } while(0)

/* NVIC */
typedef struct
{
	volatile uint32_t ISER[7];
} NVIC_ISER_t;

#define NVIC_ISER_BASE_ADDRESS		(0xE000E100U)
#define NVIC_ISER					((NVIC_ISER_t*)NVIC_ISER_BASE_ADDRESS)

typedef struct
{
	volatile uint32_t ICER[7];
} NVIC_ICER_t;

#define NVIC_ICER_BASE_ADDRESS		(0xE000E180U)
#define NVIC_ICER					((NVIC_ICER_t*)NVIC_ICER_BASE_ADDRESS)

#define NVIC_IPR_BASE_ADDRESS		((volatile uint32_t*)0xE000E400U)
#define NO_IPR_BITS_IMPLEMENTED		4

#define NVIC_PRIO_BITS				4U

#define ENABLE						1
#define DISABLE						0

/* FLASH */
#define FLASH_ACR_LATENCY_Pos		(0U)
#define FLASH_ACR_LATENCY_Msk		(0xFUL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY			FLASH_ACR_LATENCY_Msk

/* RCC */
#define RCC_CFGR_SW_Pos				(0U)
#define RCC_CFGR_SW_Msk				(0x3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW					RCC_CFGR_SW_Msk

#define RCC_CFGR_SW_HSI				0x00000000U
#define RCC_CFGR_SW_HSE				0x00000001U
#define RCC_CFGR_SW_PLL				0x00000002U

#define RCC_CFGR_SWS_Pos			(2U)
#define RCC_CFGR_SWS_Msk			(0x3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS				RCC_CFGR_SWS_Msk
#define RCC_CFGR_SWS_0				(0x1UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_1				(0x2UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSI			0x0U
#define RCC_CFGR_SWS_HSE			0x4U
#define RCC_CFGR_SWS_PLL			0x8U

#define RCC_CFGR_HPRE_Pos			(4U)
#define RCC_CFGR_HPRE_Msk			(0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE				RCC_CFGR_HPRE_Msk
#define RCC_CFGR_HPRE_0				(0x1UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_1				(0x2UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_2				(0x4UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_3				(0x8UL << RCC_CFGR_HPRE_Pos)

#define RCC_CFGR_HPRE_DIV1			0x00000000U
#define RCC_CFGR_HPRE_DIV2			0x00000080U
#define RCC_CFGR_HPRE_DIV4			0x00000090U
#define RCC_CFGR_HPRE_DIV8			0x000000A0U
#define RCC_CFGR_HPRE_DIV16			0x000000B0U
#define RCC_CFGR_HPRE_DIV64			0x000000C0U
#define RCC_CFGR_HPRE_DIV128		0x000000D0U
#define RCC_CFGR_HPRE_DIV256		0x000000E0U
#define RCC_CFGR_HPRE_DIV512		0x000000F0U

#define RCC_CFGR_PPRE1_Pos			(10U)
#define RCC_CFGR_PPRE1_Msk			(0x7UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1				RCC_CFGR_PPRE1_Msk
#define RCC_CFGR_PPRE1_0			(0x1UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_1			(0x2UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_2			(0x4UL << RCC_CFGR_PPRE1_Pos)

#define RCC_CFGR_PPRE1_DIV1			0x00000000U
#define RCC_CFGR_PPRE1_DIV2			0x00001000U
#define RCC_CFGR_PPRE1_DIV4			0x00001400U
#define RCC_CFGR_PPRE1_DIV8			0x00001800U
#define RCC_CFGR_PPRE1_DIV16		0x00001C00U

#define RCC_CFGR_PPRE2_Pos			(13U)
#define RCC_CFGR_PPRE2_Msk			(0x7UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2				RCC_CFGR_PPRE2_Msk
#define RCC_CFGR_PPRE2_0			(0x1UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_1			(0x2UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_2			(0x4UL << RCC_CFGR_PPRE2_Pos)

#define RCC_CR_HSEON_POS			(16U)
#define RCC_CR_HSEON_MSK			(0x01UL << RCC_CR_HSEON_POS)
#define RCC_CR_HSEON				RCC_CR_HSEON_MSK

#define RCC_CR_HSEBYP_POS			(18U)
#define RCC_CR_HSEBYP_MSK			(0x01UL << RCC_CR_HSEBYP_POS)
#define RCC_CR_HSEBYP				RCC_CR_HSEBYP_MSK

#define RCC_PLLCFGR_PLLM_Pos		(0U)
#define RCC_PLLCFGR_PLLM_Msk		(0x3FUL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM			RCC_PLLCFGR_PLLM_Msk

#define RCC_PLLCFGR_PLLN_Pos		(6U)
#define RCC_PLLCFGR_PLLN_Msk		(0x1FFUL << RCC_PLLCFGR_PLLN_Pos)
#define RCC_PLLCFGR_PLLN			RCC_PLLCFGR_PLLN_Msk

#define RCC_PLLCFGR_PLLP_Pos		(16U)
#define RCC_PLLCFGR_PLLP_Msk		(0x3UL << RCC_PLLCFGR_PLLP_Pos)
#define RCC_PLLCFGR_PLLP			RCC_PLLCFGR_PLLP_Msk

#define RCC_PLLCFGR_PLLSRC_Pos		(22U)
#define RCC_PLLCFGR_PLLSRC_Msk		(0x1UL << RCC_PLLCFGR_PLLSRC_Pos)
#define RCC_PLLCFGR_PLLSRC			RCC_PLLCFGR_PLLSRC_Msk

#define RCC_PLLCFGR_PLLSRC_HSE_POS	(22U)
#define RCC_PLLCFGR_PLLSRC_HSE_MSK	(0x1UL << RCC_PLLCFGR_PLLSRC_HSE_POS)
#define RCC_PLLCFGR_PLLSRC_HSE		RCC_PLLCFGR_PLLSRC_HSE_MSK
#define RCC_PLLCFGR_PLLSRC_HSI		0x00000000U

#define RCC_PLLCFGR_PLLQ_Pos		(24U)
#define RCC_PLLCFGR_PLLQ_Msk		(0xFUL << RCC_PLLCFGR_PLLQ_Pos)
#define RCC_PLLCFGR_PLLQ			RCC_PLLCFGR_PLLQ_Msk

#define RCC_PLLCFGR_PLLR_Pos		(28U)
#define RCC_PLLCFGR_PLLR_Msk		(0x7UL << RCC_PLLCFGR_PLLR_Pos)
#define RCC_PLLCFGR_PLLR			RCC_PLLCFGR_PLLR_Msk

#define RCC_CR_PLLON_Pos			(24U)
#define RCC_CR_PLLON_Msk			(0x1UL << RCC_CR_PLLON_Pos)
#define RCC_CR_PLLON				RCC_CR_PLLON_Msk

#define SET_BIT(REG, BIT)					((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)					((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)					((REG) & (BIT))
#define CLEAR_REG(REG)						((REG) = (0x0))
#define WRITE_REG(REG, VAL)					((REG) = (VAL))
#define READ_REG(REG)						((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)	WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define RCC_PLL_CONFIG(__RCC_PLLSource__, __PLLM__, __PLLN__, __PLLP__, __PLLQ__, __PLLR__)  \
		(RCC->PLLCFGR = ((__RCC_PLLSource__) | (__PLLM__)                 | \
				((__PLLN__) << RCC_PLLCFGR_PLLN_Pos)                      | \
				((((__PLLP__) >> 1) -1) << RCC_PLLCFGR_PLLP_Pos)          | \
				((__PLLQ__) << RCC_PLLCFGR_PLLQ_Pos)                      | \
				((__PLLR__) << RCC_PLLCFGR_PLLR_Pos)))

#include "monitoring_hal.h"
#include "stm32f76xxx_gpio.h"
#include "stm32f76xxx_rcc.h"
#include "stm32f76xxx_flash.h"

#endif /* STM32F76XXX_H_ */
