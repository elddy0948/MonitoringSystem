/*
 * stm32f76xxx.h
 *
 *  Created on: Apr 10, 2024
 *      Author: HoJoon
 */

#ifndef STM32F76XXX_H_
#define STM32F76XXX_H_

#include <stdint.h>

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
#define RCC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x3800U)
#define CRC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x3000U)

#define GPIOB_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x0400U)
#define GPIOC_BASE_ADDRESS		(AHB1_BASE_ADDRESS + 0x0800U)

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

#endif /* STM32F76XXX_H_ */
