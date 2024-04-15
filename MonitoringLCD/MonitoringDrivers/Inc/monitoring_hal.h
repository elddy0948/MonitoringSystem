/*
 * monitoring_hal.h
 *
 *  Created on: Apr 14, 2024
 *      Author: HoJoon
 */

#ifndef INC_MONITORING_HAL_H_
#define INC_MONITORING_HAL_H_

#include "cm7.h"
#include "stm32f76xxx.h"

#ifndef __weak
#define __weak		__attribute__((weak))
#endif

#define TICK_INT_PRIORITY		((uint32_t)15U)

#define NVIC_PRIORITY_GROUP_4		((uint32_t)0x3U)

extern uint32_t SystemCoreClock;
extern const uint8_t  AHBPrescTable[16];
extern const uint8_t  APBPrescTable[8];

typedef enum
{
	TICK_FREQ_10HZ = 100U,
	TICK_FREQ_100HZ = 10U,
	TICK_FREQ_1KHZ = 1U,
	TICK_FREQ_DEFAULT = TICK_FREQ_1KHZ
} TickFreq_t;

extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern TickFreq_t uwTickFreq;

void Set_priority(int32_t IRQNumber, uint32_t PreemptPriority, uint32_t SubPriority);
uint32_t SYSTICK_Config(uint32_t TicksNumb);

uint32_t Get_tick(void);
void InitTick(uint32_t TickPriority);

void Monitoring_Initialize(void);
void MSP_Initialize(void);

#endif /* INC_MONITORING_HAL_H_ */
