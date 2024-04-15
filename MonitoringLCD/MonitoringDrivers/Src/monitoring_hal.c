/*
 * monitoring_hal.c
 *
 *  Created on: Apr 14, 2024
 *      Author: HoJoon
 */

#include "monitoring_hal.h"

#include "FreeRTOS.h"

uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

volatile uint32_t uwTick;
uint32_t uwTickPrio = (1UL << NVIC_PRIO_BITS);
TickFreq_t uwTickFreq = TICK_FREQ_DEFAULT;

void Set_priority(int32_t IRQNumber, uint32_t PreemptPriority, uint32_t SubPriority)
{
	uint32_t priority_group = 0x00;
	priority_group = NVIC_get_priority_grouping();
	NVIC_set_priority(IRQNumber, NVIC_encode_priority(priority_group, PreemptPriority, SubPriority));
}

uint32_t SYSTICK_Config(uint32_t TicksNumb)
{
	return SysTick_config(TicksNumb);
}

__weak uint32_t Get_tick(void)
{
	return uwTick;
}

__weak void InitTick(uint32_t TickPriority)
{
	if(SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
	{
		return;
	}
	if(TickPriority < (1UL << NVIC_PRIO_BITS))
	{
		Set_priority(-1, TickPriority, 0U);
		uwTickPrio = TickPriority;
	}
	else
	{
		return;
	}
}

void Monitoring_Initialize(void)
{
	NVIC_set_priority_grouping(NVIC_PRIORITY_GROUP_4);
	InitTick(TICK_INT_PRIORITY);
	MSP_Initialize();
}

void MSP_Initialize(void)
{
		SYSCFG_PCLK_ENABLE();
}
