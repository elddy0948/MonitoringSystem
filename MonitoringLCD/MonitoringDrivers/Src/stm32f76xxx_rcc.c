/*
 * stm32f76xxx_rcc.c
 *
 *  Created on: Apr 14, 2024
 *      Author: HoJoon
 */

#include "stm32f76xxx_rcc.h"

void RCC_Osc_config(RCC_OscInit_t* pRCCOsc)
{
	uint32_t tickstart;
	uint32_t pll_config;

	/* Check Null pointer */
	if (pRCCOsc == 0)
	{
		return;
	}

	/*------------------------------- HSE Configuration ------------------------*/
	if (((pRCCOsc->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{
		/* When the HSE is used as system clock or clock source for PLL, It can not be disabled */
		if ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE) || ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)))
		{
			if ((RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0) && (pRCCOsc->HSEState == RCC_HSE_OFF))
			{
				return;
			}
		}
		else
		{
			RCC_HSE_CONFIG(pRCCOsc->HSEState);
			if (pRCCOsc->HSEState != RCC_HSE_OFF)
			{
				tickstart = Get_tick();
				while (RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0)
				{
					if ((Get_tick() - tickstart) > HSE_TIMEOUT_VALUE)
					{
						return;
					}
				}
			}
			else
			{
				/* Get Start Tick*/
				tickstart = Get_tick();

				/* Wait till HSE is bypassed or disabled */
				while (RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0)
				{
					if ((Get_tick() - tickstart) > HSE_TIMEOUT_VALUE)
					{
						return;
					}
				}
			}
		}
	}
	/*-------------------------------- PLL Configuration -----------------------*/
	if ((pRCCOsc->PLL.PLLState) != RCC_PLL_NONE)
	{
		/* Check if the PLL is used as system clock or not */
		if (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
		{
			if ((pRCCOsc->PLL.PLLState) == RCC_PLL_ON)
			{
				RCC_PLL_DISABLE();
				tickstart = Get_tick();
				while (RCC_GET_FLAG(RCC_FLAG_PLLRDY) != 0)
				{
					if ((Get_tick() - tickstart) > PLL_TIMEOUT_VALUE)
					{
						return;
					}
				}

				/* Configure the main PLL clock source, multiplication and division factors. */
#if defined (RCC_PLLCFGR_PLLR)
				RCC_PLL_CONFIG(pRCCOsc->PLL.PLLSource,
						pRCCOsc->PLL.PLLM,
						pRCCOsc->PLL.PLLN,
						pRCCOsc->PLL.PLLP,
						pRCCOsc->PLL.PLLQ,
						pRCCOsc->PLL.PLLR);
#else
				RCC_PLL_CONFIG(pRCCOsc->PLL.PLLSource,
						pRCCOsc->PLL.PLLM,
						pRCCOsc->PLL.PLLN,
						pRCCOsc->PLL.PLLP,
						pRCCOsc->PLL.PLLQ);
#endif
				RCC_PLL_ENABLE();

				/* Get Start Tick*/
				tickstart = Get_tick();

				/* Wait till PLL is ready */
				while (RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 0)
				{
					if ((Get_tick() - tickstart) > PLL_TIMEOUT_VALUE)
					{
						return;
					}
				}
			}
			else
			{
				/* Disable the main PLL. */
				RCC_PLL_DISABLE();

				/* Get Start Tick*/
				tickstart = Get_tick();

				/* Wait till PLL is ready */
				while (RCC_GET_FLAG(RCC_FLAG_PLLRDY) != 0)
				{
					if ((Get_tick() - tickstart) > PLL_TIMEOUT_VALUE)
					{
						return;
					}
				}
			}
		}
		else
		{
			/* Do not return HAL_ERROR if request repeats the current configuration */
			pll_config = RCC->PLLCFGR;
#if defined (RCC_PLLCFGR_PLLR)
			if (((pRCCOsc->PLL.PLLState) == RCC_PLL_OFF) ||
					(READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != pRCCOsc->PLL.PLLSource) ||
					(READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != pRCCOsc->PLL.PLLM) ||
					(READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (pRCCOsc->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)) ||
					(READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != ((((pRCCOsc->PLL.PLLP) >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)) ||
					(READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (pRCCOsc->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)) ||
					(READ_BIT(pll_config, RCC_PLLCFGR_PLLR) != (pRCCOsc->PLL.PLLR << RCC_PLLCFGR_PLLR_Pos)))
#else
				if (((pRCCOsc->PLL.PLLState) == RCC_PLL_OFF) ||
						(READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != pRCCOsc->PLL.PLLSource) ||
						(READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != pRCCOsc->PLL.PLLM) ||
						(READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (pRCCOsc->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)) ||
						(READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != ((((pRCCOsc->PLL.PLLP) >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)) ||
						(READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (pRCCOsc->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)))
#endif
				{
					return;
				}
		}
	}
	return;
}

void RCC_Clock_config(RCC_CLKInit_t* pRCCClk, uint32_t FLatency)
{
	uint32_t tickstart = 0;

	if(pRCCClk == 0) { return; }

	if(FLatency > FLASH_GET_LATENCY())
	{
		FLASH_SET_LATENCY(FLatency);

		if(FLASH_GET_LATENCY() != FLatency) { return; }
	}

	if (((pRCCClk->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
	{
		if (((pRCCClk->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
		{
			MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
		}

		if (((pRCCClk->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
		{
			MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
		}

		MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, pRCCClk->AHBCLKDivider);
	}

	if (((pRCCClk->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
	{
		if (pRCCClk->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
		{
			if (RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0)
			{
				return;
			}
		}
		else if (pRCCClk->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
		{
			if (RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 0)
			{
				return;
			}
		}
		else
		{
			if (RCC_GET_FLAG(RCC_FLAG_HSIRDY) == 0)
			{
				return;
			}
		}

		RCC_SYSCLK_CONFIG(pRCCClk->SYSCLKSource);

		tickstart = Get_tick();

		while (RCC_GET_SYSCLK_SOURCE() != (pRCCClk->SYSCLKSource << RCC_CFGR_SWS_Pos))
		{
			if ((Get_tick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
			{
				return;
			}
		}
	}

	if (FLatency < FLASH_GET_LATENCY())
	{
		FLASH_SET_LATENCY(FLatency);
		if (FLASH_GET_LATENCY() != FLatency)
		{
			return;
		}
	}

	if (((pRCCClk->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
	{
		MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, pRCCClk->APB1CLKDivider);
	}

	if (((pRCCClk->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
	{
		MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((pRCCClk->APB2CLKDivider) << 3));
	}

	SystemCoreClock = RCC_Get_SysClock_freq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

	InitTick(uwTickPrio);

	return;
}

uint32_t RCC_Get_SysClock_freq(void)
{
	uint32_t pllm = 0, pllvco = 0, pllp = 0;
	uint32_t sysclockfreq = 0;

	/* Get SYSCLK source -------------------------------------------------------*/
	switch (RCC->CFGR & RCC_CFGR_SWS)
	{
	case RCC_SYSCLKSOURCE_STATUS_HSI:
	{
		sysclockfreq = HSI_VALUE;
		break;
	}
	case RCC_SYSCLKSOURCE_STATUS_HSE:
	{
		sysclockfreq = HSE_VALUE;
		break;
	}
	case RCC_SYSCLKSOURCE_STATUS_PLLCLK:
	{
		pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
		if (RCC_GET_PLL_OSCSOURCE() != RCC_PLLCFGR_PLLSRC_HSI)
		{
			pllvco = (uint32_t)((((uint64_t) HSE_VALUE * ((uint64_t)((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
		}
		else
		{
			pllvco = (uint32_t)((((uint64_t) HSI_VALUE * ((uint64_t)((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
		}
		pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2);

		sysclockfreq = pllvco / pllp;
		break;
	}
	default:
	{
		sysclockfreq = HSI_VALUE;
		break;
	}
	}
	return sysclockfreq;
}
