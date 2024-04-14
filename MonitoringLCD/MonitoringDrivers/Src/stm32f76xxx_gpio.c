/*
 * stm32f76xxx_gpio.c
 *
 *  Created on: Apr 11, 2024
 *      Author: HoJoon
 */

#include "stm32f76xxx_gpio.h"

void GPIO_Initialize(GPIO_Handle_t* pGPIOHandle)
{
	if(pGPIOHandle->PinConfig.PinMode <= GPIO_MODE_ANALOG)
	{
		// Clear mode bits
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->PinConfig.PinMode << (2 * pGPIOHandle->PinConfig.PinNumber));
	}
	else
	{
		// Interrupt mode
		uint8_t exticr_position, exticr_pin_position, port = 0;
		exticr_position = pGPIOHandle->PinConfig.PinNumber / 4;
		exticr_pin_position = pGPIOHandle->PinConfig.PinNumber % 4;
		port = GET_EXTI_PORT(pGPIOHandle->pGPIOx);

		SYSCFG->EXTICR[exticr_position] |= (port << (exticr_pin_position * 4));

		switch(pGPIOHandle->PinConfig.PinMode)
		{
		case GPIO_MODE_IT_FALLING:
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
			break;
		case GPIO_MODE_IT_RISING:
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
			break;
		case GPIO_MODE_IT_FALLING_RISING:
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			break;
		default:
			break;
		}

		SYSCFG_PCLK_ENABLE();

		EXTI->IMR |= (1 << pGPIOHandle->PinConfig.PinNumber);
	}

	// Output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->PinConfig.OutputType << pGPIOHandle->PinConfig.PinNumber);

	// Output speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->PinConfig.OutputSpeed << pGPIOHandle->PinConfig.PinNumber);

	/* PullUp PullDown control */
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->PinConfig.PullUpPullDown << pGPIOHandle->PinConfig.PinNumber);

	/* Alternate functionality */
	if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_ALTERNATEFUNCTION)
	{
		uint8_t afr_position, pin_position = 0;

		afr_position = pGPIOHandle->PinConfig.PinNumber / 8;
		pin_position = pGPIOHandle->PinConfig.PinNumber % 8;

		// Clear position
		pGPIOHandle->pGPIOx->AFR[afr_position] &= ~(0xF << (4 * pin_position));
		pGPIOHandle->pGPIOx->AFR[afr_position] |= (pGPIOHandle->PinConfig.AlternateFunction << (4 * pin_position));
	}
}

void GPIO_DeInitialize(GPIO_RegisterDefinition_t* pGPIOx)
{
	if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
}

void GPIO_write_to_pin(GPIO_RegisterDefinition_t* pGPIOx, uint8_t pin, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pin);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pin);
	}
}

void GPIO_write_to_port(GPIO_RegisterDefinition_t* pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_IRQ_interrupt_config(uint8_t IRQNumber, uint8_t status)
{
	 if(status == ENABLE)
	 {
		 NVIC_ISER->ISER[IRQNumber / 32] |= (1 << (IRQNumber % (32 * (IRQNumber / 32))));
	 }
	 else
	 {
		 NVIC_ICER->ICER[IRQNumber / 32] |= (1 << (IRQNumber % (32 * (IRQNumber / 32))));
	 }
}

void GPIO_IRQ_priority_config(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDRESS + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQ_handling(uint8_t pin)
{
	if(EXTI->PR & (1 << pin))
	{
		EXTI->PR |= (1 << pin);
	}
}
