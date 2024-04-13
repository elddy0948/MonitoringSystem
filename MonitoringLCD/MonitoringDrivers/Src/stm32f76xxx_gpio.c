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

