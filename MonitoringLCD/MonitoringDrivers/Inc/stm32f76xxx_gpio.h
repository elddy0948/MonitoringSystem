/*
 * stm32f76xxx_gpio.h
 *
 *  Created on: Apr 11, 2024
 *      Author: HoJoon
 */

#ifndef STM32F76XXX_GPIO_H_
#define STM32F76XXX_GPIO_H_

#include "stm32f76xxx.h"

typedef struct
{
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t OutputType;
	uint8_t OutputSpeed;
	uint8_t PullUpPullDown;
	uint8_t AlternateFunction;
} GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegisterDefinition_t* pGPIOx;
	GPIO_PinConfig_t PinConfig;
} GPIO_Handle_t;


/* PIN NUMBER */
#define GPIO_PIN_NUMBER_0	0
#define GPIO_PIN_NUMBER_1	1
#define GPIO_PIN_NUMBER_2	2
#define GPIO_PIN_NUMBER_3	3
#define GPIO_PIN_NUMBER_4	4
#define GPIO_PIN_NUMBER_5	5
#define GPIO_PIN_NUMBER_6	6
#define GPIO_PIN_NUMBER_7	7
#define GPIO_PIN_NUMBER_8	8
#define GPIO_PIN_NUMBER_9	9
#define GPIO_PIN_NUMBER_10	10
#define GPIO_PIN_NUMBER_11	11
#define GPIO_PIN_NUMBER_12	12
#define GPIO_PIN_NUMBER_13	13
#define GPIO_PIN_NUMBER_14	14
#define GPIO_PIN_NUMBER_15	15

/* PIN MODE */
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT			1
#define GPIO_MODE_ALTERNATEFUNCTION	2
#define GPIO_MODE_ANALOG			3

/* OUTPUT TYPE */
#define GPIO_OUTPUT_TYPE_PP			0
#define GPIO_OUTPUT_TYPE_OD			1

/* PIN SPEED */
#define GPIO_PIN_SPEED_LOW			0
#define GPIO_PIN_SPEED_MEDIUM		1
#define GPIO_PIN_SPEED_HIGH			2
#define GPIO_PIN_SPEED_VERY_HIGH	3

/* PULL UP PULL DOWN */
#define GPIO_PUPD_NO_PUPD			0
#define GPIO_PUPD_PU				1
#define GPIO_PUPD_PD				2

/* ALTERNATE FUNCTIONS */
#define GPIO_ALT_FUNCTION_0			0
#define GPIO_ALT_FUNCTION_1			1
#define GPIO_ALT_FUNCTION_2			2
#define GPIO_ALT_FUNCTION_3			3
#define GPIO_ALT_FUNCTION_4			4
#define GPIO_ALT_FUNCTION_5			5
#define GPIO_ALT_FUNCTION_6			6
#define GPIO_ALT_FUNCTION_7			7
#define GPIO_ALT_FUNCTION_8			8
#define GPIO_ALT_FUNCTION_9			9
#define GPIO_ALT_FUNCTION_10		10
#define GPIO_ALT_FUNCTION_11		11
#define GPIO_ALT_FUNCTION_12		12
#define GPIO_ALT_FUNCTION_13		13
#define GPIO_ALT_FUNCTION_14		14
#define GPIO_ALT_FUNCTION_15		15

/* Useful macros */
#define GPIO_PIN_SET		1
#define GPIO_PIN_CLEAR		0

/* External interrupt macros */
#define GET_EXTI_PORT( x )		( (x == GPIOB) ? 1 : \
								  (x == GPIOC) ? 2 : 0 )

#define EXIT_PORTB		1
#define EXIT_PORTC		2

/* GPIO FUNCTIONS */
void GPIO_Initialize(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInitialize(GPIO_RegisterDefinition_t* pGPIOx);

void GPIO_write_to_pin(GPIO_RegisterDefinition_t* pGPIOx, uint8_t pin, uint8_t value);
void GPIO_write_to_port(GPIO_RegisterDefinition_t* pGPIOx, uint16_t value);

#endif /* STM32F76XXX_GPIO_H_ */
