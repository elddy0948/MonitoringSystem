/*
 * stm32f76xxx_flash.h
 *
 *  Created on: Apr 15, 2024
 *      Author: HoJoon
 */

#ifndef INC_STM32F76XXX_FLASH_H_
#define INC_STM32F76XXX_FLASH_H_

#include "stm32f76xxx.h"

#define FLASH_GET_LATENCY()				(READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

#define FLASH_SET_LATENCY(__LATENCY__)	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(__LATENCY__))


#endif /* INC_STM32F76XXX_FLASH_H_ */
