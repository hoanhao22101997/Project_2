/*
 * gpio.h
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */

#ifndef WRAPPER_GPIO_INC_GPIO_H_
#define WRAPPER_GPIO_INC_GPIO_H_

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

#define INPUT 	0
#define OUTPUT 	1

#define LOW		0
#define HIGH	1


struct PinSetup
{
	GPIO_TypeDef  *GPIOx;
	uint32_t Pin;       /*!< Specifies the GPIO pins to be configured. */
};

void pinMode(struct PinSetup *pin, uint8_t pMode);

void digitalWrite(struct PinSetup *pin,uint8_t status);

uint8_t digitalRead(struct PinSetup *pin);

#endif /* WRAPPER_GPIO_INC_GPIO_H_ */
