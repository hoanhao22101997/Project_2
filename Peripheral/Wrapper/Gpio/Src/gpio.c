/*
 * gpio.c
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */
#include "gpio.h"
void pinMode(struct PinSetup *pin, uint8_t pMode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(pMode == 1)
	{
	  GPIO_InitStruct.Pin = pin->Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(pin->GPIOx, &GPIO_InitStruct);
	}else if(pMode == 0)
	{
	  GPIO_InitStruct.Pin = pin->Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(pin->GPIOx, &GPIO_InitStruct);
	}
}

void digitalWrite(struct PinSetup *pin,uint8_t status){
	switch(status)
	{
		case 0:
			HAL_GPIO_WritePin(pin->GPIOx, pin->Pin,GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(pin->GPIOx, pin->Pin,GPIO_PIN_SET);
			break;
		default:
			break;

	}
}
uint8_t digitalRead(struct PinSetup *pin){
	uint8_t ret = HAL_GPIO_ReadPin(pin->GPIOx, pin->Pin);
	return ret;
}

