/*
 * analog.c
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */

#include "analog.h"
uint16_t analogRead(ADC_HandleTypeDef *adcx) {
	HAL_ADC_Start(adcx);
	HAL_ADC_PollForConversion(adcx, 1000);
	uint16_t ret = HAL_ADC_GetValue(adcx);
	HAL_ADC_Stop(adcx);
	return ret;

}

