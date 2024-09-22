/*
 * analog.h
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */

#ifndef WRAPPER_ANALOG_INC_ANALOG_H_
#define WRAPPER_ANALOG_INC_ANALOG_H_
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_adc.h"

uint16_t analogRead(ADC_HandleTypeDef *adcx);


#endif /* WRAPPER_ANALOG_INC_ANALOG_H_ */
