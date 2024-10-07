/*
 * timer.h
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */

#ifndef WRAPPER_TIMER_INC_TIMER_H_
#define WRAPPER_TIMER_INC_TIMER_H_

#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_tim.h"

enum TimerStatus
{
	TIMER_NOT_INITIAL,
	TIMER_INITIALIZED
};
struct Timer_Conf_t
{
	TIM_HandleTypeDef *htim;
	uint16_t 		   us;
};
void timer_init(TIM_HandleTypeDef *htim);
void delay(uint16_t ms);
void delayMicroseconds(uint16_t us);

extern struct Timer_Conf_t TimerConfig;
#endif /* WRAPPER_TIMER_INC_TIMER_H_ */
