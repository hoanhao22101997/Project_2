/*
 * timer.c
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */


#include "timer.h"
struct Timer_Conf_t TimerConfig;
enum TimerStatus gTimerStatus = TIMER_NOT_INITIAL;

void timer_init(TIM_HandleTypeDef *htim){
	gTimerStatus = TIMER_NOT_INITIAL;
	switch(gTimerStatus)
	{
		case TIMER_NOT_INITIAL:
			TimerConfig.htim = htim;
			HAL_TIM_Base_Start(TimerConfig.htim);
			gTimerStatus = TIMER_INITIALIZED;
			break;
		case TIMER_INITIALIZED:
			// TODO
			break;
		default:
			break;
	}
}
void delay(uint16_t ms){

	switch(gTimerStatus)
	{
		case TIMER_NOT_INITIAL:
			// TODO
			break;
		case TIMER_INITIALIZED:
			// TODO becasue you can use HAL_DELAY();
			for(uint16_t i = 0;i<ms;i++){
				delayMicroseconds(1000);
			}
			break;
		default:
			break;
	}
}
void delayMicroseconds(uint16_t us){
	switch(gTimerStatus)
	{
		case TIMER_NOT_INITIAL:
			// TODO
			break;
		case TIMER_INITIALIZED:
			TimerConfig.us = us;
			__HAL_TIM_SET_COUNTER(TimerConfig.htim, 0);
			while (__HAL_TIM_GET_COUNTER(TimerConfig.htim) <TimerConfig.us );
			break;
		default:
			break;
	}
}
