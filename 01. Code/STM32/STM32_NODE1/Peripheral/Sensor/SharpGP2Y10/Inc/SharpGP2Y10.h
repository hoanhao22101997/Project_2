/*
 * SharpGP2Y10.h
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */

#ifndef SENSOR_SHARPGP2Y10_INC_SHARPGP2Y10_H_
#define SENSOR_SHARPGP2Y10_INC_SHARPGP2Y10_H_
/**************************************************************************/
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_adc.h"
#include "timer.h"
#include "gpio.h"
#include "analog.h"
/*!
@file     SharpGP2Y10.h
@author   lnquy065
@license  GNU GPLv3
@version  1.0
First version of an Arduino Library for the SharpGP2Y10 dust sensor
*/
/**************************************************************************/


    //Mỗi xung lấy mẫu là 10ms
    //Mỗi xung bật LED là 0.32ms, trong đó:
    //  + 0.28ms: Thời gian lấy mẫu.
    //  + 0.04ms: Thời gian không lấy mẫu.
    // => Qui trình:
    //  + Bật LED, delay 0.28ms
    //  + Đọc Input, delay 0.04ms
    //  + Tắt LED, delay 9.680ms
#define SharpGP2Y10_SAMPLINGTIME 280
#define SharpGP2Y10_DELTATIME 40
#define SharpGP2Y10_SLEEPINGTIME 9680

struct SharpGP2Y10
{
	ADC_HandleTypeDef 	*adcx;
	struct PinSetup		LedPin;
    float 				dustDensity;
    float 				volMeasured;
    float 				calcVoltage;
    float 				vccVol;
};

float getDustDensityField();
float getDustDensity();

float getVotageField();
float getVotage();

int getADCField();
int getADC();

void SharpGP2Y10_Init(ADC_HandleTypeDef 	*adcx, struct PinSetup *LedPin);


extern struct SharpGP2Y10 sharpgp2y10_config;
#endif /* SENSOR_SHARPGP2Y10_INC_SHARPGP2Y10_H_ */
