/*
 * SharpGP2Y10.c
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */
#include "SharpGP2Y10.h"
struct SharpGP2Y10 sharpgp2y10_config;


void SharpGP2Y10_Init(ADC_HandleTypeDef *adcx, struct PinSetup *LedPin) {
    sharpgp2y10_config.adcx = adcx;
    sharpgp2y10_config.LedPin->GPIOx = LedPin->GPIOx;
    sharpgp2y10_config.LedPin->Pin = LedPin->Pin;
    pinMode(sharpgp2y10_config.LedPin, OUTPUT);
}



void calc() {
    digitalWrite(sharpgp2y10_config.LedPin, LOW); //turn on LED

    delayMicroseconds(SharpGP2Y10_SAMPLINGTIME);     //Sampling time

    sharpgp2y10_config.volMeasured = (float)analogRead(sharpgp2y10_config.adcx);    //read ADC VoPin

    delayMicroseconds(SharpGP2Y10_DELTATIME);
    digitalWrite(sharpgp2y10_config.LedPin, HIGH); //turn on LED
    delayMicroseconds(SharpGP2Y10_SLEEPINGTIME);

    sharpgp2y10_config.calcVoltage = (sharpgp2y10_config.volMeasured * (sharpgp2y10_config.vccVol / 1024));   //calc real Voltage

    // Linear equation taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    sharpgp2y10_config.dustDensity = 0.17 * sharpgp2y10_config.calcVoltage - 0.1;



    if (sharpgp2y10_config.dustDensity < 0)
    {
    	sharpgp2y10_config.dustDensity = 0.00;
    }
}

// recalculator
float getDustDensity()
{
    calc();
    return sharpgp2y10_config.dustDensity;
}

float getVotage()
{
    calc();
    return sharpgp2y10_config.calcVoltage;
}

int getADC()
{
    calc();
    return sharpgp2y10_config.volMeasured;
}



//Get fields value only
float getDustDensityField()
{
    return sharpgp2y10_config.dustDensity;
}

float getVotageField()
{
    return sharpgp2y10_config.calcVoltage;
}

int getADCField()
{
    return sharpgp2y10_config.volMeasured;
}
