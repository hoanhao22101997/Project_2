/*
 * SharpGP2Y10.c
 *
 *  Created on: Sep 13, 2024
 *      Author: phat.nguyen-thanh
 */
#include "SharpGP2Y10.h"
struct SharpGP2Y10 sharpgp2y10_config;

SharpGP2Y10_init(int voPin, int ledPin) {
//    pinMode(ledPin, OUTPUT);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
    sharpgp2y10_config.voPin = voPin;
    sharpgp2y10_config.ledPin = ledPin;
}



void calc() {
//    digitalWrite(ledPin, LOW); //turn on LED
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);

//    delayMicroseconds(SharpGP2Y10_SAMPLINGTIME);     //Sampling time

//    sharpgp2y10_config.volMeasured = analogRead(voPin);    //read ADC VoPin

//    delayMicroseconds(SharpGP2Y10_DELTATIME);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
//    digitalWrite(ledPin, HIGH); //turn off LED
//    delayMicroseconds(SharpGP2Y10_SLEEPINGTIME);

    sharpgp2y10_config.calcVoltage = sharpgp2y10_config.volMeasured * (vccVol / 1024);   //calc real Voltage

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
