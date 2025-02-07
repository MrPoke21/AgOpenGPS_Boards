#ifndef ZINPUT_H
#define ZINPUT_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "main.h"
extern int16_t steeringPosition;               //from steering sensor
extern bool adcConnected;

void calcSteerAngle();
void inputHandler();
void initInput();

#endif