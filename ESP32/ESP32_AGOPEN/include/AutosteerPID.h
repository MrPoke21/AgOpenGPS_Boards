#ifndef AUTOSTEERPID_H
#define AUTOSTEERPID_H

#include <Arduino.h>
#include "main.h"
//pwm variables
int16_t pwmDrive = 0;
float pValue = 0, errorAbs = 0, highLowPerDeg = 0;

void motorDrive();
void calcSteeringPID(void);

#endif