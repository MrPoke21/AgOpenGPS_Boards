#ifndef AUTOSTEERPID_H
#define AUTOSTEERPID_H

#include <Arduino.h>
#include "main.h"
//pwm variables - declared as extern (defined in AutosteerPID.cpp)
extern int16_t pwmDrive;
extern float pValue, errorAbs, highLowPerDeg;

void motorStateControl();
void motorDrive();
void calcSteeringPID(void);

#endif