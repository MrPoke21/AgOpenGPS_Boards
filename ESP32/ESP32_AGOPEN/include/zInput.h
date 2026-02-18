#ifndef ZINPUT_H
#define ZINPUT_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "main.h"

/**
 * ADC Sensor Configuration & Task-Based Architecture:
 * Channel 0: Wheel Angle Sensor (WAS) - Primary steering feedback
 * Channel 1: Current/Pressure Sensor (future: PID Motor Angle Feedback)
 * 
 * **OPTIMIZATION: ADC reads run in separate FreeRTOS task (CORE 1, low priority)**
 * - Main loop never blocks on I2C
 * - ADC task reads every 20ms (50Hz) or 10ms (100Hz) based on ADC_SENSOR_TYPE
 * - Main loop uses already-cached, filtered values in <1µs
 * 
 * Sampling Rates (automatically configured):
 * - Type 0 (Pressure/Current): 50Hz (20ms) - sufficient for pressure/current response
 * - Type 1 (PID Motor Angle): 100Hz (10ms) - required for fast PID control stability
 * 
 * To switch to PID motor angle feedback:
 * 1. Set ADC_SENSOR_TYPE = 1 in Configuration.h
 * 2. ADC sampling will automatically increase to 100Hz (10ms)
 * 3. Implement motor angle feedback logic
 * 4. Update inputHandler() to use motor feedback instead of current/pressure
 */

extern int16_t steeringPosition;               // from steering sensor
extern bool adcConnected;

void calcSteerAngle();
void inputHandler();
void initInput();
void updateADCCacheAsync();  // Non-blocking - called from main loop just for profiling tracking (optional)

#endif