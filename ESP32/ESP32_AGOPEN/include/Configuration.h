#pragma once

/**
 * AgOpenGPS ESP32 Autosteer Configuration
 * Define system parameters, intervals, and calibration constants
 */

// ==================== TIMING ====================
/** Autosteer PID calculation interval in milliseconds */
#define AUTOSTEER_INTERVAL 100

// ==================== MOTOR CONTROL ====================
/** Maximum acceleration for steering wheel per tick (PWM units/ms) */
#define MAX_ACCEL  20

// ==================== SENSOR CALIBRATION ====================
/** Current sensor scaling modifier (0.1 = 0.1A per ADC unit for 40W 12V motor) */
#define CURRENT_SENSORE_MODIFIER 0.1

// ==================== ADC CONFIGURATION ====================
/** ADC sampling validation: max readings per second */
#define ADC_MAX_SAMPLES_PER_SEC 860  // ADS1115 @ 860 SPS

// ==================== SAFETY LIMITS ====================
/** Maximum steering angle limit in degrees */
#define MAX_STEER_ANGLE 45.0f

/** Minimum PWM threshold to avoid motor hum */
#define MIN_PWM_THRESHOLD 9

/** Maximum PWM value for safety */
#define MAX_PWM_VALUE 255