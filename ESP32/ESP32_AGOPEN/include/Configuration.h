#pragma once

/**
 * AgOpenGPS ESP32 Autosteer Configuration
 * Define system parameters, intervals, and calibration constants
 */

// ==================== DEBUG CONFIGURATION ====================
/** Enable debug output to Serial (1: enabled, 0: disabled) */
#define DEBUG 0

// Debug macro - only prints if DEBUG is enabled
#if DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...) ((void)0)
  #define DEBUG_PRINTLN(...) ((void)0)
  #define DEBUG_PRINTF(...) ((void)0)
#endif

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

// ==================== WiFi & UDP CONFIGURATION ====================
/** Enable WiFi and UDP communication (1: enabled, 0: disabled) */
#define ENABLE_UDP 0

/** UDP port for data exchange */
#define UDP_PORT 8888

/** UDP buffer size - max packet size in bytes */
#define UDP_BUFFER_SIZE 1024

/** WiFi operating mode: 1 = AP (creates network), 0 = STA (connects to existing) */
/** Note: AP mode has lower latency (~10-20ms), STA mode higher (~30-50ms) */
#define WIFI_MODE 1

/** WiFi TX Power (dBm): 8=7dBm, 20=20dBm, 78=20.5dBm (maximum) */
#define WIFI_TX_POWER 78

/** Disable WiFi power save mode for lower latency (1: disabled for speed, 0: enabled for power) */
#define WIFI_DISABLE_PS 1

/** WiFi RX buffer count (default 16, increase to 32 for lower latency at cost of RAM) */
#define WIFI_RX_BUF_COUNT 32

/** WiFi TX buffer count (default 32, increase to 64 for better throughput) */
#define WIFI_TX_BUF_COUNT 64

/** WiFi Access Point SSID (max 32 characters) */
#define WIFI_SSID "AGOPEN_ESP32_AP"

/** WiFi Access Point password (max 63 characters) */
#define WIFI_PASS "12345678"

/** UDP broadcast IP address for STA mode */
#define BROADCAST_IP "192.168.0.255"

// ==================== ADC SENSOR CONFIGURATION ====================
/** ADC Channel 0: Steering Wheel Angle Sensor (WAS) */
#define ADC_CHANNEL_STEER 0

/** ADC Channel 1: Pressure/Current Sensor 
 * Note: Future change - will be replaced with PID motor controller angle sensor */
#define ADC_CHANNEL_SENSOR 1

/** ADC Sensor Type: 0 = Current/Pressure, 1 = PID Motor Angle Feedback (future) 
 * 
 * Sampling Rates:
 * - Type 0 (Pressure/Current): 50Hz (20ms interval) - sufficient for slow pressure/current feedback
 * - Type 1 (PID Motor Angle): 100Hz (10ms interval) - required for fast PID control loop
 */
#define ADC_SENSOR_TYPE 0  // Change to 1 when PID motor angle feedback is implemented

// ==================== AUTOSTEER PID CONSTANTS ====================
/** Danfoss valve center position (0-255 PWM) */
#define DANFOSS_CENTER_POS 128

/** Danfoss PWM shift value (right shift by this value divides by 2^n) */
#define DANFOSS_SHIFT_BITS 2

/** Angle range (degrees) for low-to-high PWM transition curve */
#define LOW_HIGH_DEGREES 5.0f

// ==================== STEERING SENSOR CALIBRATION ====================
/** Steering wheel angle sensor (WAS) center position ADC value */
#define WAS_CENTER_POSITION 6805

/** Steering sensor scaling reference point (usually CENTER_POSITION - 5) */
#define WAS_HELLO_POSITION 6800