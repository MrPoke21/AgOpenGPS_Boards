#include <Configuration.h>
#include <zInput.h>
#include <freertos/semphr.h>

int16_t steeringPosition = 0;
Adafruit_ADS1115 adc;
bool adcConnected = false;
int16_t current_zero = 0;
int16_t cachedSteeringSensor = 0;
int16_t cachedCurrentSensor = 0;
int16_t filteredSteeringSensor = 0;  // EMA filtered steering
int16_t filteredCurrentSensor = 0;    // EMA filtered current
uint32_t lastADCReadTime = 0;

// Mutex for cache access from both ADC task and main loop
SemaphoreHandle_t adcCacheMutex = NULL;

// Adaptive ADC sampling based on sensor type
#if ADC_SENSOR_TYPE == 0
  const uint16_t ADC_READ_INTERVAL_MS = 20;  // 50Hz for pressure/current sensor
#else
  const uint16_t ADC_READ_INTERVAL_MS = 10;  // 100Hz for PID motor angle feedback
#endif

const float ADC_FILTER_ALPHA = 0.3f;  // EMA smoothing factor (0.2-0.4)

// Forward declarations
void adcTaskFunction(void* parameter);
void initInput() {
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  
  // Create mutex for ADC cache protection
  adcCacheMutex = xSemaphoreCreateMutex();
  
  // Check ADC
  if (adc.begin(0x48)) {  // Specify I2C address explicitly
    DEBUG_PRINTLN("ADC Connection OK");
    adc.setDataRate(RATE_ADS1115_860SPS);
    adc.setGain(GAIN_TWOTHIRDS);
    // Increase I2C clock speed for faster communication
    Wire.setClock(400000);  // 400kHz I2C (standard fast mode)
    adcConnected = true;
    
    // Cache initial readings (blocking, but only once on startup)
    cachedCurrentSensor = adc.readADC_SingleEnded(ADC_CHANNEL_SENSOR);
    cachedSteeringSensor = adc.readADC_SingleEnded(ADC_CHANNEL_STEER);
    // Initialize filtered values
    filteredCurrentSensor = cachedCurrentSensor;
    filteredSteeringSensor = cachedSteeringSensor;
    current_zero = cachedCurrentSensor;
    lastADCReadTime = millis();
    
    // Create FreeRTOS task for ADC reading (runs on Core 1, low priority)
    xTaskCreatePinnedToCore(
      adcTaskFunction,
      "adcRead",
      2048,      // Stack size
      NULL,      // Parameter
      1,         // Priority (low - won't block main loop)
      NULL,      // Task handle
      1          // Core 1 (leaves Core 0 for WiFi)
    );
    
    DEBUG_PRINTLN("[ADC] Background task created for non-blocking reads");
  } else {
    DEBUG_PRINTLN("ADC Connection FAILED!");
  }
}

void inputHandler() {
  if (!adcConnected) {
    return;
  }
  
  // Get filtered sensor value with mutex protection (non-blocking read from cache)
  int16_t sensor = filteredCurrentSensor;
  
  if (xSemaphoreTake(adcCacheMutex, 0) == pdTRUE) {
    sensor = filteredCurrentSensor;  // Safe read
    xSemaphoreGive(adcCacheMutex);
  }
  // If mutex is busy, just use stale value - main loop must not block!
  
  // Validate sensor reading
  if (sensor < 0 || sensor > 32767) {
    DEBUG_PRINTLN("ERROR: Invalid ADC reading");
    return;
  }

  // Pressure sensor?
  if (steerConfig.PressureSensor) {
    sensorSample = sensor * 0.25;  // Fixed: was *= which would accumulate
    sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }
  }

  // Current sensor?
  if (steerConfig.CurrentSensor) {
    sensorSample = abs((float)sensor - current_zero);
    sensorSample = sensorSample * CURRENT_SENSORE_MODIFIER;
    sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
    sensorReading = constrain(sensorReading, 0, 255);  // Better than _min
    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }
  }
}

void calcSteerAngle() {
  if (!adcConnected) {
    return;
  }
  
  // Use cached steering position instead of blocking read every time
  // ADC readings are already updated in a separate interval
  int16_t sensor = filteredSteeringSensor;
  
  // Validate sensor reading
  if (sensor < 0 || sensor > 32767) {
    DEBUG_PRINTLN("ERROR: Invalid steering position ADC reading");
    return;
  }

  if (steeringPosition == 0 && sensor > 0) {
    steeringPosition = sensor >> 1; // 1st time init
  }

  // EMA filter: filtered = (filtered * 7 + raw * 3) / 10
  steeringPosition = (steeringPosition * 7 + (sensor >> 1) * 3) / 10;
  
  // Define calibration constant for readability
  const int16_t CENTER_POSITION = 6805;
  
  helloSteerPosition = steeringPosition - (CENTER_POSITION - 5);  // 6800
  
  // convert position to steer angle. 32 counts per degree of steer pot position
  // in my case
  //   ***** make sure that negative steer angle makes a left turn and positive
  //   value is a right turn *****
  if (steerConfig.InvertWAS) {
    steeringPosition = (steeringPosition - CENTER_POSITION - steerSettings.wasOffset);
    steerAngleActual =
        (float)(steeringPosition) / -steerSettings.steerSensorCounts;
  } else {
    steeringPosition = (steeringPosition - CENTER_POSITION + steerSettings.wasOffset);
    steerAngleActual =
        (float)(steeringPosition) / steerSettings.steerSensorCounts;
  }

  // Ackerman fix - only apply when steering left (negative angle)
  if (steerAngleActual < 0)
    steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);
}

/**
 * ADC Background Task - Runs on Core 1 with low priority
 * Non-blocking I2C reads happen here, not in main loop
 * Reads every 20ms (50Hz) or 10ms (100Hz) based on ADC_SENSOR_TYPE
 * No mutex needed during initialization - only main loop reads from cache after setup
 */
void adcTaskFunction(void* parameter) {
  uint32_t lastReadTime = millis();
  
  while (1) {
    if (!adcConnected) {
      vTaskDelay(pdMS_TO_TICKS(ADC_READ_INTERVAL_MS));
      continue;
    }
    
    uint32_t currentTime = millis();
    if (currentTime - lastReadTime >= ADC_READ_INTERVAL_MS) {
      lastReadTime = currentTime;
      
      // Read both ADC channels (blocking I2C, but in background task)
      int16_t rawSteer = adc.readADC_SingleEnded(ADC_CHANNEL_STEER);
      int16_t rawCurrent = adc.readADC_SingleEnded(ADC_CHANNEL_SENSOR);
      
      // Update cache with mutex protection
      if (xSemaphoreTake(adcCacheMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cachedSteeringSensor = rawSteer;
        cachedCurrentSensor = rawCurrent;
        
        // Apply EMA filter
        filteredSteeringSensor = (int16_t)((1.0f - ADC_FILTER_ALPHA) * filteredSteeringSensor + 
                                           ADC_FILTER_ALPHA * rawSteer);
        filteredCurrentSensor = (int16_t)((1.0f - ADC_FILTER_ALPHA) * filteredCurrentSensor + 
                                          ADC_FILTER_ALPHA * rawCurrent);
        xSemaphoreGive(adcCacheMutex);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));  // Prevent task starvation, short sleep
  }
}

// Non-blocking helper (stub for profiling, actual work is in adcTaskFunction)
void updateADCCacheAsync() {
  // No-op: ADC reading happens in background task
  // This function is kept for optional profiling or future extensions
}
