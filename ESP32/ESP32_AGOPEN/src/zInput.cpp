#include <Configuration.h>
#include <zInput.h>

int16_t steeringPosition = 0; // from steering sensor
Adafruit_ADS1115 adc;         // Use this for the 16-bit version ADS1115
bool adcConnected = false;
int16_t current_zero = 0;
void initInput() {
  // keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  // Check ADC
  if (adc.begin()) {
    Serial.println("ADC Connecton OK");
    adc.setDataRate(RATE_ADS1115_860SPS); // 860 samples per second
    adc.setGain(GAIN_TWOTHIRDS);
    adcConnected = true;
    current_zero = adc.readADC_SingleEnded(1);
  } else {
    Serial.println("ADC Connecton FAILED!");
  }
}

void inputHandler() {
  if (!adcConnected) {
    return;
  }
  int16_t sensore = adc.readADC_SingleEnded(1);
  
  // Validate sensor reading
  if (sensore < 0 || sensore > 32767) {
    Serial.println("ERROR: Invalid ADC reading");
    return;
  }

  // Pressure sensor?
  if (steerConfig.PressureSensor) {
    sensorSample = sensore * 0.25;  // Fixed: was *= which would accumulate
    sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }
  }

  // Current sensor?
  if (steerConfig.CurrentSensor) {
    sensorSample = abs((float)sensore - current_zero);
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

  int16_t sensore = adc.readADC_SingleEnded(0);
  
  // Validate sensor reading
  if (sensore < 0 || sensore > 32767) {
    Serial.println("ERROR: Invalid steering position ADC reading");
    return;
  }

  if (steeringPosition == 0 && sensore > 0) {
    steeringPosition = sensore >> 1; // 1st time init
  }

  // EMA filter: filtered = (filtered * 7 + raw * 3) / 10
  steeringPosition = (steeringPosition * 7 + (sensore >> 1) * 3) / 10;
  
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
