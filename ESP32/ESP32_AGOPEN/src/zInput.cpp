#include <zInput.h>

int16_t steeringPosition = 0;               //from steering sensor
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);  // Use this for the 16-bit version ADS1115
bool adcConnected = false;

void initInput() {
  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  // Check ADC
  if (adc.testConnection()) {
    Serial.println("ADC Connecton OK");
    adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);  //128 samples per second
    adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    adcConnected = true;
  } else {
    Serial.println("ADC Connecton FAILED!");
  }
}


void inputHandler() {
  // Load sensor?
  if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
  }

  // Pressure sensor?
  if (steerConfig.PressureSensor) {
    sensorSample *= 0.25;
    sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1;  // reset values like it turned off
      currentState = 1;
      previous = 0;
    }
  }

  // Current sensor?
  if (steerConfig.CurrentSensor) {
    sensorSample = (abs(775 - sensorSample)) * 0.5;
    sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
    sensorReading = _min(sensorReading, 255);

    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1;  // reset values like it turned off
      currentState = 1;
      previous = 0;
    }
  }
}

void calcSteerAngle() {
  if (adcConnected) {
    if (steerConfig.SingleInputWAS)  //Single Input ADS
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
    } else  //ADS1115 Differential Mode
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
    }

    steeringPosition = adc.getConversion();
    adc.triggerConversion();
  }
  steeringPosition = (steeringPosition >> 1);  //bit shift by 2  0 to 13610 is 0 to 5v
  helloSteerPosition = steeringPosition - 6800;
  //convert position to steer angle. 32 counts per degree of steer pot position in my case
  //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
  if (steerConfig.InvertWAS) {
    steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);  // 1/2 of full scale
    steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
  } else {
    steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);  // 1/2 of full scale
    steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
  }

  //Ackerman fix
  if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);
}
