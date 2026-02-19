#include <Arduino.h>
#include <main.h>
#include <zAutosteer.h>
#include <zInput.h>
#include <zUDP.h>
#include <AutosteerPID.h>

void readInputSwitches() {
  // Button toggle state variables (static = memory persists between calls)
  static uint8_t currentState = 1;
  static uint8_t reading = 0;
  static uint8_t previous = 0;
  static unsigned long lastDebugTime = 0;

  // read all the switches
  workSwitch = !gpio_get_level((gpio_num_t)WORKSW_PIN);

  if (steerConfig.SteerSwitch == 1) // steer switch on - off
  {
    steerSwitch = gpio_get_level((gpio_num_t)STEERSW_PIN); // read auto steer enable switch (inverted: 1 when shorted to GND)
  } else if (steerConfig.SteerButton == 1) // steer Button momentary
  {
    // Detect steerEnable state change from external sources
    static uint8_t lastSteerEnable = 0;
    
    reading = !gpio_get_level((gpio_num_t)STEERSW_PIN);  // inverted: 1 when button shorted to GND
    
    if (steerEnable != lastSteerEnable) {
      steerSwitch = !steerEnable;  // Sync toggle state with external changes
      currentState = steerSwitch;  // Update toggle state to match switch
      lastSteerEnable = steerEnable;
    }
    
    // Toggle on rising edge (LOW to HIGH transition) - now detects GND release
    if (reading == HIGH && previous == LOW) {
      currentState = currentState ? 0 : 1;  // Toggle state
      steerSwitch = currentState;
    }
    previous = reading;
  } else // No steer switch and no steer button - keep steerSwitch at default (1)
  {
    // When no physical switch is configured, steerSwitch remains 1
    // The guidance status is handled separately via guidanceBit in packet processing
    // This prevents steerSwitch from being affected by guidance packets
  }
  switchByte = 0;
  switchByte |= (steerSwitch << 1); // put steerswitch status in bit 1
                                    // position
  switchByte |= workSwitch;
}

void autosteerLoop() {
  
  inputHandler();

  calcSteerAngle();
  
  // Handle motor on/off state safely (only transitions once)
  motorStateControl();
  
  // Only calculate PID when steering is enabled to save resources
  if (steerEnable) {
    calcSteeringPID();
  }
}
