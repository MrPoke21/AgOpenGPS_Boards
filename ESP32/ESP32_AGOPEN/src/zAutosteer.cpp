#include <Arduino.h>
#include <main.h>
#include <zAutosteer.h>
#include <zInput.h>

void autosteerLoop() {

  if (Autosteer_running) {
    // read all the switches
    workSwitch = digitalRead(WORKSW_PIN); // read work switch

    if (steerConfig.SteerSwitch == 1) // steer switch on - off
    {
      steerSwitch = digitalRead(
          STEERSW_PIN); // read auto steer enable switch open = 0n closed = Off
    } else if (steerConfig.SteerButton == 1) // steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH) {
        if (currentState == 1) {
          currentState = 0;
          steerSwitch = 0;
        } else {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    } else // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 &&
          previous == 0) {
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the
      // guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 &&
          previous == 1) {
        steerSwitch = 1;
        previous = 0;
      }
    }

    inputHandler();

    switchByte = 0;
    switchByte |= (steerSwitch << 1); // put steerswitch status in bit 1
                                      // position
    switchByte |= workSwitch;
    calcSteerAngle();
    if ((steerEnable && motorON) || (steerEnable != motorON)) {
      calcSteeringPID();
    }
    // end of timed loop
  }

  // Speed pulse

} // end of main loop
