#include <AutosteerPID.h>

// Danfoss PWM constants
#define DANFOSS_PWM_MIN -250
#define DANFOSS_PWM_MAX 250
#define DANFOSS_SHIFT_BITS 2  // Divide by 4
#define DANFOSS_CENTER_POS 128

void calcSteeringPID(void) {
  //Proportional only
  float steerAngleError = steerAngleActual - steerAngleSetPoint;
  pValue = steerSettings.Kp * steerAngleError;
  pwmDrive = (int16_t)pValue;

  errorAbs = abs(steerAngleError);
  int16_t newMax = 0;

  // Calculate PWM scaling for smooth transition between low and high PWM
  if (highLowPerDeg == 0) {
    highLowPerDeg = (steerSettings.highPWM - steerSettings.lowPWM) / LOW_HIGH_DEGREES;
  }

  if (errorAbs < LOW_HIGH_DEGREES) {
    newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  } else newMax = steerSettings.highPWM;

  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

  //limit the pwm drive
  if (pwmDrive > newMax) pwmDrive = newMax;
  if (pwmDrive < -newMax) pwmDrive = -newMax;


  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

  if (steerConfig.IsDanfoss) {
    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, DANFOSS_PWM_MIN, DANFOSS_PWM_MAX));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> DANFOSS_SHIFT_BITS;  // Divide by 4
    pwmDrive += DANFOSS_CENTER_POS;             // add Center Pos.

    // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
    // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
    // Voh=11.08 volts, Vol=0.185v
    // (11.08/12)*255=235.45
    // (0.185/12)*255=3.93
    // output now lies in the range [67 ... 205], the center position is now 136
    //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
  }
    motorDrive();
}

//#########################################################################################

void motorDrive(void) {
  if (steerEnable) {
    // Used with Cytron MD30C Driver
    // Steering Motor
    // Dir + PWM Signal
    
    if (!motorON){
      analogWrite(PWM_ENABLE, 255);
      motorON = true;
    }
    if (steerConfig.CytronDriver) {
      // Cytron MD30C Driver Dir + PWM Signal
      if (pwmDrive >= 0) {
        ledcWrite(PWM_CHANNEL_LPWM, 255);
      } else {
        ledcWrite(PWM_CHANNEL_LPWM, 0);
      }
      //write out the 0 to 255 value
      ledcWrite(PWM_CHANNEL_RPWM, abs(pwmDrive));
    } else {

      if (pwmDrive > 0) {
        ledcWrite(PWM_CHANNEL_RPWM, 0);
        ledcWrite(PWM_CHANNEL_LPWM, pwmDrive);
      } else {
        ledcWrite(PWM_CHANNEL_LPWM, 0);
        ledcWrite(PWM_CHANNEL_RPWM, abs(pwmDrive));
      }
    }
  } else {
    if (motorON) {
      analogWrite(PWM_ENABLE, 0);
      ledcWrite(PWM_CHANNEL_LPWM, 0);
      ledcWrite(PWM_CHANNEL_RPWM, 0);
      motorON = false;
    }
  }
}
