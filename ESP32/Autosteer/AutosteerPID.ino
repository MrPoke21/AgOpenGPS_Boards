

//pwm variables
int16_t pwmDisplay = 0, pwmDrive = 0;
float pValue = 0, errorAbs = 0, highLowPerDeg = 0;
bool motorON = true;


void steerSettingsInit() {
}

void calcSteeringPID(void) {
  //Proportional only
  float steerAngleError = steerAngleActual - steerAngleSetPoint;
  pValue = steerSettings.Kp * steerAngleError;
  pwmDrive = (int16_t)pValue;

  errorAbs = abs(steerAngleError);
  int16_t newMax = 0;

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
    pwmDrive = (constrain(pwmDrive, -250, 250));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2;  // Devide by 4
    pwmDrive += 128;           // add Center Pos.

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
      analogWrite(DIR1_RL_ENABLE, 255);
      motorON = true;
    }
    if (steerConfig.CytronDriver) {
      // Cytron MD30C Driver Dir + PWM Signal
      if (pwmDrive >= 0) {
        analogWrite(PWM1_LPWM, 255);
      } else {
        analogWrite(PWM1_LPWM, 0);
      }
      //write out the 0 to 255 value
      analogWrite(PWM2_RPWM, abs(pwmDrive));
    } else {

      if (pwmDrive > 0) {
        analogWrite(PWM2_RPWM, 0);  //Turn off before other one on
        analogWrite(PWM1_LPWM, pwmDrive);
      } else {
        analogWrite(PWM1_LPWM, 0);  //Turn off before other one on
        analogWrite(PWM2_RPWM, abs(pwmDrive));
      }
    }
  } else {
    if (motorON) {
      analogWrite(DIR1_RL_ENABLE, 0);
      if (steerConfig.CytronDriver) {
        analogWrite(PWM1_LPWM, 0);
      }
      motorON = false;
    }
  }
}
