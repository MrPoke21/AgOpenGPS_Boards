#pragma once

//Uncomment one connection type
//#define WIFI
#define USB
//#define ETHERNET

#ifdef WIFI
  //#define WIFI_AP_MODE
  #define WIFI_SID "96"
  #define WIFI_PASS "Jelszo1234"
#endif

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 2.0

//
#define AUTOSTEER_INTERVAL 100

//Acceleration /tick steer wheel
#define MAX_ACCEL  20

