#ifndef MAIN_H
#define MAIN_H

#include <EEPROM.h>
#include "CyclicTimer.h"

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2500

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define PWM_ENABLE 27

//PWM1 for , Cytron Dir Left PWM for IBT2
#define PWM1_LPWM 12

//Cytron PWM, Right PWM for IBT2
#define PWM2_RPWM 14

#define PWM_FREQ  1000
#define PWM_RESOLUTION  8
#define PWM_CHANNEL_LPWM  0
#define PWM_CHANNEL_RPWM  1

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 25
#define WORKSW_PIN 26

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

//Define sensor pin for current or pressure sensor
#define LOAD_SENSOR_PIN 39
#define WAS_SENSOR_PIN 36

#define PACKET_TIMEOUT 1000

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 2.0

//Functions
void sendData(byte* data, uint8_t datalen);
void printLnByteArray(byte* data, uint8_t datalen);
// booleans to see if we are using BNO08x
extern bool useBNO08x;

extern CyclicTimer t_imuTask;
extern CyclicTimer t_autosteerLoop;

extern uint8_t aog2Count;
extern float sensorReading;
extern float sensorSample;


//EEPROM
extern int16_t EEread;

//Relays
extern bool isRelayActiveHigh;
extern uint8_t relay, relayHi, uTurn;
extern uint8_t tram;

//Switches
extern uint8_t workSwitch, steerSwitch, switchByte;

//On Off
extern uint8_t guidanceStatus;
extern uint8_t prevGuidanceStatus;
extern bool guidanceStatusChanged;
extern bool steerEnable;

//speed sent as *10
extern float gpsSpeed;
extern bool GGA_Available;  //Do we have GGA on correct port?
extern bool Autosteer_running;

extern const bool invertRoll; //Used for IMU with dual antenna

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};
extern euler_t ypr;


//steering variables
extern float steerAngleActual;
extern float steerAngleSetPoint;  //the desired angle from AgOpen
//float steerAngleError = 0;     //setpoint - actual
extern int16_t helloSteerPosition;
extern uint8_t pwmDisplay;
extern bool motorON;
//Steer switch button  ***********************************************************************************************************
extern uint8_t currentState, reading, previous;

extern unsigned long lastPacket;

//Variables for settings
struct Storage {
  uint8_t Kp = 40;      // proportional gain
  uint8_t lowPWM = 10;  // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;  // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;  // sent as percent
};
extern Storage steerSettings;  // 11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;  // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  // 1 if switch selected
  uint8_t SteerButton = 0;  // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;  //Set to 0 to use X Axis, 1 to use Y avis
};
extern Setup steerConfig;  // 9 bytes

class NmeaPGN {
public:
  NmeaPGN() {
    data[0] = 0x80;
    data[1] = 0x81;
    data[2] = 0x7C;
    data[3] = 0xD6;
    data[4] = 0x39;  // nmea total array count
  };

  void writeDouble(double value, int index) {
    byte* tmp = (byte*)&value;
    for (int i = 0; i < 8; i++) {
      data[i + index] = tmp[i];
    }
  }

  void writeFloat(float value, int index) {
    byte* tmp = (byte*)&value;
    for (int i = 0; i < 4; i++) {
      data[i + index] = tmp[i];
    }
  }

  void writeInt(int value, int index) {
    byte* tmp = (byte*)&value;
    for (int i = 0; i < 4; i++) {
      data[i + index] = tmp[i];
    }
  }

  void writeShort(short value, int index) {
    byte* tmp = (byte*)&value;
    for (int i = 0; i < 4; i++) {
      data[i + index] = tmp[i];
    }
  }

  void writeByte(byte value, int index) {
    data[index] = value;
  }

  byte* getBytes() {
    return data;
  }

private:
  byte data[63];
};

extern NmeaPGN nmeaData;

#endif