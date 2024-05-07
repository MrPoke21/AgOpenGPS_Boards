#include "Configuration.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#ifdef WIFI
#include <WiFi.h>
#elif defined(ETHERNET)
#include <Ethernet.h>
#endif
#include "zNMEAParser.h"
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
#define DIR1_RL_ENABLE 27

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM 12

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM 14

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 25
#define WORKSW_PIN 26

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

//Define sensor pin for current or pressure sensor
#define LOAD_SENSOR_PIN 39
#define WAS_SENSOR_PIN 36

#define PACKET_TIMEOUT 1000

#if defined(WIFI) || defined(ETHERNET)
IPAddress myip = IPAddress(0, 0, 0, 0);
IPAddress ipDes = IPAddress(192, 168, 0, 255);  //AOG IP
#endif

#ifdef WIFI
WiFiClient client;
#elif defined(ETHERNET)
EthernetClient client;
#endif

void imuTask();

void gpsStream();

void inputHandler();

void autosteerLoop();

void commandHandler();

void clientConnect();

void BuildNmea();

void autoSteerPacketPerser();

void steerSettingsInit();

// booleans to see if we are using BNO08x
bool useBNO08x = false;

CyclicTimer t_imuTask;
CyclicTimer t_autosteerLoop;
CyclicTimer t_commandHandler;
CyclicTimer t_ClientReconnect;

uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;


//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;
bool steerEnable = false;

//speed sent as *10
float gpsSpeed = 0;
bool GGA_Available = false;  //Do we have GGA on correct port?
bool Autosteer_running = true;

const bool invertRoll = true;  //Used for IMU with dual antenna

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;


//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0;  //the desired angle from AgOpen
//float steerAngleError = 0;     //setpoint - actual
int16_t helloSteerPosition = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;

unsigned long lastPacket = 0;

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
Storage steerSettings;  // 11 bytes

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
Setup steerConfig;  // 9 bytes

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

  uint8_t* getBytes() {
    return data;
  }

private:
  uint8_t data[63];
};

NmeaPGN nmeaData = NmeaPGN();

void autosteerSetup() {
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  
  if (PWM_Frequency == 0) {
    ledcSetup(PWM1_LPWM, 490, 8);
    ledcSetup(PWM2_RPWM, 490, 8);
  } else if (PWM_Frequency == 1) {
    ledcSetup(PWM1_LPWM, 122, 8);
    ledcSetup(PWM2_RPWM, 122, 8);
  } else if (PWM_Frequency == 2) {
    ledcSetup(PWM1_LPWM, 3921, 8);
    ledcSetup(PWM2_RPWM, 3921, 8);
  }*/


  pinMode(DIR1_RL_ENABLE, OUTPUT);
  analogWrite(DIR1_RL_ENABLE, 0);

  pinMode(PWM1_LPWM, OUTPUT);
  pinMode(PWM2_RPWM, OUTPUT);
  analogWrite(PWM1_LPWM, 0);
  analogWrite(PWM2_RPWM, 0);

  EEPROM.begin(80);
  EEPROM.get(0, EEread);  // read identifier

  uint8_t ipArray[] = { 192, 168, 0, 255 };

  if (EEread != EEP_Ident)  // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, ipArray);
    EEPROM.commit();
  } else {
    EEPROM.get(10, steerSettings);  // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, ipArray);
#ifndef USB
    ipDes = IPAddress(ipArray[0], ipArray[1], ipArray[2], ipArray[3]);
#endif
  }
  steerSettingsInit();
}  // End of Setup
/*
TaskHandle_t Task1;

void Task0code(void* pvParameters) {
  for (;;) {
    autoSteerPacketPerser();
    gpsStream();
  }
}
*/
void setup() {
  // Setup Serial Monitor

  Serial.begin(115200);
  Serial2.begin(115200);

  autosteerSetup();

  initUDP();
  initHandler();

  initIMU();

  initInput();

  t_imuTask.setPeriod(50);
  t_autosteerLoop.setPeriod(AUTOSTEER_INTERVAL);
  t_commandHandler.setPeriod(1000);
  t_ClientReconnect.setPeriod(6000);

  /*
  // Setup Serial Monitor
  xTaskCreatePinnedToCore(
    Task0code, //ask function. 
    "Task1",   // name of task. 
    10000,     // Stack size of task 
    NULL,      // parameter of the task 
    1,         // priority of the task 
    &Task1,    // Task handle to keep track of created task 
    0);*/
}

void loop() {
  if (useBNO08x && t_imuTask.tickAndTest()) {
    imuTask();
  }
  gpsStream();
  autoSteerPacketPerser();

  if (t_autosteerLoop.tickAndTest()) {
    autosteerLoop();
  }
  if (t_commandHandler.tickAndTest()) {
    commandHandler();
  }

#ifndef USB
  if (t_ClientReconnect.tickAndTest()) {
    if (!client.connected()) {
      Serial.println("Client has been connecting.....");
      if (client.connect(ipDes, 16666)) {
        Serial.println("Client has been connected");
      }
    }
  }
#endif
}


void printLnByteArray(uint8_t* data, uint8_t datalen) {
  for (int i = 0; i < datalen; i++) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println();
}