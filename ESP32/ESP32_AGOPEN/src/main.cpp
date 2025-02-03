#include "Configuration.h"
#include <main.h>
#include <Wire.h>
#include "zNMEAParser.h"
#include <Arduino.h>
#include <zInput.h>
#include <zHandlers.h>
#include <zPackets.h>
#include <zAutosteer.h>

bool useBNO08x = false;

CyclicTimer t_imuTask;
CyclicTimer t_autosteerLoop;

uint8_t aog2Count = 0;
float sensorReading= 0;
float sensorSample = 0;


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
bool motorON = true;

//speed sent as *10
float gpsSpeed = 0;
bool GGA_Available = false;  //Do we have GGA on correct port?
bool Autosteer_running = true;

const bool invertRoll = true;  //Used for IMU with dual antenna

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0;  //the desired angle from AgOpen
//float steerAngleError = 0;     //setpoint - actual
int16_t helloSteerPosition = 0;
uint8_t pwmDisplay = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;

unsigned long lastPacket = 0;

NmeaPGN nmeaData = NmeaPGN();
Setup steerConfig = Setup();
Storage steerSettings = Storage(); 

void autosteerSetup() {


  pinMode(PWM_ENABLE, OUTPUT);
  analogWrite(PWM_ENABLE, 0);

  ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PWM1_LPWM, PWM_CHANNEL_LPWM);
  ledcAttachPin(PWM2_RPWM, PWM_CHANNEL_RPWM);

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
  }
}  // End of Setup


void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  //set up communication
  Wire.begin();
  delay(500);
  autosteerSetup();

  initIMU();
  
  initHandler();

  initInput();

  t_imuTask.setPeriod(50);
  t_autosteerLoop.setPeriod(AUTOSTEER_INTERVAL);
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
}


void printLnByteArray(byte* data, uint8_t datalen) {
  for (int i = 0; i < datalen; i++) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void sendData(byte* data, uint8_t datalen) {

  int16_t CK_A = 0;
  for (char i = 2; i < datalen - 1; i++) {
    CK_A = (CK_A + data[i]);
  }
  data[datalen - 1] = CK_A;

  while (Serial.availableForWrite() < datalen) {
        delayMicroseconds(10); // Várj rövid időt, amíg van hely a pufferben
    }
  Serial.write(data, datalen);
  Serial.flush();
}