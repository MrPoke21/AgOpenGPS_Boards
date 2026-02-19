#include "Configuration.h"
#include <main.h>
#include <Wire.h>
#include "zNMEAParser.h"
#include <Arduino.h>
#include <zInput.h>
#include <zHandlers.h>
#include <zPackets.h>
#include <zAutosteer.h>
#include <zUDP.h>
#include <zSerial.h>

bool useBNO08x = false;

CyclicTimer t_imuTask;
CyclicTimer t_inputSwitches;
CyclicTimer t_autosteerLoop;

uint8_t aog2Count = 0;
float sensorReading= 0;


//EEPROM
int16_t EEread = 0;

//Relays
uint8_t relay = 0, relayHi = 0;
uint8_t tram = 0;

//Switches
uint8_t workSwitch = 0, steerSwitch = 1, switchByte = 0;  // steerSwitch defaults to 1 (enabled) when no switch configured

// Steer enable state tracking - prevent rapid toggling
bool prevSteerEnableCondition = false;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;
bool steerEnable = false;

//speed sent as *10
float gpsSpeed = 0;
bool GGA_Available = false;  //Do we have GGA on correct port?

const bool invertRoll = true;  //Used for IMU with dual antenna

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0;  //the desired angle from AgOpen
//float steerAngleError = 0;     //setpoint - actual
int16_t helloSteerPosition = 0;
uint8_t pwmDisplay = 0;

NmeaPGN nmeaData = NmeaPGN();
Setup steerConfig = Setup();
Storage steerSettings = Storage(); 

void autosteerSetup() {

  // Setup switch pins with internal pull-up (active-low configuration)
  pinMode(WORKSW_PIN, INPUT_PULLUP);   // Work switch - pulled high, shorted to GND when active
  pinMode(STEERSW_PIN, INPUT_PULLUP);  // Steer switch - pulled high, shorted to GND when active

  pinMode(PWM_ENABLE, OUTPUT);
  digitalWrite(PWM_ENABLE, LOW);

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
  Serial.begin(460800);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(500);
  
  //set up communication
  Wire.begin();
  delay(500);
  autosteerSetup();

  // Initialize Serial queue (always, whether UDP is enabled or not)
  if (!initSerialQueue()) {
    DEBUG_PRINTLN("[SETUP] Serial queue initialization failed");
  }

  // Initialize WiFi and UDP
#if ENABLE_UDP
  if (initWiFi()) {
    delay(500);
    initUDP();
    printWiFiStatus();
  } else {
    DEBUG_PRINTLN("[SETUP] WiFi initialization failed");
  }
#endif

  initIMU();
  
  initHandler();

  initInput();

  t_imuTask.setPeriod(50);
  t_inputSwitches.setPeriod(200);  // 5 Hz for input switches
  t_autosteerLoop.setPeriod(AUTOSTEER_INTERVAL);
}

void loop() {
  
  if (useBNO08x && t_imuTask.tickAndTest()) {
    imuTask();
  }

  // Non-blocking ADC cache update (every 20ms)
  updateADCCacheAsync();  // Non-blocking - actual ADC reads happen in background task

  gpsStream();

  autoSteerPacketPerser();

  if (t_inputSwitches.tickAndTest()) {
    readInputSwitches();  // Read switches at 5 Hz
  }

  if (t_autosteerLoop.tickAndTest()) {
    autosteerLoop();
  }
}


void printLnByteArray(byte* data, uint8_t datalen) {
  for (int i = 0; i < datalen; i++) {
    DEBUG_PRINT(data[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();
}

void sendData(byte* data, uint8_t datalen) {

  int16_t CK_A = 0;
  for (char i = 2; i < datalen - 1; i++) {
    CK_A = (CK_A + data[i]);
  }
  data[datalen - 1] = CK_A;
  
  DEBUG_PRINT("[SEND] Sending ");
  DEBUG_PRINT(datalen);
  DEBUG_PRINT(" bytes via ");
  
  // Send via Serial and/or UDP depending on configuration
#if ENABLE_UDP
  // Send both Serial and UDP
  DEBUG_PRINTLN("UDP");
  //Serial.write(data, datalen);
  if (!sendUDP(data, datalen)) {
    DEBUG_PRINTLN("[SEND] ERROR: UDP queue full - packet dropped!");
  }
#else
  // Only Serial - send via queue (non-blocking)
  DEBUG_PRINTLN("Serial");
  if (!sendSerial(data, datalen)) {
    DEBUG_PRINTLN("[SEND] ERROR: Serial queue full - packet dropped!");
  }
#endif
}