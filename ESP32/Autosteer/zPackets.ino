#ifdef WIFI
#include "esp_wifi.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

//uint8_t data[128];
uint8_t* buffer;
char packetBuffer[256];
int stateIndex = 255;
int totalHeaderByteCount = 5;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//fromAutoSteerData FA 250 - sensor values etc
uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

int packetSize;

unsigned int AOGNtripPort = 2233;      // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;  // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;   // Port of AOG that listens

#ifdef WIFI
WiFiUDP udp;
WiFiUDP ntrip;
#elif defined(ETHERNET)
EthernetUDP udp;
EthernetUDP ntrip;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
#endif

void initUDP() {

  // heading
  PGN_253[7] = (uint8_t)9999;
  PGN_253[8] = 9999 >> 8;

  // roll
  PGN_253[9] = (uint8_t)8888;
  PGN_253[10] = 8888 >> 8;
#ifdef WIFI
#ifdef WIFI_AP_MODE
  WiFi.mode(WIFI_AP);                // Changing ESP32 wifi mode to AccessPoint
  WiFi.softAP(WIFI_SID, WIFI_PASS);  //Starting AccessPoint on given credential
#else
  // Supress Debug information
  WiFi.begin(WIFI_SID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
#endif

  WiFi.useStaticBuffers(true);
  esp_wifi_set_ps(WIFI_PS_NONE);

  myip = WiFi.localIP();
  // Connected!
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#elif defined(ETHERNET)
  Ethernet.init(5);
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      ESP.restart();
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
      ESP.restart();
    }
  } else {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
  }
#endif
#ifndef USB
  udp.begin(AOGAutoSteerPort);
  ntrip.begin(AOGNtripPort);
#endif
}

void autoSteerPacketPerser() {
  int packetLenght = 0;
  int packetSize = 0;
#ifndef USB
  if (client.connected()) {
    packetLenght = client.available();
    if (packetLenght > 0) {
      packetSize = client.read((uint8_t*)packetBuffer, packetLenght);
      parsePacket((uint8_t*)packetBuffer, packetSize, client.remoteIP());
    }
  }

  packetSize = ntrip.parsePacket();
  if (packetSize > 0) {
    packetLenght = ntrip.read(packetBuffer, 256);
    Serial2.write(packetBuffer, packetLenght);
  }


  packetSize = udp.parsePacket();

  if (packetSize <= 6)
    return;

  packetLenght = udp.read(packetBuffer, 256);
  if (packetLenght > 0) {
    parsePacket((uint8_t*)packetBuffer, packetLenght, udp.remoteIP());
  }
#else
  int aas = Serial.available();
  byte a;
  for (int i = 0; i < aas; i++) {
    a = (char)Serial.read();

    switch (packetBuffer[stateIndex]) {
      case 0:  //find 0x80
        {
          if (a == 128) packetBuffer[packetBuffer[stateIndex]++] = a;
          else packetBuffer[stateIndex] = 0;
          break;
        }

      case 1:  //find 0x81
        {
          if (a == 129) packetBuffer[packetBuffer[stateIndex]++] = a;
          else {
            if (a == 181) {
              packetBuffer[stateIndex] = 0;
              packetBuffer[packetBuffer[stateIndex]++] = a;
            } else packetBuffer[stateIndex] = 0;
          }
          break;
        }
      case 2:  //Source Address (7F)
        {
          if (a < 128 && a > 120)
            packetBuffer[packetBuffer[stateIndex]++] = a;
          else packetBuffer[stateIndex] = 0;
          break;
        }
      case 3:  //PGN ID
        {
          packetBuffer[packetBuffer[stateIndex]++] = a;
          break;
        }

      case 4:  //Num of data bytes
        {
          packetBuffer[packetBuffer[stateIndex]++] = a;
          break;
        }
      default:  //Data load and Checksum
        {
          if (packetBuffer[stateIndex] > 4) {
            int length = packetBuffer[4] + 6;
            if ((packetBuffer[stateIndex]) < length) {
              packetBuffer[packetBuffer[stateIndex]++] = a;
              break;
            } else {
              parsePacket((uint8_t*)packetBuffer, length, IPAddress(0, 0, 0, 0));

              //clear out the current pgn
              packetBuffer[stateIndex] = 0;
              return;
            }
          }
          break;
        }
    }
  }
#endif
}

void parsePacket(uint8_t* packet, int size, IPAddress IP) {

  if (packet[0] == 128 && packet[1] == 129) {
    int lenght = packet[4] + 6;
    if (lenght != size) {
      Serial.print("Packet: lenght error: ");
      Serial.println(size);
      printLnByteArray(packet, lenght);
      return;
    }

    byte CK_A = 0;
    for (int j = 2; j < lenght - 1; j++) {
      CK_A += packet[j];
    }

    if (packet[lenght - 1] != (byte)CK_A) {
      Serial.println("Packet: CRC error: ");
      Serial.print(CK_A);
      printLnByteArray(packet, lenght);
      return;
    }
  }

  if (packet[0] == 0x80 && packet[1] == 0x81 && packet[2] == 0x7F)  //Data
  {
    switch (packet[3]) {
      case 0xFE:
        {
          gpsSpeed = ((float)(packet[5] | packet[6] << 8)) * 0.1;

          prevGuidanceStatus = guidanceStatus;

          guidanceStatus = packet[7];
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)(packet[8] | ((int8_t)packet[9]) << 8)) * 0.01;  //high low bytes

          if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1)) {
            steerEnable = false;  //turn off steering motor
          } else                  //valid conditions to turn on autosteer
          {
            steerEnable = true;  //reset watchdog
          }
          //Bit 10 Tram
          tram = packet[10];
          //Bit 11
          relay = packet[11];
          //Bit 12
          relayHi = packet[12];
          //----------------------------------------------------------------------------
          //Serial Send to agopenGPS

          int16_t sa = (int16_t)(steerAngleActual * 100);

          PGN_253[5] = (uint8_t)sa;
          PGN_253[6] = sa >> 8;

          PGN_253[11] = switchByte;
          PGN_253[12] = (uint8_t)pwmDisplay;

          sendToClient(PGN_253, sizeof(PGN_253));

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
            if (aog2Count++ > 2) {
              //Send fromAutosteer2
              PGN_250[5] = (byte)sensorReading;

              sendToClient(PGN_250, sizeof(PGN_250));
              aog2Count = 0;
            }
          }
          break;
        }
      //steer settings
      case 252:
        {  //0xFC
          //PID values
          steerSettings.Kp = ((float)packet[5]);    // read Kp from AgOpenGPS
          steerSettings.highPWM = packet[6];        // read high pwm
          steerSettings.lowPWM = (float)packet[7];  // read lowPWM from AgOpenGPS
          steerSettings.minPWM = packet[8];         //read the minimum amount of PWM for instant on
          float temp = (float)steerSettings.minPWM * 1.2;
          steerSettings.lowPWM = (byte)temp;
          steerSettings.steerSensorCounts = packet[9];   //sent as setting displayed in AOG
          steerSettings.wasOffset = (packet[10]);        //read was zero offset Lo
          steerSettings.wasOffset |= (packet[11] << 8);  //read was zero offset Hi
          steerSettings.AckermanFix = (float)packet[12] * 0.01;

          //crc
          //autoSteerUdpData[13];

          //store in EEPROM
          EEPROM.put(10, steerSettings);
          EEPROM.commit();
          // Re-Init steer settings
          steerSettingsInit();
          break;
        }
      case 251:  //251 FB - SteerConfig
        {
          uint8_t sett = packet[5];  //setting0

          if (bitRead(sett, 0)) steerConfig.InvertWAS = 1;
          else steerConfig.InvertWAS = 0;
          if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1;
          else steerConfig.IsRelayActiveHigh = 0;
          if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1;
          else steerConfig.MotorDriveDirection = 0;
          if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1;
          else steerConfig.SingleInputWAS = 0;
          if (bitRead(sett, 4)) steerConfig.CytronDriver = 1;
          else steerConfig.CytronDriver = 0;
          if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1;
          else steerConfig.SteerSwitch = 0;
          if (bitRead(sett, 6)) steerConfig.SteerButton = 1;
          else steerConfig.SteerButton = 0;
          if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1;
          else steerConfig.ShaftEncoder = 0;

          steerConfig.PulseCountMax = packet[6];

          //was speed
          //autoSteerUdpData[7];

          sett = packet[8];  //setting1 - Danfoss valve etc

          if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1;
          else steerConfig.IsDanfoss = 0;
          if (bitRead(sett, 1)) steerConfig.PressureSensor = 1;
          else steerConfig.PressureSensor = 0;
          if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1;
          else steerConfig.CurrentSensor = 0;
          if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1;
          else steerConfig.IsUseY_Axis = 0;

          //crc
          //autoSteerUdpData[13];

          EEPROM.put(40, steerConfig);
          EEPROM.commit();
          // Re-Init
        }
      case 200:
        {  // Hello from AgIO

          int16_t sa = (int16_t)(steerAngleActual * 100);

          helloFromAutoSteer[5] = (uint8_t)sa;
          helloFromAutoSteer[6] = sa >> 8;

          helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
          helloFromAutoSteer[8] = helloSteerPosition >> 8;
          helloFromAutoSteer[9] = switchByte;

          sendData(helloFromAutoSteer, sizeof(helloFromAutoSteer));
          if (useBNO08x) {
            sendData(helloFromIMU, sizeof(helloFromIMU));
          }
        }
      case 202:
        {
          //make really sure this is the reply pgn
          if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202) {
            uint8_t scanReply[] = { 128, 129, 126, 203, 7,
                                    0, 0, 0, 0,
                                    0, 0, 0, 23 };
#ifndef USB
            ipDes = IP;
            //hello from AgIO
            scanReply[5] = myip[0];
            scanReply[6] = myip[1];
            scanReply[7] = myip[2];
            scanReply[8] = myip[3];
            scanReply[9] = myip[0];
            scanReply[10] = myip[1];
            scanReply[11] = myip[2];
            uint8_t ipArray[] = {
              ipDes[0],
              ipDes[1],
              ipDes[2],
              ipDes[3],
            };
            //printLnByteArray(ipArray, 4);
            EEPROM.put(60, ipArray);
            EEPROM.commit();
#endif
            sendData(scanReply, sizeof(scanReply));
          }
        }
    }
  } else {
    Serial.print("Unknown packet!!! : ");
    printLnByteArray(packet, size);
  }
}

void sendData(uint8_t* data, uint8_t datalen) {

  int16_t CK_A = 0;
  for (uint8_t i = 2; i < datalen - 1; i++) {
    CK_A = (CK_A + data[i]);
  }
  data[datalen - 1] = CK_A;
#ifdef USB
  Serial.write(data, datalen);
#else
  udp.beginPacket(ipDes, portDestination);
  udp.write(data, datalen);
  udp.endPacket();
#endif
}

void sendToClient(uint8_t* data, uint8_t datalen) {
  int16_t CK_A = 0;
  for (uint8_t i = 2; i < datalen - 1; i++) {
    CK_A = (CK_A + data[i]);
  }
  data[datalen - 1] = CK_A;
#ifdef USB
  Serial.write(data, datalen);
#else
  if (client.connected()) {
    client.write(data, datalen);
  } else {
    //Serial.println("Packet send error.....");
  }
#endif
}
