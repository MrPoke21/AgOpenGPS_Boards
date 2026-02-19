
#include <Arduino.h>
#include <Configuration.h>
#include <zPackets.h>
#include <zUDP.h>
#include <main.h>

// Buffer size definitions
#define BUFFER_SIZE 1024
#define MAX_PACKET_SIZE 256
#define MAX_PACKET_DATA_SIZE (MAX_PACKET_SIZE - 6)  // Header + checksum

// Forward declaration
void processPacketBytes(byte* dataBuffer, uint16_t dataLen);

//uint8_t data[128];
byte buffer[BUFFER_SIZE];
byte packetBuffer[MAX_PACKET_SIZE];
byte ntripData[BUFFER_SIZE];
int stateIndex = 0;
int totalHeaderByteCount = 5;
int count;
//Heart beat hello AgIO
byte helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
byte helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
byte PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//fromAutoSteerData FA 250 - sensor values etc
byte PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//Scan reply
byte scanReply[] = { 128, 129, 126, 203, 7, 0, 0, 0, 0, 0, 0, 0, 23 };



void autoSteerPacketPerser() {
#if ENABLE_UDP == 1
  // Process UDP data only
  uint16_t udpLen = receiveUDP((uint8_t*)buffer, BUFFER_SIZE);
  if (udpLen > 0) {
    DEBUG_PRINTF("[PKT] Received %d bytes from UDP\n", udpLen);
    processPacketBytes(buffer, udpLen);
  }
#else
  // Process Serial data only - non-blocking byte-by-byte read
  int aas = Serial.available();
  if (aas > 0) {
    int readCount = (aas > 64) ? 64 : aas;
    DEBUG_PRINT("[PKT] Received ");
    DEBUG_PRINT(readCount);
    DEBUG_PRINTLN(" bytes from Serial");
    for (int i = 0; i < readCount; i++) {
      byte b = Serial.read();
      buffer[i] = b;
    }
    processPacketBytes(buffer, readCount);
  }
#endif
}

// Helper function to process byte stream from either Serial or UDP
void processPacketBytes(byte* dataBuffer, uint16_t dataLen) {
  byte a;
  for (int i = 0; i < dataLen; i++) {
    a = dataBuffer[i];
    
    // Prevent buffer overflow
    if (stateIndex >= MAX_PACKET_SIZE - 1) {
      DEBUG_PRINTLN("ERROR: Packet buffer overflow");
      stateIndex = 0;
      continue;
    }

    switch (stateIndex) {
      case 0:  //find 0x80
        {
          if (a == 128) packetBuffer[stateIndex++] = a;
          else stateIndex = 0;
          break;
        }

      case 1:  //find 0x81
        {
          if (a == 129) packetBuffer[stateIndex++] = a;
          else {
            if (a == 181) {
              stateIndex = 0;
              packetBuffer[stateIndex++] = a;
            } else stateIndex = 0;
          }
          break;
        }
      case 2:  //Source Address (7F)
        {
          if (a < 128 && a > 120)
            packetBuffer[stateIndex++] = a;
          else stateIndex = 0;
          break;
        }
      case 3:  //PGN ID
      case 4:  //Num of data bytes
        {
          packetBuffer[stateIndex++] = a;
          break;
        }
      default:  //Data load and Checksum
        {
          if (stateIndex > 4) {
            int length = packetBuffer[4] + 6;
            packetBuffer[stateIndex++] = a;
            if (stateIndex < length) {
              break;
            } else {
              DEBUG_PRINT("[PKT] Complete packet: length=");
              DEBUG_PRINT(length);
              DEBUG_PRINT(", PGN=0x");
              DEBUG_PRINTLN(packetBuffer[3], HEX);
              parsePacket(packetBuffer, length);
              //clear out the current pgn
              stateIndex = 0;
              break;
            }
          }
          break;
        }
    }
  }
}

void parsePacket(byte* packet, int size) {
  // Validate packet size
  if (size < 6 || size > MAX_PACKET_SIZE || packet == NULL) {
    DEBUG_PRINT("[PKT] ERROR: Invalid packet size: ");
    DEBUG_PRINTLN(size);
    return;
  }
  
  DEBUG_PRINT("[PKT] Parsing packet: size=");
  DEBUG_PRINT(size);
  DEBUG_PRINT(", PGN=0x");
  DEBUG_PRINTLN((size > 3) ? packet[3] : 0, HEX);
  
  if (packet[0] == 128 && packet[1] == 129) {
    int length = packet[4] + 6;
    
    // Bounds check
    if (length < 6 || length > MAX_PACKET_SIZE || length != size) {
      DEBUG_PRINT("ERROR: Packet length mismatch: expected ");
      DEBUG_PRINT(length);
      DEBUG_PRINT(" got ");
      DEBUG_PRINTLN(size);
      return;
    }

    byte CK_A = 0;
    for (int j = 2; j < length - 1; j++) {
      CK_A += packet[j];
    }

    if (packet[length - 1] != (byte)CK_A) {
      DEBUG_PRINT("ERROR: Checksum mismatch. Expected: 0x");
      DEBUG_PRINT(CK_A, HEX);
      DEBUG_PRINT(" Got: 0x");
      DEBUG_PRINTLN(packet[length - 1], HEX);
      return;
    }
  }

  if (packet[0] == 0x80 && packet[1] == 0x81 && packet[2] == 0x7F)  //Data
  {
    int packetLength = packet[4] + 6;
    
    switch (packet[3]) {
      case 0xFE:
        {
          // Validate packet has minimum required length for 0xFE command
          if (packetLength < 13) {
            Serial.println("ERROR: 0xFE packet too short");
            break;
          }
          
          gpsSpeed = ((float)(packet[5] | packet[6] << 8)) * 0.1;

          prevGuidanceStatus = guidanceStatus;
          guidanceStatus = packet[7];
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)(packet[8] | ((int8_t)packet[9]) << 8)) * 0.01;  //high low bytes

          byte guidanceBit = bitRead(guidanceStatus, 0);
          
          steerEnable = (guidanceBit != 0);
          
          // Only update steerEnable if the condition actually changed (prevent rapid toggling)
          if (steerEnable != prevSteerEnableCondition) {
            prevSteerEnableCondition = steerEnable;
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

          sendData(PGN_253, sizeof(PGN_253));
          DEBUG_PRINTLN("[PKT] Response sent: PGN_253 (0xFD)");

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
            if (aog2Count++ > 2) {
              //Send fromAutosteer2
              PGN_250[5] = (byte)sensorReading;

              sendData(PGN_250, sizeof(PGN_250));
              DEBUG_PRINTLN("[PKT] Response sent: PGN_250 (0xFA)");
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
          break;
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
          
          // Send IMU hello immediately after (no delay)
          if (useBNO08x) {
            sendData(helloFromIMU, sizeof(helloFromIMU));
          }
          break;
        }
      case 202:
        {
          //make really sure this is the reply pgn
          if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202) {
#if ENABLE_UDP
            // Fill scanReply with local and remote IP addresses
            IPAddress myIP;
            
            // Get local IP address based on WiFi mode
#if WIFI_MODE == 1
            myIP = WiFi.softAPIP();  // AP mode
#else
            myIP = WiFi.localIP();   // STA mode
#endif
            
            // Fill local IP bytes (5-8)
            scanReply[5] = myIP[0];
            scanReply[6] = myIP[1];
            scanReply[7] = myIP[2];
            scanReply[8] = myIP[3];
            
            // Fill remote IP bytes (9-11 - first 3 octets)
            scanReply[9] = udpRemoteIP[0];
            scanReply[10] = udpRemoteIP[1];
            scanReply[11] = udpRemoteIP[2];
#endif
            
            sendData(scanReply, sizeof(scanReply));
            DEBUG_PRINTLN("[PKT] Response sent: scanReply (0xCB)");
          }
          break;
        }
      //NTRIP DATA
      case 215:
        {
          int len = packet[4];
          if (len > 1) {
            for (int i = 0; i < len; i++) {
              ntripData[i] = packet[i+5];
            }
            Serial2.write(ntripData, len);
          }
          break;
        }
    }
  } else {
    DEBUG_PRINT("Unknown packet!!! : ");
    printLnByteArray(packet, size);
  }
}
