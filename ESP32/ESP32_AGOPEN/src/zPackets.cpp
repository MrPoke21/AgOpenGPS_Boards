
#include <Arduino.h>
#include <zPackets.h>
#include <main.h>
//uint8_t data[128];
byte buffer[1024];
byte packetBuffer[1024];
byte ntripData[1024];
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



void autoSteerPacketPerser() {
  int aas = Serial.available();
  if (aas < 1) {
    return;
  }

  Serial.readBytes(buffer, aas);
  byte a;
  for (int i = 0; i < aas; i++) {
    a = buffer[i];

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

          sendData(PGN_253, sizeof(PGN_253));

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
            if (aog2Count++ > 2) {
              //Send fromAutosteer2
              PGN_250[5] = (byte)sensorReading;

              sendData(PGN_250, sizeof(PGN_250));
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
          if (useBNO08x) {
            delay(5);
            sendData(helloFromIMU, sizeof(helloFromIMU));
          }
          break;
        }
      case 202:
        {
          //make really sure this is the reply pgn
          if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202) {
            byte scanReply[] = { 128, 129, 126, 203, 7,
                                 0, 0, 0, 0,
                                 0, 0, 0, 23 };
            sendData(scanReply, sizeof(scanReply));
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
    Serial.print("Unknown packet!!! : ");
    printLnByteArray(packet, size);
  }
}
