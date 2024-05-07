#include <Adafruit_BNO08x.h>


#define RAD_TO_DEG_X_10 57.295779513082320876798154814105
// Conversion to Hexidecimal
const char *asciiHex = "0123456789ABCDEF";

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

struct IMUVector {
  uint32_t time;
  float qr;
  float qi;
  float qj;
  float qk;
} imuVector;


// booleans to see if we are using BNO08x

uint8_t error;

Adafruit_BNO08x bno08x(-1);
// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;

sh2_SensorValue_t sensorValue;

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char speedKnots[10] = {};


void initHandler() {
  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
}
// If odd characters showed up.
void errorHandler() {
  //nothing at the moment
}

void GGA_Handler()  //Rec'd GGA
{
  // fix time
  parser.getArg(0, fixTime);

  // latitude
  parser.getArg(1, latitude);
  parser.getArg(2, latNS);

  // longitude
  parser.getArg(3, longitude);
  parser.getArg(4, lonEW);

  // fix quality
  parser.getArg(5, fixQuality);

  // satellite #
  parser.getArg(6, numSats);

  // HDOP
  parser.getArg(7, HDOP);

  // altitude
  parser.getArg(8, altitude);

  // time of last DGPS update
  parser.getArg(12, ageDGPS);

  GGA_Available = true;

  if (useBNO08x) {
    calculateIMU();
  }               //Get IMU data ready
  buildnmeaPGN();  //Build & send data GPS data to AgIO (Both Dual & Single)
}

void gpsStream() {
  while (Serial2.available()) {
    parser << Serial2.read();
  }
}

void readBNO(float qr, float qi, float qj, float qk) {
  imuVector.time = millis();
  imuVector.qr = qr;
  imuVector.qi = qi;
  imuVector.qj = qj;
  imuVector.qk = qk;
}

void calculateIMU() {
  quaternionToEuler(imuVector.qr, imuVector.qi, imuVector.qj, imuVector.qk);
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  if (steerConfig.IsUseY_Axis) {
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  } else {
    ypr.roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.pitch = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  }

  ypr.yaw *= -RAD_TO_DEG;
  if (ypr.yaw < 0) {
    ypr.yaw += 360;
  }
  ypr.pitch *= RAD_TO_DEG;
  ypr.roll *= RAD_TO_DEG;

  if (invertRoll) {
    ypr.roll *= -1;
  }
}

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable stabilized remote vector");
    return;
  }
}

void imuTask() {

  if (!useBNO08x) {
    return;
  }
  if (bno08x.wasReset()) {
    Serial.println("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

    case SH2_GAME_ROTATION_VECTOR:
      readBNO(sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
      break;
  }
}


void initIMU() {
  //set up communication
  Wire.begin();
  for (int16_t i = 0; i < nrBNO08xAdresses; i++) {
    bno08xAddress = bno08xAddresses[i];

    //Serial.print("\r\nChecking for BNO08X on ");
    //Serial.println(bno08xAddress, HEX);
    Wire.beginTransmission(bno08xAddress);
    error = Wire.endTransmission();

    if (error == 0) {
      //Serial.println("Error = 0");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X Ok.");
      // Initialize BNO080 lib
      if (bno08x.begin_I2C((int32_t)bno08xAddress))  //??? Passing NULL to non pointer argument, remove maybe ???
      {
        useBNO08x = true;
        break;
      } else {
        Serial.println("BNO080 not detected at given I2C address.");
      }
    } else {
      //Serial.println("Error = 4");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X not Connected or Found");
    }
  }
}

void buildnmeaPGN() {
  double lan = convertToDecimalDegrees(latitude, latNS);
  nmeaData.writeDouble(lan, 13);

  double lon = convertToDecimalDegrees(longitude, lonEW);

  nmeaData.writeDouble(lon, 5);
  float speed = String(speedKnots).toFloat();
  speed *= 1.852f;
  nmeaData.writeFloat(speed, 29);

  nmeaData.writeFloat(((String)altitude).toFloat(), 37);

  nmeaData.writeShort((short)(String(numSats).toInt()), 41);

  nmeaData.writeByte((byte)(String(fixQuality).toInt()), 43);

  short hdopx100 = (short)((String(HDOP).toFloat()) * 100);
  nmeaData.writeShort(hdopx100, 44);

  short ageDGPSx100 = (short)(String(ageDGPS).toFloat()) * 100;
  nmeaData.writeShort(ageDGPSx100, 46);

  nmeaData.writeFloat(ypr.yaw, 48);

  nmeaData.writeFloat(ypr.roll, 52);

  nmeaData.writeFloat(ypr.pitch, 56);

  //imuYawRate[
  //nmeaData.writeShort(0, 60);
  
  sendToClient(nmeaData.getBytes(), 63);
}

double convertToDecimalDegrees(const char *latLon, const char *direction) {
  char deg[4] = { 0 };
  char *dot, *min;
  int len;
  double dec = 0;

  if ((dot = strchr(latLon, '.'))) {   // decimal point was found
    min = dot - 2;                     // mark the start of minutes 2 chars back
    len = min - latLon;                // find the length of degrees
    strncpy(deg, latLon, len);         // copy the degree string to allow conversion to float
    dec = atof(deg) + atof(min) / 60;  // convert to float
    if (strcmp(direction, "S") == 0 || strcmp(direction, "W") == 0)
      dec *= -1;
  }
  return dec;
}

void VTG_Handler() {
  // vtg Speed knots
  parser.getArg(4, speedKnots);
}