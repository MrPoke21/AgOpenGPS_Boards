#ifndef ZHANDLERS_H
#define ZHANDLERS_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "zNMEAParser.h"
#include "main.h"
#define RAD_TO_DEG_X_10 57.295779513082320876798154814105



struct IMUVector {
  uint32_t time;
  float qr;
  float qi;
  float qj;
  float qk;
};
extern IMUVector imuVector;


// booleans to see if we are using BNO08x

//extern uint8_t error;

// BNO08x address variables to check where it is
extern const uint8_t bno08xAddresses[2];
extern const int16_t nrBNO08xAdresses;
extern uint8_t bno08xAddress;

// the new PANDA sentence buffer
extern char nmea[100];

// GGA
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];

// VTG
extern char speedKnots[10];

double convertToDecimalDegrees(const char *latLon, const char *direction);
void quaternionToEuler(float qr, float qi, float qj, float qk);
void buildnmeaPGN();
void calculateIMU();

void errorHandler();
void GGA_Handler();
void VTG_Handler();

void initIMU();
void initHandler();
void imuTask();
void gpsStream();
#endif