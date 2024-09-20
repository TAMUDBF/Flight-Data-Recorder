#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>

void setupIMU();
void readIMU();
void checkIMU();
void setupGPS();
void readGPS();
void checkGPS();
void setupPitot();
void readPitot();
void checkPitot();

#endif