#ifndef SENSORS_H
#define SENSORS_H
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

class imuHandler {
private:
    #define BNO08X_CS 10
    #define BNO08X_INT 9
    #define BNO08X_RESET -1
    Adafruit_BNO08x* bno08x;
    sh2_SensorValue_t sensorValue;
public:
    void setup();
    void read();
    boolean working();
};

class gpsHandler {
private:
    static const int RXPin = 15, TXPin = 14;
    static const uint32_t GPSBaud = 38400;
    TinyGPSPlus gps;
    SoftwareSerial* ss;
public:
    void setup();
    void read();
    boolean working();
};

// Pitot is a WIP
class pitotHandler {
private:
public:
    void setup();
    void read();
    boolean working();
};

#endif