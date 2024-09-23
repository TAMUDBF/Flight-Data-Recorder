#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <SdFat.h>
#include <sbus.h>
#include <SoftwareSerial.h>
#include <Logging.h>


// Initialize data types
struct Vector {
    float x, y, z;
    void serialLog(String label){
        Serial.print(label + ": (" + x + ", " + y + ", " + z + ")\n");
    }
    String csvLog(){
        return String("," + String(x) + "," + String(y) + "," + String(z));
    }
};

struct Quaternion {
    float i, j, k, real;
    void serialLog(String label){
        Serial.print(label + ": (" + i + ", " + j + ", " + k + ", " + real + ")\n");
    }
    String csvLog(){
        return ("," + String(i) + "," + String(j) + "," + String(k) + "," + String(real));
    }
};

struct Time {
    int second, minute, hour;
    void serialLog(String label){
        Serial.print(label + ": " + second + ":" + minute + ":" + hour + "\n");
    }
    String csvLog(){
        return String("," + String(second) + ":" + String(minute) + ":" + String(hour) + "\n");
    }
    String formatted() {
        return String(second) + "." + String(minute) + "." + String(hour);
    }
};

// Initialize data structure
struct Data {
    Time utcTime;
    unsigned long imuTimestamp;
    unsigned long gpsTimestamp;
    Vector acceleration;
    Vector angular_velocity;
    Vector linear_acceleration;
    Vector gravity;
    Vector magnet;
    Quaternion orientation;
    float latitude;
    float longitude;
    float altitude;
    float speed;
    float course;
    float hdop;
};

#endif