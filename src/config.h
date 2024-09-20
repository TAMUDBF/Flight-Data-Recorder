#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <SdFat.h>
#include <sbus.h>
#include <SoftwareSerial.h>
#include <Logging.h>

#define THROTTLE_MAX 1940
// ...etc. for all mapping values

struct Color {
    int r, g, b; 
    Color(int red, int green, int blue) : r(red), g(green), b(blue) {}
};

const Color RED(255, 0, 0);
const Color GREEN(0, 255, 0);
const Color BLUE(0, 0, 255);
const Color PURPLE(160, 32, 255);
const Color YELLOW(255, 140, 0);
const Color WHITE(255, 255, 255);
// ...etc. for colors used by status LED


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

struct ADC {
    int raw, mapped;
    void map(){

    }
    String csvLog(){
        return String("," + String(raw));
    }
};


// Initialize data structure
struct Data {
    Time utcTime;
    Vector acceleration;
    Vector angular_velocity;
    Vector linear_acceleration;
    Vector gravity;
    Vector magnet;
    Quaternion orientation;
    unsigned long imuTimestamp;
    float latitude;
    float longitude;
    float altitude;
    float speed;
    float course;
    float hdop;
    ADC aileron;
    ADC elevator;
    ADC rudder;
    ADC throttle;
    ADC aux;
    ADC logging;
    ADC batteryVoltage;
    //...etc. for all data collected
};

#endif