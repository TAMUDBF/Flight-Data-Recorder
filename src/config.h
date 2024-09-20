#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <Adafruit_NeoPixel.h>
#include <SdFat.h>
#include <sbus.h>
#include <SoftwareSerial.h>

// IMU Initialization
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern sh2_SensorId_e imuSensors[];

// GPS Initialization
static const int RXPin = 15, TXPin = 14;
static const uint32_t GPSBaud = 38400;
extern TinyGPSPlus gps;
extern SoftwareSerial ss;

// SBUS Initialization
#define AILERON_CH 0
#define ELEVATOR_CH 1
#define THROTTLE_CH 3
#define RUDDER_CH 4
#define AUX_CH 5
#define LOG_CH 6
extern bfs::SbusRx sbus_rx;
extern bfs::SbusTx sbus_tx;
extern bfs::SbusData recieverData;

// Battery Initialization
#define BATTERY_ADC_PIN 27

// LED Initialization
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define PIN 23 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 
extern Adafruit_NeoPixel pixels;
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

// SD Setup
extern String filename;
#define SD_FAT_TYPE 3
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else   // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
#define SPI_CLOCK SD_SCK_MHZ(50)
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
extern SdFs sd;
extern FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE


#define IMU true
#define PITOT false
// ...etc. include booleans for if various peripherals will be used

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
        Serial.print(label + ": (" + x + ", " + y + ", " + z + "\n");
    }
    void csvLog(){
        file.print("," + String(x) + "," + String(y) + "," + String(z));
    }
};

struct Quaternion {
    float i, j, k, real;
    void serialLog(String label){
        Serial.print(label + ": (" + i + ", " + j + ", " + k + ", " + real + "\n");
    }
    void csvLog(){
        file.print("," + String(i) + "," + String(j) + "," + String(k) + "," + String(real));
    }
};

struct Time {
    int second, minute, hour;
    void serialLog(String label){
        Serial.print(label + ": " + second + ":" + minute + ":" + hour + "\n");
    }
    void csvLog(){
        file.print("," + String(second) + ":" + String(minute) + ":" + String(hour) + "\n");
    }
};

struct ADC {
    int raw, mapped;
    void map(){

    }
    void csvLog(){
        file.print("," + String(raw));
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


extern Data data;

#endif