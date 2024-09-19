#ifndef CONFIG_H
#define CONFIG_H

#define BNO08X_CS 10
// ...etc. include all pins

#define IMU true
#define PITOT false
// ...etc. include booleans for if various peripherals will be used

#define THROTTLE_MAX 1940
// ...etc. for all ADC mapping values

struct Color {
    int r, g, b; 
    Color(int red, int green, int blue) : r(red), g(green), b(blue) {}
};

const Color RED(255, 0, 0);
// ...etc. for colors used by status LED


// initialize data structure
struct SensorData {
    float acceleration[3];
    float angular_velocity[3];
    //...etc. for all data collected
};

SensorData sensorData;

#endif