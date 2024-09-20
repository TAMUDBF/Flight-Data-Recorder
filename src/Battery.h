#ifndef BATTERY_H
#define BATTERY_H

class batteryHandler {
private:
    #define BATTERY_ADC_PIN 27
public:
    void setup();
    void read();
    boolean working();
};

#endif