#ifndef LEDMANAGER_H
#define LEDMANAGER_H
#include <config.h>

class ledHandler {
private:
    #ifdef __AVR__
    #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
    #endif
    #define PIN 23 // On Trinket or Gemma, suggest changing this to 1
    #define NUMPIXELS 1 
    Adafruit_NeoPixel* pixels;
    #define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
public:
    void setup();
    void setColor(Color color);
};

#endif