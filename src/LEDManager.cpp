#include <Adafruit_NeoPixel.h>
#include <config.h>

void setupLED() {
    pixels.begin();
    updateLED(WHITE);
}

void updateLED(Color color) {
    pixels.setPixelColor(0, pixels.Color(color.r, color.g, color.b));
    pixels.show();
}