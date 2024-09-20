#include <Adafruit_NeoPixel.h>
#include <config.h>

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void updateLED(Color color) {
    pixels.setPixelColor(0, pixels.Color(color.r, color.g, color.b));
    pixels.show();
}

void setupLED() {
    pixels.begin();
    updateLED(WHITE);
}