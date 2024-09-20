#include <Adafruit_NeoPixel.h>
#include <config.h>
#include <LEDManager.h>

void ledHandler::setColor(Color color) {
    pixels->setPixelColor(0, pixels->Color(color.r, color.g, color.b));
    pixels->show();
}

void ledHandler::setup() {
    pixels = new Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
    pixels->begin();
    pixels->setPixelColor(0, pixels->Color(255, 255, 255)); // White
}