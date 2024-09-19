#include <config.h>

void setupBattery() {
    pinMode(BATTERY_ADC_PIN, INPUT);
}

void readBattery() {
    data.batteryVoltage.raw = analogRead(BATTERY_ADC_PIN);
}

void checkBattery() {

}