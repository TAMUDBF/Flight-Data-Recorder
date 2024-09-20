#include <config.h>
#include <Battery.h>

extern Data data;

void batteryHandler::setup() {
    pinMode(BATTERY_ADC_PIN, INPUT);
}

void batteryHandler::read() {
    data.batteryVoltage.raw = analogRead(BATTERY_ADC_PIN);
}

boolean batteryHandler::working() {

}