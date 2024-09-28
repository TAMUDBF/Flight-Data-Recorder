#include <config.h>
#include <Battery.h>

void batteryHandler::setup() {
    pinMode(BATTERY_ADC_PIN, INPUT);
}

void batteryHandler::read(Data data) {
    data.batteryVoltage.raw = analogRead(BATTERY_ADC_PIN);
}

boolean batteryHandler::working() {

}