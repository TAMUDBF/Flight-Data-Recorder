#include <TimeLib.h>
#include <Logging.h>
#include <Sensors.h>
#include <LEDManager.h>
#include <Reciever.h>
#include <Battery.h>
#include <config.h>

Data data;
imuHandler IMU;
gpsHandler GPS;
pitotHandler pitotTube;
sbusHandler SBus;
batteryHandler battery;
ledHandler LED;
loggingHandler logger;

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // Builtin LED
  Teensy3Clock.set(0);

  LED.setup();
  IMU.setup(LED);
  GPS.setup();
  pitotTube.setup();
  SBus.setup();
  battery.setup();
  logger.setup(LED);
}

void loop() {
  delay(10); // read rate
  IMU.read();
  GPS.read();
  pitotTube.read();
  SBus.read();
  battery.read();
  logger.csvLog();
  logger.serialLog();
}
