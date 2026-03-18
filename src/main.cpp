#include <TimeLib.h>
#include <Logging.h>
#include <Sensors.h>
#include <LEDManager.h>
#include <Reciever.h>
#include <Battery.h>
#include <config.h>

Data data;
gpsHandler GPS;
loggingHandler logger;

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // Builtin LED
  Teensy3Clock.set(0);
  GPS.setup();
  logger.setup();
}

void loop() {
  delay(10); // read rate
  GPS.read();
  logger.csvLog();
  logger.serialLog();
}
