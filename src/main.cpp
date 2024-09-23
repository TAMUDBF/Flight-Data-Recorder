#include <TimeLib.h>
#include <Logging.h>
#include <Sensors.h>
#include <config.h>

Data data;
imuHandler IMU;
gpsHandler GPS;
loggingHandler logger;

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // Builtin LED
  Teensy3Clock.set(0);

  IMU.setup();
  GPS.setup();
  logger.setup();
}

void loop() {
  delay(10); // read rate
  IMU.read();
  GPS.read();
  logger.csvLog();
  logger.serialLog();
}
