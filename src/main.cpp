#include <TimeLib.h>
#include <Logging.h>
#include <Sensors.h>
#include <config.h>

Data data;
sensorHandler sensors;
loggingHandler logger;

// Timing variables for IMU, GPS, and logging
unsigned long lastLogTime = 0;  // For logging
unsigned long logInterval = 10; // Logging interval in ms

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // Builtin LED
  Teensy3Clock.set(0);

  sensors.setup();
  logger.setup();
}

void loop() {
  unsigned long currentMillis = millis();

  sensors.read(); // continuously read

  // Check if it's time to log data
  if (currentMillis - lastLogTime >= logInterval) {
    lastLogTime = currentMillis;  // Update the last log time
    logger.csvLog();
    logger.serialLog();
  }
}
