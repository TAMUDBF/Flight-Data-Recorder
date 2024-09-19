#include <TimeLib.h>
#include <Logging.h>
#include <Sensors.h>
#include <LEDManager.h>
#include <Reciever.h>
#include <Battery.h>
#include <config.h>

// Look at integrating this into GPS itself for more accurate delay?
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // Builtin LED
  Teensy3Clock.set(0);

  setupLED();
  setupIMU();
  setupGPS();
  setupPitot();
  setupSbus();
  setupBattery();
  setupSD();
  setupCSV();
}

void loop() {
  smartDelay(10);
  logToCSV();
  logToSerial();
}
