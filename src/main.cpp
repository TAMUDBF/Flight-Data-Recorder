#include <TimeLib.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SdFat.h>
#include <Adafruit_NeoPixel.h>
#include <sbus.h>

#include <Logging.h>
#include <Sensors.h>
#include <Servos.h>
#include <LEDManager.h>
#include <Battery.h>
#include <config.h>

//Output Variables
String utcTime = "";
float acceleration[4];
float angular_velocity[4];
float linear_acceleration[4];
float gravity[4];
float magnet[4];
float orientation[5];
float latitude =0;
float longitude=0;
float altitude=0;
float speed=0;
float course=0;
float hdop=0;
float throttleMapped=0;
float aileronMapped=0;
float elevatorMapped=0;
float rudderMapped=0;
float batteryVoltage=0;
float batteryVoltageMapped=0;
boolean logging = false;
boolean aux = false;

//Servo Trims
float elevatorTrim=0;
float aileronTrim=0;
float rudderTrim=0;


//Battery Type 
int batteryCells = 0;

//SD Card Setup
String filename = "default";
#define SD_FAT_TYPE 3
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else   // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
#define SPI_CLOCK SD_SCK_MHZ(50)
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

// IMU Setup
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_e imuSensors[] = {SH2_ACCELEROMETER,SH2_GYROSCOPE_CALIBRATED,SH2_LINEAR_ACCELERATION,SH2_GRAVITY,SH2_MAGNETIC_FIELD_CALIBRATED, SH2_ROTATION_VECTOR};

//GPS Setup
static const int RXPin = 15, TXPin = 14;
static const uint32_t GPSBaud = 38400;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

//LED Setup
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define PIN 23 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

//Servo Setup 
const int throttle_pin = 2;
const int elevator_pin = 4;
const int alieron_pin = 3;
const int rudder_pin = 5;

long pwm_time = 0;
volatile int throttle = 0;
volatile int elevator = 0;
volatile int aileron = 0;
volatile int rudder = 0;

//SBUS Setup
bfs::SbusRx sbus_rx(&Serial6);
bfs::SbusTx sbus_tx(&Serial6);
bfs::SbusData data;

// Extra Switches
const int logging_pin = 6;
const int aux_pin = 7;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
void throttle_isr(){
  if(digitalRead(throttle_pin) == 1){
    pwm_time = micros();
  }
  else{
    throttle = int(micros()) - int(pwm_time);
  }
}

void elevator_isr(){
  elevator = int(micros()) - int(pwm_time);
}

void alieron_isr(){
  aileron = int(micros()) - int(pwm_time);
}

void rudder_isr(){
  rudder = int(micros()) - int(pwm_time);
}

void logging_isr(){
  int logging_raw = int(micros()) - int(pwm_time);
  if(logging_raw > 1700){logging = true;}
  else{logging = false;}
}

void aux_isr(){
  int aux_raw = int(micros()) - int(pwm_time);
  if(aux_raw > 1700){aux = true;}
  else{aux = false;}
}
*/

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void logRow(float data[2]){
  Serial.print("X: ");
  Serial.print(data[0], 5);
  Serial.print(" Y: ");
  Serial.print(data[1], 5);
  Serial.print(" Z: ");
  Serial.println(data[2], 5);
}

void logMonitor(){
  //Serial Monitor
  Serial.print("Acceleration        | ");
  logRow(acceleration);
  Serial.print("Angular Velocity    | ");
  logRow(angular_velocity);
  Serial.print("Linear Acceleration | ");
  logRow(linear_acceleration);
  Serial.print("Gravity             | ");
  logRow(gravity);
  Serial.print("Magnetometer        | ");
  logRow(magnet);
  Serial.print("Orientation         | ");
  logRow(orientation);
  Serial.print(" Real: ");
  Serial.println(orientation[3],5);
  Serial.print("UTC Time            | ");
  Serial.print(utcTime);
  Serial.println();
  Serial.print("Latitude            | ");
  Serial.print(gps.location.lat(),11);
  Serial.println();
  Serial.print("Longitude           | ");
  Serial.print(gps.location.lng(),12);
  Serial.println();
  Serial.print("Servo Rotations     | ");
  Serial.print(String(throttle) + ", " + String(elevator) + ", " + String(aileron) + ", " + String(rudder));
  Serial.println();
  Serial.print("Throttle            | ");
  Serial.print(String(throttleMapped));
  Serial.println();
  Serial.print("Elevator            | ");
  Serial.print(String(elevatorMapped));
  Serial.println();
  Serial.print("Aileron             | ");
  Serial.print(String(aileronMapped));
  Serial.println();
  Serial.print("Rudder              | ");
  Serial.print(String(rudderMapped));
  Serial.println();
  Serial.print("Battery Voltage     | ");
  Serial.print(String(batteryVoltageMapped));
  Serial.println();
  Serial.print("Battery Raw         | ");
  Serial.print(String(batteryVoltage));
  Serial.println();
  Serial.print("ms Elapsed          | ");
  Serial.print(millis());
  Serial.println();
  Serial.print("Auxiliary           | ");
  Serial.print(String(aux));
  Serial.println();
  Serial.print("Logging             | ");
  Serial.print(String(logging));
  Serial.println();
  Serial.print("Sensor Event        | ");
  Serial.print(String(sensorValue.sensorId));
  Serial.println();
  Serial.print("Sensor Status       | ");
  Serial.print(String(sensorValue.status));
  Serial.println();
}

void logCSV(){
  digitalWrite(LED_BUILTIN,HIGH);

  //SD Card
  file.open(filename.c_str(), FILE_WRITE);
  file.print(String(millis())+","+utcTime+","+String(acceleration[0])+","+String(acceleration[1])+","+String(acceleration[2])+","+String(acceleration[3])+","+
  String(angular_velocity[0])+","+String(angular_velocity[1])+","+String(angular_velocity[2])+","+String(angular_velocity[3])+","+
  String(linear_acceleration[0])+","+String(linear_acceleration[1])+","+String(linear_acceleration[2])+","+String(linear_acceleration[3])+","+
  String(gravity[0])+","+String(gravity[1])+","+String(gravity[2])+","+String(gravity[3])+","+String(magnet[0])+","+String(magnet[1])+","+String(magnet[2])+","+String(magnet[3])+","+
  String(orientation[0])+","+String(orientation[1])+","+String(orientation[2])+","+String(orientation[3])+","+String(orientation[4])+","+
  String(longitude,11)+","+String(latitude,12)+","+String(altitude)+","+String(speed)+","+String(course)+","+String(hdop)+","+
  String(throttleMapped)+","+String(elevatorMapped)+","+String(aileronMapped)+","+String(rudderMapped)+","+
  String(throttle)+","+String(elevator)+","+String(aileron)+","+String(rudder)+","+String(batteryVoltageMapped)+","+String(aux)+"\r");
  file.close(); 
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);
  Teensy3Clock.set(0);

  //LED Setup  
  pixels.begin();
  pixels.setPixelColor(0,pixels.Color(255, 255, 255));
  pixels.show(); // White

  //Battery Initialization
  pinMode(27,INPUT);
  batteryVoltage = analogRead(27);
  batteryVoltageMapped = map(batteryVoltage,0,786,0,22.9);
  if(batteryVoltageMapped > 18){batteryCells = 6;} 
  else{batteryCells = 4;}

  //IMU Initialization
  if(!bno08x.begin_I2C()){
    while(!bno08x.begin_I2C()){
      Serial.println("IMU not found");
      pixels.setPixelColor(0,pixels.Color(160, 32, 255)); 
      pixels.show(); // Purple
      delay(100);
    }
  }

  for (sh2_SensorId_e sensorID : imuSensors){
    while(!bno08x.enableReport(sensorID)){
      Serial.print("Could not initialize sensor" + String(sensorID));
      delay(10);
    }
  }

  //Servo Initialization
  sbus_rx.Begin();
  sbus_tx.Begin();
  /*
  attachInterrupt(digitalPinToInterrupt(throttle_pin), throttle_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(elevator_pin), elevator_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(alieron_pin), alieron_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(rudder_pin), rudder_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(logging_pin), logging_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(aux_pin), aux_isr, FALLING);
  */

  //Servo Trims
  /*
  delay(1000);
  elevatorTrim = elevator;
  aileronTrim = aileron;
  rudderTrim = rudder;
  */

  //GPS Initialization
  ss.begin(GPSBaud);

  //SD Initialization
  if(!sd.begin(SD_CONFIG)){
    Serial.println("SD Card not found");
    while(!sd.begin(SD_CONFIG)){
      pixels.setPixelColor(0,pixels.Color(255, 140, 0));
      pixels.show(); // Yellow
      delay(100);
    }
  }

  // File Creation
  int version = 0;
  filename = String(gps.time.hour()) + "." + String(gps.time.minute()) + "." + String(gps.time.second()) + ".csv";
  while(sd.exists(filename)){
    version = version + 1;
    filename = String(gps.time.hour()) + "." + String(gps.time.minute()) + "." + String(gps.time.second()) + "(" + String(version) + ")" + ".csv";
  }
  file.open(filename.c_str(), FILE_WRITE);
  file.print(
      F("time_ms,time_utc,acceleration_x,acceleration_y,acceleration_z,acceleration_t,angular_velocity_x,angular_velocity_y,angular_velocity_z,angular_velocity_t,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,linear_acceleration_t,gravity_x,gravity_y,gravity_z,gravity_t,orientation_i,orientation_j,orientation_k,orientation_real,orientation_t,longitude,latitude,altitude,speed,course,hdop,throttle,elevator,aileron,rudder,throttle_raw,elevator_raw,aileron_raw,rudder_raw,battery_voltage,aux\r\n"));
  file.close(); 
}


void loop() {
  smartDelay(10);
  //Read IMU
  bno08x.getSensorEvent(&sensorValue);
  switch (sensorValue.sensorId) {
  case SH2_ACCELEROMETER:
    acceleration[0] = sensorValue.un.accelerometer.x;
    acceleration[1] = sensorValue.un.accelerometer.y;
    acceleration[2] = sensorValue.un.accelerometer.z;
    acceleration[3] = sensorValue.timestamp;
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    angular_velocity[0] = sensorValue.un.gyroscope.x;
    angular_velocity[1] = sensorValue.un.gyroscope.y;
    angular_velocity[2] = sensorValue.un.gyroscope.z;
    angular_velocity[3] = sensorValue.timestamp;
    break;
  case SH2_LINEAR_ACCELERATION:
    linear_acceleration[0] = sensorValue.un.linearAcceleration.x;
    linear_acceleration[1] = sensorValue.un.linearAcceleration.y;
    linear_acceleration[2] = sensorValue.un.linearAcceleration.z;
    linear_acceleration[3] = sensorValue.timestamp;
    break;
  case SH2_GRAVITY:
    gravity[0] = sensorValue.un.gravity.x;
    gravity[1] = sensorValue.un.gravity.y;
    gravity[2] = sensorValue.un.gravity.z;
    gravity[3] = sensorValue.timestamp;
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    magnet[0] = sensorValue.un.magneticField.x;
    magnet[1] = sensorValue.un.magneticField.y;
    magnet[2] = sensorValue.un.magneticField.z;
    magnet[3] = sensorValue.timestamp;
    break;

  case SH2_ROTATION_VECTOR:
    orientation[0] = sensorValue.un.rotationVector.i;
    orientation[1] = sensorValue.un.rotationVector.j;
    orientation[2] = sensorValue.un.rotationVector.k;
    orientation[3] = sensorValue.un.rotationVector.real;
    orientation[4] = sensorValue.timestamp;
    break;
  }

  //Read GPS
  utcTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
  latitude = gps.location.lat();
  longitude = gps.location.lng();
  altitude = gps.altitude.feet();
  speed = gps.speed.mph();
  course = gps.course.deg();
  hdop = gps.hdop.hdop();

  //Read Servo Rotations
  /*
  throttleMapped = map(float(throttle),1100,1940,0,100);
  elevatorMapped = map(float(elevator),elevatorTrim,1783,0,100);
  aileronMapped = map(float(aileron),aileronTrim,1933,0,100);
  rudderMapped = map(float(rudder),rudderTrim,1940,0,100);
  */
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    aileron = data.ch[0];
    elevator = data.ch[1];
    throttle = data.ch[3];
    rudder = data.ch[4];
    aux = data.ch[5];
    logging = data.ch[6];
  }
  /* Set the SBUS TX data to the received data */
  sbus_tx.data(data);
  /* Write the data to the servos */
  sbus_tx.Write();

  //Read Battery Voltage
  batteryVoltage = analogRead(27);
  batteryVoltageMapped = map(batteryVoltage,0,786,0,22.9);

  // IMU Valid Check
  if((gravity[0]+gravity[1]+gravity[2])==0){
    pixels.setPixelColor(0,pixels.Color(160, 32, 255)); 
    pixels.show(); // Purple
  }
  else{
    // GPS Fix Check
    if(gps.satellites.value() < 4){
      pixels.setPixelColor(0,pixels.Color(0, 0, 255)); 
      pixels.show(); // Blue
    }
    else {
      pixels.setPixelColor(0,pixels.Color(0, 255, 0)); 
      pixels.show(); // Green
    }
  }

  //Low Battery Indicator
  // if(batteryVoltageMapped < 3.75*batteryCells){
  //   pixels.setPixelColor(0,pixels.Color(255, 0, 0)); 
  //   pixels.show(); // Red
  // }

  logMonitor();
  //if(logging){
  logCSV();
  //
}
