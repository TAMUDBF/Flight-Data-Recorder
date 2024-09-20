#include <config.h>
#include <LEDManager.h>

// IMU 
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_e imuSensors[] = {SH2_ACCELEROMETER,SH2_GYROSCOPE_CALIBRATED,SH2_LINEAR_ACCELERATION,SH2_GRAVITY,SH2_MAGNETIC_FIELD_CALIBRATED, SH2_ROTATION_VECTOR};

// GPS
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setupIMU() {
    if(!bno08x.begin_I2C()){
        while(!bno08x.begin_I2C()){
            Serial.println("IMU not found");
            updateLED(PURPLE);
            delay(100);
        }
    }
    for (sh2_SensorId_e sensorID : imuSensors){
        while(!bno08x.enableReport(sensorID)){
            Serial.print("Could not initialize sensor" + String(sensorID));
            updateLED(PURPLE);
            delay(100);
        }
    }
}

void readIMU() {
    bno08x.getSensorEvent(&sensorValue);
    switch (sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
            data.acceleration.x = sensorValue.un.accelerometer.x;
            data.acceleration.y = sensorValue.un.accelerometer.y;
            data.acceleration.z = sensorValue.un.accelerometer.z;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            data.angular_velocity.x = sensorValue.un.gyroscope.x;
            data.angular_velocity.y = sensorValue.un.gyroscope.y;
            data.angular_velocity.z = sensorValue.un.gyroscope.z;
            break;
        case SH2_LINEAR_ACCELERATION:
            data.linear_acceleration.x = sensorValue.un.linearAcceleration.x;
            data.linear_acceleration.y = sensorValue.un.linearAcceleration.y;
            data.linear_acceleration.z = sensorValue.un.linearAcceleration.z;
            break;
        case SH2_GRAVITY:
            data.gravity.x = sensorValue.un.gravity.x;
            data.gravity.y = sensorValue.un.gravity.y;
            data.gravity.z = sensorValue.un.gravity.z;
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            data.magnet.x = sensorValue.un.magneticField.x;
            data.magnet.y = sensorValue.un.magneticField.y;
            data.magnet.z = sensorValue.un.magneticField.z;
            break;

        case SH2_ROTATION_VECTOR:
            data.orientation.i = sensorValue.un.rotationVector.i;
            data.orientation.j = sensorValue.un.rotationVector.j;
            data.orientation.k = sensorValue.un.rotationVector.k;
            data.orientation.real = sensorValue.un.rotationVector.real;
            break;
    }
    data.imuTimestamp = sensorValue.timestamp;
}

void checkIMU() {
    if((data.gravity.x+data.gravity.y+data.gravity.z)==0){
        updateLED(PURPLE);
    }
}

void setupGPS() {
    ss.begin(GPSBaud);
}

void readGPS() {
    data.utcTime.second = gps.time.second();
    data.utcTime.minute = gps.time.minute();
    data.utcTime.hour = gps.time.hour();
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.feet();
    data.speed = gps.speed.mph();
    data.course = gps.course.deg();
    data.hdop = gps.hdop.hdop();
}

void checkGPS() {
    if(gps.satellites.value() < 4){
        updateLED(BLUE);
    }
    else {
        updateLED(GREEN);
    }
}

void setupPitot() {
    // Pitot not currently implemented, WIP
}

void readPitot() {
    
}