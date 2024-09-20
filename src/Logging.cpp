#include <config.h>
#include <LEDManager.h>

String filename = "default";
SdFs sd;
FsFile file;
Data data;

void setupSD() {
    if(!sd.begin(SD_CONFIG)){
        Serial.println("SD Card not found");
        while(!sd.begin(SD_CONFIG)){
            updateLED(YELLOW);
            delay(100);
        }
    }
}

void checkSD() {
    // Should only need to be run once in main setup. Will indicate if no sd card present.
}

void setupCSV() {
    int version = 0;
    String filename = String(gps.time.hour()) + "." + String(gps.time.minute()) + "." + String(gps.time.second()) + ".csv";
    while(sd.exists(filename)){
    version = version + 1;
    filename = String(gps.time.hour()) + "." + String(gps.time.minute()) + "." + String(gps.time.second()) + "(" + String(version) + ")" + ".csv";
    }
    file.open(filename.c_str(), FILE_WRITE);
    file.print(
        F("time_ms,time_utc,acceleration_x,acceleration_y,acceleration_z,acceleration_t,angular_velocity_x,angular_velocity_y,angular_velocity_z,angular_velocity_t,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,linear_acceleration_t,gravity_x,gravity_y,gravity_z,gravity_t,orientation_i,orientation_j,orientation_k,orientation_real,orientation_t,longitude,latitude,altitude,speed,course,hdop,throttle,elevator,aileron,rudder,throttle_raw,elevator_raw,aileron_raw,rudder_raw,battery_voltage,aux\r\n"));
    file.close(); 
}

void logToCSV() {
    digitalWrite(LED_BUILTIN, HIGH); // Builtin LED indicates that data is being written to the SD card
    file.open(filename.c_str(), FILE_WRITE);
    file.print(millis());
    data.utcTime.csvLog();
    data.acceleration.csvLog();
    data.angular_velocity.csvLog();
    data.linear_acceleration.csvLog();
    data.gravity.csvLog();
    data.magnet.csvLog();
    data.orientation.csvLog();
    file.print("," + String(data.latitude));
    file.print("," + String(data.longitude));
    file.print("," + String(data.altitude));
    file.print("," + String(data.speed));
    file.print("," + String(data.course));
    file.print("," + String(data.hdop));
    data.throttle.csvLog();
    data.aileron.csvLog();
    data.elevator.csvLog();
    data.rudder.csvLog();
    data.batteryVoltage.csvLog();
    data.aux.csvLog();
    file.print("\r");
    file.close(); 
}

void logToSerial() {
    data.acceleration.serialLog("Acceleration");
    data.angular_velocity.serialLog("Angular Velocity");
    data.linear_acceleration.serialLog("Linear Acceleration");
    data.gravity.serialLog("Gravity");
    data.magnet.serialLog("Magnetometer");
    data.orientation.serialLog("Orientation");
    data.utcTime.serialLog("UTC Time");
    Serial.println("Latitude:" + String(data.latitude));
    Serial.println("Longitude:" + String(data.longitude));
    Serial.println("ms Elapsed: " + millis());
}