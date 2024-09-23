#include <config.h>
#include <Logging.h>
#include <SdFat.h>

extern Data data;
String filename;

void loggingHandler::setup() {
    // SD setup
    if(!sd.begin(SD_CONFIG)){
        Serial.println("SD Card not found");
        while(!sd.begin(SD_CONFIG)){
            delay(100);
        }
    }

    // CSV setup
    int version = 0;
    filename = String(data.utcTime.formatted()) + ".csv";
    while(sd.exists(filename)){
        version = version + 1;
        filename = String(data.utcTime.formatted()) + "(" + String(version) + ")" + ".csv";
    }
    csvFile.open(filename.c_str(), FILE_WRITE);
    csvFile.print(
        F("time_ms,time_utc,imu_timestamp,gps_timestamp,acceleration_x,acceleration_y,acceleration_z,angular_velocity_x,angular_velocity_y,angular_velocity_z,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,gravity_x,gravity_y,gravity_z,orientation_i,orientation_j,orientation_k,orientation_real,longitude,latitude,altitude,speed,course,hdop\r\n"));
    csvFile.close(); 
}

void loggingHandler::csvLog() {
    digitalWrite(LED_BUILTIN, HIGH); // Builtin LED indicates that data is being written to the SD card
    csvFile.open(filename.c_str(), FILE_WRITE);
    csvFile.print(millis());
    csvFile.print(data.utcTime.csvLog());
    csvFile.print("," + String(data.imuTimestamp));
    csvFile.print("," + String(data.gpsTimestamp));
    csvFile.print(data.acceleration.csvLog());
    csvFile.print(data.angular_velocity.csvLog());
    csvFile.print(data.linear_acceleration.csvLog());
    csvFile.print(data.gravity.csvLog());
    csvFile.print(data.magnet.csvLog());
    csvFile.print(data.orientation.csvLog());
    csvFile.print("," + String(data.latitude));
    csvFile.print("," + String(data.longitude));
    csvFile.print("," + String(data.altitude));
    csvFile.print("," + String(data.speed));
    csvFile.print("," + String(data.course));
    csvFile.print("," + String(data.hdop));
    csvFile.print("\r");
    csvFile.close(); 
}

void loggingHandler::serialLog() {
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
    Serial.println("IMU Timestamp: " + String(data.imuTimestamp));
    Serial.println("GPS Timestamp: " + String(data.gpsTimestamp));
}