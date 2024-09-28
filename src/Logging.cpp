#include <LEDManager.h>
#include <Logging.h>
#include <SdFat.h>

extern ledHandler LED;
String filename;

void loggingHandler::setup(Data data) {
    // SD setup
    if(!sd.begin(SD_CONFIG)){
        Serial.println("SD Card not found");
        while(!sd.begin(SD_CONFIG)){
            LED.setColor(YELLOW);
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
        F("time_ms,time_utc,acceleration_x,acceleration_y,acceleration_z,acceleration_t,angular_velocity_x,angular_velocity_y,angular_velocity_z,angular_velocity_t,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,linear_acceleration_t,gravity_x,gravity_y,gravity_z,gravity_t,orientation_i,orientation_j,orientation_k,orientation_real,orientation_t,longitude,latitude,altitude,speed,course,hdop,throttle,elevator,aileron,rudder,throttle_raw,elevator_raw,aileron_raw,rudder_raw,battery_voltage,aux\r\n"));
    csvFile.close(); 
}

void loggingHandler::csvLog(Data data) {
    digitalWrite(LED_BUILTIN, HIGH); // Builtin LED indicates that data is being written to the SD card
    csvFile.open(filename.c_str(), FILE_WRITE);
    csvFile.print(millis());
    csvFile.print(data.utcTime.csvLog());
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
    csvFile.print(data.throttle.csvLog());
    csvFile.print(data.aileron.csvLog());
    csvFile.print(data.elevator.csvLog());
    csvFile.print(data.rudder.csvLog());
    csvFile.print(data.batteryVoltage.csvLog());
    csvFile.print(data.aux.csvLog());
    csvFile.print("\r");
    csvFile.close(); 
}

void loggingHandler::serialLog(Data data) {
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