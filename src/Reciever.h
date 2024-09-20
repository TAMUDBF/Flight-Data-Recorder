#ifndef RECIEVER_H
#define RECIEVER_H

class sbusHandler{
private:
    #define AILERON_CH 0
    #define ELEVATOR_CH 1
    #define THROTTLE_CH 3
    #define RUDDER_CH 4
    #define AUX_CH 5
    #define LOG_CH 6
    bfs::SbusRx* sbus_rx;
    bfs::SbusTx* sbus_tx;
    bfs::SbusData recieverData;
public:
    void setup();
    void read();
    void map();
    boolean working();
};

#endif