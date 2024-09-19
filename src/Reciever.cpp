#include <sbus.h>
#include <config.h>

void setupSbus() {
    sbus_rx.Begin();
    sbus_tx.Begin();
}

void readSbus() {
    if (sbus_rx.Read()) {
        recieverData = sbus_rx.data();
        data.aileron.raw = recieverData.ch[AILERON_CH];
        data.elevator.raw = recieverData.ch[ELEVATOR_CH];
        data.rudder.raw = recieverData.ch[RUDDER_CH];
        data.throttle.raw = recieverData.ch[THROTTLE_CH];
        data.aux.raw = recieverData.ch[AUX_CH];
        data.logging.raw = recieverData.ch[LOG_CH];
    }
    /* Set the SBUS TX data to the received data */
    sbus_tx.data(recieverData);
    /* Write the data to the servos */
    sbus_tx.Write();
}

void checkSbus() {

}

void mapSbus() {
    
}