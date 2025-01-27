/**
 * @file DecaWaveDistanceMeasurement.cpp
 */

#include "common/DecaWaveDistanceMeasurement.h"

// RODOS Config
extern HAL_GPIO UWBirq;
HAL_GPIO ledb(GPIO_060);
HAL_GPIO ledr(GPIO_061);
HAL_GPIO ledo(GPIO_062);
HAL_GPIO ledg(GPIO_063);
HAL_GPIO led(GPIO_078);
HAL_UART uart_usb(UART_IDX3);

// ROS Config
ros::NodeHandle nh;

// ROS Topics + RODOS SUbscribers:
ROS_Topic<uwb::UWBMsg> uwbMsgT(-1, "/uwb/uwbMsg");
uwb::UWBMsg uwbMsgMsg;

// RODOS - ROS Gateway
ROS_Gateway gw(&uart_usb, &nh);

/**
 * Initzialize the thread
 */
void DecaWaveDistanceMeasurement::init() {
    ledb.init(1, 1, 0);
    ledr.init(1, 1, 0);
    ledo.init(1, 1, 0);
    ledg.init(1, 1, 0);
    led.init(1, 1, 1);
    uart_usb.init(115200);
    gw.init();
    gw.addPublisher(&uwbMsgT);
}

/**
 * Main loop of the thread
 *
 * Initializes the decaWave module, sets the TX(transmition) power and starts
 * the main loop
 */
void DecaWaveDistanceMeasurement::run() {
    init_decaWaveModule(&config, 0);
    // set TX Power
    dwt_setsmarttxpower(0);
    dwt_write32bitreg(TX_POWER_ID, 0x009A9A00); // Configuration and control of the transmitter output power

    // main loop
    while (1) {
        ledb.setPins(~ledb.readPins());

        // Enable receiver at DecaWave module
        uint16_t states_reg = dwt_read16bitoffsetreg(SYS_STATE_ID, 2);
        if (!(states_reg & 0x04)){ // check if receiver is not already enabled
            dwt_rxenable(0);
        }

        // Wait for data to be received + interrupt
        UWBirq.suspendUntilDataReady(NOW() + SECONDS);
        UWBirq.resetInterruptEventStatus();
        receiveMessages();
    }
}

DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread("Decawave Distance Measurement Node Thread");

/**
 * This function is used to handle messages in the RX buffer
 */
uint8_t DecaWaveDistanceMeasurement::receiveMessages() {
    if (uwb_read(rx_buffer)) {
        led.setPins(~led.readPins());
        uint8_t sourceId = rx_buffer[ALL_MSG_SOURCE_ID_IDX];
        uint8_t destId = rx_buffer[ALL_MSG_DEST_ID_IDX];
        uint8_t msgSN = rx_buffer[ALL_MSG_SN_IDX];
        uint8_t msgType = rx_buffer[ALL_MSG_TYPE_IDX];

        uwbMsgMsg.msgType = msgType;
        uwbMsgMsg.sender = sourceId;
        uwbMsgMsg.receiver = destId;
        uwbMsgMsg.distance = 0;
        uwbMsgMsg.state = 0;

        switch (msgType) {
            case MSG_TYPE_DISTANCE: { // distance message
                float distance;
                RODOS::memcpy(&distance, &rx_buffer[DISTANCE_MSG_DISTANCE_IDX], sizeof(distance));
                uwbMsgMsg.distance = distance;
                break;
            }
            case MSG_TYPE_STATE: {
                uwbMsgMsg.state = rx_buffer[STATE_MSG_STATE_IDX];
                break;
            }
        }
        uwbMsgT.publish(uwbMsgMsg);
        return msgType;
    } else {
       return 0xFD; 
    }
}
