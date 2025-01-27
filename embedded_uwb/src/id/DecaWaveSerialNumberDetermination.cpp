/**
 * @file DecaWaveDistanceMeasurement.cpp
 */

#include "common/DecaWaveDistanceMeasurement.h"

#include <stdio.h>

#include "common/decaWaveModule.h"
#include "../decadriver/deca_device_api.h"
#include "../decadriver/deca_regs.h"
#include "common/defines_resp.h"
#include "../ros_lib/ros.h"
#include "../ros_lib/ros_gateway.h"
#include "../ros_lib/std_msgs/Bool.h"
#include "../ros_lib/std_msgs/Float64.h"
#include "../rodos/api/hal.h"
//#include "hal.h"

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
ROS_Topic<std_msgs::Float64> tag_ID(-1, "/uwb/tag_ID");
std_msgs::Float64 msg;

// RODOS - ROS Gateway
ROS_Gateway gw(&uart_usb, &nh);

int32_t nodeId = 0;

/**
 * Initzialize the thread
 *  */
void DecaWaveDistanceMeasurement::init() {
    ledb.init(1, 1, 0);
    ledr.init(1, 1, 0);
    ledo.init(1, 1, 0);
    ledg.init(1, 1, 0);
    led.init(1, 1, 1);
    uart_usb.init(115200);
    gw.init();
    gw.addPublisher(&tag_ID); 
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

    ledg.setPins(1); // green side
    ledr.setPins(1);
    ledo.setPins(1);
    ledb.setPins(1); //blue led
    led.setPins(1);  //red led ?
    while (1){
        ledb.setPins(~ledb.readPins());
        suspendCallerUntil(NOW() + SECONDS);
        msg.data = getNodeNumber();
        tag_ID.publish(msg);
    }
}

DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread("Decawave Distance Measurement Node Thread");
