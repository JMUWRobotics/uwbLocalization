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

Node *anchor;

void DecaWaveDistanceMeasurement::init(){
    ledb.init(1, 1, 0);
    ledr.init(1, 1, 0);
    ledo.init(1, 1, 0);
    ledg.init(1, 1, 0);
    led.init(1, 1, 1);
    anchor = new Node(getNodeNumber());
}

void DecaWaveDistanceMeasurement::run(){
    /***  INIT DECAWAVE ***/
    init_decaWaveModule(&config, antenna_delays[anchor->getId()]);
    dwt_setsmarttxpower(0); // Turns off the smart TX power feature
    dwt_write32bitreg(TX_POWER_ID, TX_POWER_ID_VAL); // Configuration and control of the transmitter output power

    nextTime2Action = NOW();

    while (1){
        ledb.setPins(~ledb.readPins());

        //if it is time for the next action preform it depending on the state of the node:
        if (nextTime2Action <= NOW()) {
            switch (anchor->getState()){
                // start ranging process:
                case NodeState::START_TWR:
                    start_twr();
                    anchor->toggleState();
                    nextTime2Action = NOW() + numberOffActiveNodes * respWaitTimePerNode;
                    break;
                // send final msg with those rsp timestamps that arrived:
                case NodeState::SEND_FINAL:
                    send_final();
                    anchor->toggleState();
                    nextTime2Action = NOW() + numberOffActiveNodes * respWaitTimePerNode;
                    break;
                // activate next node:
                case NodeState::ACTIVATE_NEXT_NODE:
                    anchor->toggleState();
                    send_activate(next_destId());
                    nextTime2Action = NOW() + 3 * SECONDS;
                    break;
                // do nothing:
                case NodeState::START:
                case NodeState::PAUSE:
                default:
                    nextTime2Action = NOW() + 1 * SECONDS;
                    break;
            }
        }

        // Enable receiver at DecaWave module
        uint16_t states_reg = dwt_read16bitoffsetreg(SYS_STATE_ID, 2);
        if (!(states_reg & 0x04)){ // check if receiver is not already enabled
            dwt_rxenable(0);
        }

        // Wait for data to be received + interrupt
        UWBirq.suspendUntilDataReady(nextTime2Action);
        UWBirq.resetInterruptEventStatus();
        receiveMessages();
    }
}

DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread("Decawave Distance Measurement Node Thread");

uint8_t DecaWaveDistanceMeasurement::next_destId(uint8_t offset/* = 0*/) {
    if (numberOffActiveNodes == 1){
        return anchor->getNodeIndex();
    }
    return (anchor->getNodeIndex() + 1 + offset) % numberOffActiveNodes;
}

void DecaWaveDistanceMeasurement::reset(){
    numberOffActiveNodes = 0;
    anchor->reset();
}

void DecaWaveDistanceMeasurement::send_dist(float distance, uint8_t destId, uint8_t twrSourceId){
    RODOS::memcpy(&tx_distance_msg[DISTANCE_MSG_DISTANCE_IDX], &distance, sizeof(distance));
    tx_distance_msg[DISTANCE_MSG_SOURCE_IDX] = twrSourceId;
    uwb_write(anchor->getNodeIndex(), destId, tx_distance_msg, sizeof(tx_distance_msg));
}

void DecaWaveDistanceMeasurement::send_state(uint8_t destId){
    tx_state_msg[STATE_MSG_STATE_IDX] = (uint8_t) anchor->getState();
    uwb_write(anchor->getNodeIndex(), destId, tx_state_msg, sizeof(tx_state_msg));
}

void DecaWaveDistanceMeasurement::send_activate(uint8_t destId, uint8_t counter /* = 0*/){
    if (destId == anchor->getNodeIndex()){
        anchor->toggleState();
        return;
    }
    tx_activate_msg[ACTIVATE_MSG_NUMNODES_IDX] = numberOffActiveNodes;
    uwb_write(anchor->getNodeIndex(), destId, tx_activate_msg, sizeof(tx_activate_msg));

    // Enable receiver at DecaWave module
    uint16_t states_reg = dwt_read16bitoffsetreg(SYS_STATE_ID, 2);
    if (!(states_reg & 0x04)){ // check if receiver is not already enabled
        dwt_rxenable(0);
    }

    // Wait for data to be received + interrupt
    UWBirq.suspendUntilDataReady(NOW() + 50 * MILLISECONDS);
    UWBirq.resetInterruptEventStatus();
    if (receiveMessages() == 0xFD){
        send_activate(next_destId(counter + 1), counter + 1);
    }
}

void DecaWaveDistanceMeasurement::send_reset(){
    uwb_write(anchor->getNodeIndex(), 255, tx_reset_msg, sizeof(tx_reset_msg));
}

/**
 * TWR step: T_SP
 * This function enables the twr process at the decawave module
 */
void DecaWaveDistanceMeasurement::start_twr(){
    uwb_write(anchor->getNodeIndex(), 255, tx_poll_msg, sizeof(tx_poll_msg));
    poll_tx_ts = get_tx_timestamp_u64();
    // clear resp_rx_ts array:
    for (uint8_t i = 0; i < MAX_ANCHORS + MAX_TAGS; i++){
        resp_rx_ts[i] = MAX_VALUE_64_BIT;
    }
}

/**
 * TWR step: T_SA
 */
void DecaWaveDistanceMeasurement::send_response(uint8_t destId){
    uwb_write(anchor->getNodeIndex(), destId, tx_resp_msg, sizeof(tx_resp_msg));
    resp_tx_ts = get_tx_timestamp_u64();
}

/**
 * TWR step: T_SF
 */
void DecaWaveDistanceMeasurement::send_final(){
    // compute final message transmission time
    uint64_t final_tx_time = get_systemtime_deca() + RESP_RX_TO_FINAL_TX_DLY_DTU_U64;
    dwt_setdelayedtrxtime(final_tx_time >> 8);
    // final TX timestamp is the transmission time we programmed plus the TX antenna delay
    uint64_t final_tx_ts = final_tx_time + antenna_delays[anchor->getId()];
    // write all timestamps in the final message
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
    for (uint8_t i = 0; i < numberOffActiveNodes; i++){
        final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX + i * FINAL_MSG_TS_LEN], resp_rx_ts[i]);
    }
    uwb_write(anchor->getNodeIndex(), 255, tx_final_msg, 10 + (2 + numberOffActiveNodes) * FINAL_MSG_TS_LEN, DWT_START_TX_DELAYED);
}

/**
 * This function estimates the distance based on the ToF
 */
float DecaWaveDistanceMeasurement::calculate_distance(){
    double dist;
    double Ra, Rb, Da, Db;
    double tof_dtu;

    // retrieve final reception timestamps.
    final_rx_ts = get_rx_timestamp_u64();

    // get timestamps embedded in the final message.
    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX + anchor->getNodeIndex() * FINAL_MSG_TS_LEN], &resp_rx_ts[anchor->getNodeIndex()]);

    // check if the resp msg. didn't arrive:
    if (resp_rx_ts[anchor->getNodeIndex()] == MAX_VALUE_64_BIT){
        return -1;
    }

    if (resp_rx_ts[anchor->getNodeIndex()] < poll_tx_ts)    {
        resp_rx_ts[anchor->getNodeIndex()] += MAX_VALUE_40_BIT;
        final_tx_ts += MAX_VALUE_40_BIT;
    }
    if (final_tx_ts < resp_rx_ts[anchor->getNodeIndex()]){
        final_tx_ts += MAX_VALUE_40_BIT;
    }
    if (resp_tx_ts < poll_rx_ts){
        resp_tx_ts += MAX_VALUE_40_BIT;
        final_rx_ts += MAX_VALUE_40_BIT;
    }
    if (final_rx_ts < resp_tx_ts){
        final_rx_ts += MAX_VALUE_40_BIT;
    }

    // compute time of flight.
    Ra = (double)(resp_rx_ts[anchor->getNodeIndex()] - poll_tx_ts);
    Rb = (double)(final_rx_ts - resp_tx_ts);
    Da = (double)(final_tx_ts - resp_rx_ts[anchor->getNodeIndex()]);
    Db = (double)(resp_tx_ts - poll_rx_ts);
    tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

    double tof = tof_dtu * DWT_TO_SECONDS;
    dist = tof * SPEED_OF_LIGHT;

    return (float)dist;
}

/**
 * This function is used to handle messages in the RX buffer
 * @return the msg type it reseaved, 0xFF if the msg wasn't for this node, 0xFE if it was a unknown msg for this node, 0xFD if there wasn't a msg to receive, 0xFC if the msg came from this node.
 */
uint8_t DecaWaveDistanceMeasurement::receiveMessages(){
    if (uwb_read(rx_buffer)){
        led.setPins(~led.readPins());
        uint8_t sourceId = rx_buffer[ALL_MSG_SOURCE_ID_IDX];
        uint8_t destId = rx_buffer[ALL_MSG_DEST_ID_IDX];
        uint8_t msgSN = rx_buffer[ALL_MSG_SN_IDX];

        if (sourceId != anchor->getNodeIndex()){ // only makes sence to read the msg if it isn't from itself
            switch (rx_buffer[ALL_MSG_TYPE_IDX]){
                case MSG_TYPE_POLL:{ // TWR poll message
                    poll_rx_ts = get_rx_timestamp_u64();
                    AT(NOW() + anchor->getNodeIndex() * respWaitTimePerNode);
                    send_response(sourceId);
                    return MSG_TYPE_POLL;
                }
                case MSG_TYPE_RESP:{ // TWR resp message
                    if (anchor->getNodeIndex() == destId){
                        resp_rx_ts[sourceId] = get_rx_timestamp_u64();
                        return MSG_TYPE_RESP;
                    } else {
                        return 0xFF;
                    }
                }
                case MSG_TYPE_FINAL:{ // TWR final message
                    float distance = calculate_distance();
                    AT(NOW() + anchor->getNodeIndex() * respWaitTimePerNode);
                    if (distance > 0){
                        send_dist(distance, 0, sourceId);
                    } 
                    return MSG_TYPE_FINAL;
                }
                case MSG_TYPE_DISTANCE:{ // Report (distance) message
                    if (anchor->getNodeIndex() == destId){
                        float distance;
                        RODOS::memcpy(&distance, &rx_buffer[DISTANCE_MSG_DISTANCE_IDX], sizeof(distance));
                        // this really should never happen, an anchor is not supposed to ever receive a distance msg.
                        // Therefore nothing is done with the received distance value.
                        return MSG_TYPE_DISTANCE;
                    } else {
                        return 0xFF;
                    }
                }
                case MSG_TYPE_STATE:{
                    if (anchor->getNodeIndex() == destId){
                        return MSG_TYPE_STATE;
                    } else {
                        return 0xFF;
                    }
                }
                case MSG_TYPE_ACTIVATE:{
                    if (anchor->getNodeIndex() == destId){
                        anchor->toggleState();
                        send_state(sourceId);
                        numberOffActiveNodes = rx_buffer[ACTIVATE_MSG_NUMNODES_IDX];
                        nextTime2Action = NOW();
                        return MSG_TYPE_ACTIVATE;
                    } else {
                        return 0xFF;
                    }
                }
                case MSG_TYPE_RESET:{
                    if (anchor->getNodeIndex() == destId){
                        reset();
                        return MSG_TYPE_RESET;
                    } else {
                        return 0xFF;
                    }
                }
                default:
                    return 0xFE;
            }
        } else {
            return 0xFC;
        }
    } else {
       return 0xFD; 
    }
}
