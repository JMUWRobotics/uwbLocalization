/**
 * @file decaWaveModule.h
*/

#ifndef DECA_WAVE_MODULE_H_
#define DECA_WAVE_MODULE_H_

#include "../../decadriver/deca_device_api.h"
#include "rodos.h"
#include "stdint.h"

#define MAX_ANCHORS 12
#define MAX_TAGS 4

/* Index to access some of the fields in the frames involved in the process. */
// all messages
constexpr uint8_t ALL_MSG_SN_IDX = 2;         // sequence number
constexpr uint8_t ALL_MSG_SOURCE_ID_IDX = 5;  // source id
constexpr uint8_t ALL_MSG_DEST_ID_IDX = 6;    // destination id
constexpr uint8_t ALL_MSG_TYPE_IDX = 7;       // message type
constexpr uint8_t ALL_MSG_DEFAULT_LENGTH = 10;

// final message
constexpr uint8_t FINAL_MSG_TS_LEN = 5;                                                     // Number of bytes of a timestamp in the message
constexpr uint8_t FINAL_MSG_POLL_TX_TS_IDX = 8;                                             // poll_tx timestamp
constexpr uint8_t FINAL_MSG_FINAL_TX_TS_IDX = FINAL_MSG_POLL_TX_TS_IDX + FINAL_MSG_TS_LEN;  // final_tx timestamp
constexpr uint8_t FINAL_MSG_RESP_RX_TS_IDX = FINAL_MSG_FINAL_TX_TS_IDX + FINAL_MSG_TS_LEN;  // resp_rx timestamp

// distance message
constexpr uint8_t DISTANCE_MSG_FLOAT_LEN = 4; // Number of bytes in a float in the msg
constexpr uint8_t DISTANCE_MSG_DISTANCE_IDX = 8;  // distance
constexpr uint8_t DISTANCE_MSG_SOURCE_IDX = DISTANCE_MSG_DISTANCE_IDX + DISTANCE_MSG_FLOAT_LEN;  // source of the twr process

// activate message
constexpr uint8_t ACTIVATE_MSG_NUMNODES_IDX = 8;  // num of active nodes

// state message
constexpr uint8_t STATE_MSG_STATE_IDX = 8;  // state

// calibrationTimes message
constexpr uint8_t CALIBRATIONTIMES_MSG_UINT64_LEN = 8; // Number of bytes in a uint64_t in the msg
constexpr uint8_t CALIBRATIONTIMES_MSG_TIME_IDX = 8;  // time_diff

// timeDiff message
constexpr uint8_t TIMEDIFF_MSG_UINT64_LEN = 8; // Number of bytes in a float in the msg
constexpr uint8_t TIMEDIFF_MSG_TIMEDIFF_IDX = 8;  // timeDiffs
constexpr uint8_t TIMEDIFF_MSG_SOURCEID_IDX = 8 + 4 * TIMEDIFF_MSG_UINT64_LEN;  // sourceId

/* Message type identifiers. */
constexpr uint8_t MSG_TYPE_POLL = 0x20;             // 32
constexpr uint8_t MSG_TYPE_RESP = 0x21;             // 33
constexpr uint8_t MSG_TYPE_FINAL = 0x22;            // 34
constexpr uint8_t MSG_TYPE_DISTANCE = 0x23;         // 35
constexpr uint8_t MSG_TYPE_ACTIVATE = 0x24;           // 36
constexpr uint8_t MSG_TYPE_STATE = 0x25;            // 37
constexpr uint8_t MSG_TYPE_RESET = 0x28;            // 40
constexpr uint8_t MSG_TYPE_CALIBRATIONTIMES = 0x30; // 48
constexpr uint8_t MSG_TYPE_TIMEDIFF = 0x31;         // 49

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
constexpr uint8_t RX_BUF_LEN = 8 + (2 + MAX_ANCHORS + MAX_TAGS) * FINAL_MSG_TS_LEN + 2;
static uint8_t rx_buffer[RX_BUF_LEN];

/* conversion factors from seconds to deca device time units and vice versa*/
constexpr double DWT_TO_SECONDS = 1.0 / 499.2e6 / 128.0;  // 15.65e-12 s
constexpr double SECONDS_TO_DWT = 499.2e6 * 128.0;        // 63.8976e9 device time units (dtu)

// This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature.
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
// This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function.
constexpr float RESP_RX_TO_FINAL_TX_DLY_S = 3.0e-3;
constexpr uint64_t RESP_RX_TO_FINAL_TX_DLY_DTU_U64 = (uint64_t)(RESP_RX_TO_FINAL_TX_DLY_S * SECONDS_TO_DWT);

// receive response timeout (not quite microseconds but 1.026 microseconds)
#define RESP_RX_TIMEOUT_UUS 60000

// Get temperature in °C
float getTemperature_C();

// speed of light in air [m/s]
constexpr double SPEED_OF_LIGHT = 299702547.2;

// tx power
constexpr uint32_t TX_POWER_ID_VAL = 0x009A9A00; //todo: set this potentially to 0x00676700 to have better transmit power

/* Declaration of static functions. */
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
// void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void final_msg_get_ts(const uint8_t *ts_field, uint64_t *ts);

uint64_t get_systemtime_deca();
double get_systemtime_seconds();

void init_decaWaveModule(dwt_config_t *conf, uint16_t antennaDelay);
void uwb_write(uint8_t sourceId, uint8_t destId, uint8_t *msg, int sizeOfMsg, uint8_t mode = DWT_START_TX_IMMEDIATE);
bool uwb_read(uint8_t *rx_buf);

#endif /* DECA_WAVE_MODULE_H_ */
