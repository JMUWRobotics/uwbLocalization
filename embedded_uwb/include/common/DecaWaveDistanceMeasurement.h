/**
 * @file DecaWaveDistanceMeasurement.h
*/

#ifndef DECA_WAVE_DISTANCE_MEASUREMENT_H
#define DECA_WAVE_DISTANCE_MEASUREMENT_H

#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>

#include "decaWaveModule.h"
#include "defines_resp.h"

#include "nodeState.h"

#include "../../decadriver/deca_device_api.h"
#include "../../decadriver/deca_regs.h"

#include "../../ros_lib/ros.h"
#include "../../ros_lib/ros_gateway.h"
#include "../../ros_lib/std_msgs/Bool.h"
#include "../../ros_lib/std_msgs/Float64.h"
#include "../../ros_lib/std_msgs/UInt8.h"
#include "../../ros_lib/std_msgs/UInt32.h"
#include "../../ros_lib/std_msgs/UInt64.h"
#include "../../ros_lib/uwb/Distance.h"
#include "../../ros_lib/uwb/UWBMsg.h"
#include "../../ros_lib/uwb/TimeDiff.h"
#include "../../ros_lib/uwb/TimingResp.h"
#include "rodos.h"
#include "hal.h"

static std::map<int32_t, uint16_t> antenna_delays = {  //old antenna delay: 16436
    {3407920, 16436},
    {3145773, 16436},
    {4259888, 16436},
    {2162733, 16436},
    {2687023, 16436},
    {3473456, 16436},
    {1572908, 16436},
    {4063280, 16436},
    {2621487, 16436},
    {2097199, 16436},
    {2555942, 16436},
    {2228272, 16436},
    {3997741, 16436},
    {2162735, 16436},
    {1900591, 16436},
    {2752557, 16436},
    {2818095, 16436}
};

/**
 * @brief finds the key in a given map, that corresponds to the given value
 * 
 * @param map the map in which to search
 * @param value the value of which the key is searched
 * @return uint8_t the key of the value 
 */
static uint8_t findInMap(std::map<uint8_t, uint8_t> map, uint8_t value){
    for (auto it = map.begin(); it != map.end(); ++it){
        if (it->second == value)
            return it->first;
    }
    return -1;
}

constexpr uint64_t MAX_VALUE_40_BIT = 0xFFFFFFFFFF;
constexpr uint64_t MAX_VALUE_64_BIT = 0xFFFFFFFFFFFFFFFF;

static Application module01("Decawave Distance Measurement Node", 2001);

class DecaWaveDistanceMeasurement : public StaticThread<>, public IOEventReceiver {
public:
    // Constants to be set:
    int64_t respWaitTimePerNode = 5 * MILLISECONDS;

    DecaWaveDistanceMeasurement(const char* name) : StaticThread<>(name, 1000) {}
    void init();
    void run();
    uint8_t next_destId(uint8_t offset = 0);
    
    void reset();

    void send_dist(float distance, uint8_t destId, uint8_t twrSourceId);
    void send_state(uint8_t destId);
    void send_activate(uint8_t destId, uint8_t counter = 0);
    void send_reset();
    void send_clibrationTimes();
    void send_timeDiff(uint64_t *timeDiff, uint8_t sourceId);

    void start_twr();
    void send_response(uint8_t destId);
    void send_final();

    uint8_t receiveMessages();

    float calculate_distance();
    void calculate_timeDiffs(uint8_t sourceId);

    uint8_t numberOffActiveNodes = 0;
    
    uint64_t poll_tx_ts;   // DS-TWR poll transmit timestamp
    uint64_t poll_rx_ts;   // DS-TWR poll receive timestamp
    uint64_t resp_tx_ts;   // DS-TWR response transmit timestamp
    uint64_t resp_rx_ts[MAX_ANCHORS + MAX_TAGS];   // DS-TWR response receive timestamps
    uint64_t final_tx_ts;  // DS-TWR final transmit timestamp
    uint64_t final_rx_ts;  // DS-TWR final receive timestamp

    float distances[MAX_ANCHORS + MAX_TAGS]; // measured distances to more then one node

    int64_t nextTime2Action;  // timestamp when the next message should be send (RODOS time)
};

#endif /* VaMEx_DWN_H_ */
