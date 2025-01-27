#ifndef NODE_STATE_H
#define NODE_STATE_H

#include <map>
#include <stdint.h>

enum class NodeState{
    START = 0x00,               // node is in start modus
    PAUSE = 0x01,               // node pauses and only respondes
    START_TWR = 0x02,           // node polls the distances to all other nodes
    SEND_FINAL = 0x03,          // node waits a certain amount of time then sends out the final msg, with as many resp. timestamps as it got
    ACTIVATE_NEXT_NODE = 0x04   // node activates the next node and puts itself into PAUSE state
};

class Node {
public:
    Node(int32_t id);

    void toggleState();

    NodeState getState() { return state; }

    int32_t getId() { return id; }

    uint8_t getNodeIndex() { return nodeIndex; }
    void setNodeIndex(uint8_t idx) { nodeIndex = idx; }

    void reset() { state = NodeState::START; }

private:
    int32_t id;         //the hardware serial number of the chip
    uint8_t nodeIndex;  //the index based on how many nodes are active
    NodeState state = NodeState::START;
};

#endif // NODE_STATE_H
