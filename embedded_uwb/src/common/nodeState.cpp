
#include "../../include/common/nodeState.h"

std::map<NodeState, NodeState> state_transitions = {
    {NodeState::START, NodeState::START_TWR},
    {NodeState::PAUSE, NodeState::START_TWR},
    {NodeState::START_TWR, NodeState::SEND_FINAL},
    {NodeState::SEND_FINAL, NodeState::ACTIVATE_NEXT_NODE},
    {NodeState::ACTIVATE_NEXT_NODE, NodeState::PAUSE},
};

// maps the hardware based nodeId to its idx:
std::map<int32_t, uint8_t> node_idx_map = {
    {2162733, 0},
    {3407920, 1},
    {4259888, 2},
    {2555942, 3},
    {4063280, 4},
    {3473456, 5},
    {3145773, 6},
    {2097199, 7},
    {2228272, 8},
    {3997741, 9},
    {2162735, 10},
    {1900591, 11},
    {2752557, 12},
    {2818095, 13},
    {2621487, 14},
    {1572908, 15},
    {2687023, 16}
};

Node::Node(int32_t id) {
    this->id = id;
    this->nodeIndex = node_idx_map[id];
}

void Node::toggleState() {
    this->state = state_transitions[this->state];
}