/*
 * ros_topic.h
 *
 *  Created on: 01.12.2020
 *      Author: Michael Strohmeier
 */

#pragma once

#include "ros_rodos.h"
#include "ros/node_handle.h"

namespace ros
{
	typedef NodeHandle_<ROS_RODOS_CLIENT_UART,
						RODOS_MAX_SUBSCRIBERS,
						RODOS_MAX_PUBLISHERS,
						RODOS_INPUT_SIZE,
						RODOS_OUTPUT_SIZE> NodeHandle;
}


