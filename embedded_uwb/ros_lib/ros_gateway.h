/*
 * ros_gateway.h
 *
 *  Created on: 16.03.2020
 *      Author: Michael Strohmeier
 */

#pragma once

#include "rodos.h"
#include "ros.h"

#include "ros_topic.h"

#define GATEWAY_SPIN_TIME		(100*MILLISECONDS)

class ROS_Gateway : public StaticThread<2000> {
	RODOS::HAL_UART *io_ptr;
	ros::NodeHandle *nh_;
	RODOS::Semaphore sema_;
	int pubCnt_, subCnt_;
public:
	ROS_Gateway(HAL_UART *uart, ros::NodeHandle *nh_) : StaticThread<2000>("ROS_Gateway", NETWORKREADER_PRIORITY){
		pubCnt_ = subCnt_ =  0;
		io_ptr = uart;
		this->nh_ = nh_;
	}

	void init(){
		nh_->initNode((char*) io_ptr);
	}

	template<typename T>
	void addSubscriber(ROS_Topic<T>* topic){
		if(topic == NULL) return;
		if(subCnt_ >= RODOS_MAX_SUBSCRIBERS) return;
		if(topic->nh_ && topic->sema_) return;
		subCnt_++;
		nh_->subscribe(topic->sub);
		topic->nh_ = this->nh_;
		topic->sema_ = &this->sema_;
	}

	template<typename T>
	void addPublisher(ROS_Topic<T>* topic){
		if(topic == NULL) return;
		if(pubCnt_ >= RODOS_MAX_PUBLISHERS) return;
		if(topic->nh_ && topic->sema_) return;
		pubCnt_++;
		nh_->advertise(topic->pub);
		topic->nh_ = this->nh_;
		topic->sema_ = &this->sema_;
	}

	void run() {
		while(1) {
			// Spin every now and then to stay sync, we spin automatically once a message is received
	    	io_ptr->suspendUntilDataReady(NOW() + GATEWAY_SPIN_TIME);
	    	sema_.enter();
	        nh_->spinOnce();
	        sema_.leave();
	    }
	}
};

