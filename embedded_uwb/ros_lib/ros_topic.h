/*
 * ros_topic.h
 *
 *  Created on: 01.12.2020
 *      Author: Michael Strohmeier
 */

#pragma once

#include "ros.h"

template <class Type>
class ROS_Topic : public Topic<Type>{
	Type msg;
public:
	ros::NodeHandle *nh_;
	RODOS::Semaphore *sema_;
	ros::Publisher pub;
	ros::Subscriber<Type, ROS_Topic> sub;

	ROS_Topic(long id, const char* name) :
		Topic<Type>(id, name),
		pub(name, &msg),
		sub(name, &ROS_Topic::callback, this){
		nh_ = NULL;
		sema_ = NULL;
	}

	void callback(const Type& data){
		Type copy = data;
		this->publish(copy, false);
	}

	/* Overload publish */
    inline unsigned long publish(Type &msg, bool shallSendToNetwork = true) {
    	if(shallSendToNetwork){
   			if(sema_)
   				sema_->enter();
    		if(nh_){
    			pub.publish(&msg);
    		}
   			if(sema_)
   				sema_->leave();
    	}
        return TopicInterface::publish(&msg, shallSendToNetwork);
    }
};
