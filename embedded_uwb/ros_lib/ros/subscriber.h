/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_SUBSCRIBER_H_
#define ROS_SUBSCRIBER_H_

#include "rosserial_msgs/TopicInfo.h"

namespace ros
{

/* Base class for objects subscribers. */
class Subscriber_
{
public:
  virtual void callback(unsigned char *data) = 0;
  virtual int getEndpointType() = 0;
  virtual ~Subscriber_(){};
  // id_ is set by NodeHandle when we advertise
  unsigned int id_;

  virtual const char * getMsgType() = 0;
  virtual const char * getMsgMD5() = 0;
  const char * topic_;
};

/* Bound function subscriber. */
template<typename MsgT, typename ObjT = void>
class Subscriber: public Subscriber_
{
public:
  typedef void(ObjT::*CallbackT)(const MsgT&);
  MsgT msg;

  Subscriber(const char * topic_name, CallbackT cb, ObjT* obj, int endpoint = rosserial_msgs::TopicInfo::ID_SUBSCRIBER) :
    cb_(cb),
    obj_(obj),
    endpoint_(endpoint)
  {
    topic_ = topic_name;
  };
  
  virtual ~Subscriber(){};
  virtual void callback(unsigned char* data) override
  {
    msg.deserialize(data);
    (obj_->*cb_)(msg);
  }

  virtual const char * getMsgType() override
  {
    return this->msg.getType();
  }
  virtual const char * getMsgMD5() override
  {
    return this->msg.getMD5();
  }
  virtual int getEndpointType() override
  {
    return endpoint_;
  }

private:
  CallbackT cb_;
  ObjT* obj_;
  int endpoint_;
};

/* Standalone function subscriber. */
template<typename MsgT>
class Subscriber<MsgT, void>: public Subscriber_
{
public:
  typedef void(*CallbackT)(const MsgT&);
  MsgT msg;

  Subscriber(const char * topic_name, CallbackT cb, int endpoint = rosserial_msgs::TopicInfo::ID_SUBSCRIBER) :
    cb_(cb),
    endpoint_(endpoint)
  {
    topic_ = topic_name;
  };

  virtual void callback(unsigned char* data) override
  {
    msg.deserialize(data);
    this->cb_(msg);
  }

  virtual const char * getMsgType() override
  {
    return this->msg.getType();
  }
  virtual const char * getMsgMD5() override
  {
    return this->msg.getMD5();
  }
  virtual int getEndpointType() override
  {
    return endpoint_;
  }

private:
  CallbackT cb_;
  int endpoint_;
};

}

#endif
