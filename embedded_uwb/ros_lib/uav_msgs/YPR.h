#ifndef _ROS_uav_msgs_YPR_h
#define _ROS_uav_msgs_YPR_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "matlib/api/ypr.h"

namespace uav_msgs
{

  template <typename TYPE = float>
  class YPR_ : public ros::Msg, public RODOS::YPR_<TYPE>
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      uint32_t offset = 0;
      union {
        double real;
        uint64_t base;
      } u_yaw;
      u_yaw.real = (TYPE) this->yaw;
      *(outbuffer + offset + 0) = (uint8_t) (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (uint8_t) (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (uint8_t) (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (uint8_t) (u_yaw.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (uint8_t) (u_yaw.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (uint8_t) (u_yaw.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (uint8_t) (u_yaw.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (uint8_t) (u_yaw.base >> (8 * 7)) & 0xFF;
      offset += sizeof(u_yaw.real);
      union {
        double real;
        uint64_t base;
      } u_pitch;
      u_pitch.real = (TYPE) this->pitch;
      *(outbuffer + offset + 0) = (uint8_t) (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (uint8_t) (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (uint8_t) (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (uint8_t) (u_pitch.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (uint8_t) (u_pitch.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (uint8_t) (u_pitch.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (uint8_t) (u_pitch.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (uint8_t) (u_pitch.base >> (8 * 7)) & 0xFF;
      offset += sizeof(u_pitch.real);
      union {
        double real;
        uint64_t base;
      } u_roll;
      u_roll.real = (TYPE) this->roll;
      *(outbuffer + offset + 0) = (uint8_t) (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (uint8_t) (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (uint8_t) (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (uint8_t) (u_roll.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (uint8_t) (u_roll.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (uint8_t) (u_roll.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (uint8_t) (u_roll.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (uint8_t) (u_roll.base >> (8 * 7)) & 0xFF;
      offset += sizeof(u_roll.real);
      return (int) offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      uint32_t offset = 0;
      union {
        double real;
        uint64_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yaw.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yaw = (TYPE) u_yaw.real;
      offset += sizeof(u_yaw.real);
      union {
        double real;
        uint64_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pitch = (TYPE) u_pitch.real;
      offset += sizeof(u_pitch.real);
      union {
        double real;
        uint64_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->roll = (TYPE) u_roll.real;
      offset += sizeof(u_roll.real);
     return (int) offset;
    }

    const char * getType(){ return "uav_msgs/YPR"; };
    const char * getMD5(){ return "b6483fb20acb1cba981c486a35a1bdbd"; };

  };
  
typedef YPR_<> YPR;

}
#endif
