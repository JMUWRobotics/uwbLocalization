#ifndef _ROS_uwb_UWBMsg_h
#define _ROS_uwb_UWBMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uwb
{

  class UWBMsg : public ros::Msg
  {
    public:
      typedef uint8_t _msgType_type;
      _msgType_type msgType;
      typedef uint8_t _sender_type;
      _sender_type sender;
      typedef uint8_t _receiver_type;
      _receiver_type receiver;
      typedef double _distance_type;
      _distance_type distance;
      typedef uint8_t _state_type;
      _state_type state;
      typedef uint64_t _timeDiff1_type;
      _timeDiff1_type timeDiff1;
      typedef uint64_t _timeDiff2_type;
      _timeDiff2_type timeDiff2;

    UWBMsg():
      msgType(0),
      sender(0),
      receiver(0),
      distance(0),
      state(0),
      timeDiff1(0),
      timeDiff2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->msgType >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msgType);
      *(outbuffer + offset + 0) = (this->sender >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sender);
      *(outbuffer + offset + 0) = (this->receiver >> (8 * 0)) & 0xFF;
      offset += sizeof(this->receiver);
      union {
        double real;
        uint64_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->distance);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      *(outbuffer + offset + 0) = (this->timeDiff1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeDiff1 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeDiff1 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeDiff1 >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->timeDiff1 >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->timeDiff1 >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->timeDiff1 >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->timeDiff1 >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timeDiff1);
      *(outbuffer + offset + 0) = (this->timeDiff2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeDiff2 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeDiff2 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeDiff2 >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->timeDiff2 >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->timeDiff2 >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->timeDiff2 >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->timeDiff2 >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timeDiff2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->msgType =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->msgType);
      this->sender =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sender);
      this->receiver =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->receiver);
      union {
        double real;
        uint64_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      this->timeDiff1 =  ((uint64_t) (*(inbuffer + offset)));
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->timeDiff1 |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->timeDiff1);
      this->timeDiff2 =  ((uint64_t) (*(inbuffer + offset)));
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->timeDiff2 |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->timeDiff2);
     return offset;
    }

    virtual const char * getType() override { return "uwb/UWBMsg"; };
    virtual const char * getMD5() override { return "65a3ff7b719ffcc99263feae3b73e108"; };

  };

}
#endif
