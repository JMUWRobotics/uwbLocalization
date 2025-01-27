#ifndef _ROS_uwb_Distance_h
#define _ROS_uwb_Distance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uwb
{

  class Distance : public ros::Msg
  {
    public:
      typedef uint8_t _node1_type;
      _node1_type node1;
      typedef uint8_t _node2_type;
      _node2_type node2;
      typedef float _distance_type;
      _distance_type distance;

    Distance():
      node1(0),
      node2(0),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->node1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->node1);
      *(outbuffer + offset + 0) = (this->node2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->node2);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->node1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->node1);
      this->node2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->node2);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
     return offset;
    }

    virtual const char * getType() override { return "uwb/Distance"; };
    virtual const char * getMD5() override { return "dfe99ced0855f61a838c6df111f1cdcb"; };

  };

}
#endif
