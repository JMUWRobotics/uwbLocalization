#ifndef _ROS_uwb_TimeDiff_h
#define _ROS_uwb_TimeDiff_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uwb
{

  class TimeDiff : public ros::Msg
  {
    public:
      typedef uint8_t _source_type;
      _source_type source;
      typedef uint8_t _receiver_type;
      _receiver_type receiver;
      typedef uint64_t _timeDiff1_type;
      _timeDiff1_type timeDiff1;
      typedef uint64_t _timeDiff2_type;
      _timeDiff2_type timeDiff2;
      typedef uint64_t _timeDiff3_type;
      _timeDiff3_type timeDiff3;
      typedef uint64_t _timeDiff4_type;
      _timeDiff4_type timeDiff4;

    TimeDiff():
      source(0),
      receiver(0),
      timeDiff1(0),
      timeDiff2(0),
      timeDiff3(0),
      timeDiff4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->source >> (8 * 0)) & 0xFF;
      offset += sizeof(this->source);
      *(outbuffer + offset + 0) = (this->receiver >> (8 * 0)) & 0xFF;
      offset += sizeof(this->receiver);
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
      *(outbuffer + offset + 0) = (this->timeDiff3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeDiff3 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeDiff3 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeDiff3 >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->timeDiff3 >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->timeDiff3 >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->timeDiff3 >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->timeDiff3 >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timeDiff3);
      *(outbuffer + offset + 0) = (this->timeDiff4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeDiff4 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeDiff4 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeDiff4 >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->timeDiff4 >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->timeDiff4 >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->timeDiff4 >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->timeDiff4 >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timeDiff4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->source =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->source);
      this->receiver =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->receiver);
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
      this->timeDiff3 =  ((uint64_t) (*(inbuffer + offset)));
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->timeDiff3 |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->timeDiff3);
      this->timeDiff4 =  ((uint64_t) (*(inbuffer + offset)));
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->timeDiff4 |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->timeDiff4);
     return offset;
    }

    virtual const char * getType() override { return "uwb/TimeDiff"; };
    virtual const char * getMD5() override { return "91726d9c4a10d55d843f8b3e96f25a25"; };

  };

}
#endif
