#ifndef _ROS_uwb_TimingResp_h
#define _ROS_uwb_TimingResp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace uwb
{

  class TimingResp : public ros::Msg
  {
    public:
      typedef uint8_t _numSucResp_type;
      _numSucResp_type numSucResp;
      typedef uint32_t _curWaitTime_type;
      _curWaitTime_type curWaitTime;

    TimingResp():
      numSucResp(0),
      curWaitTime(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->numSucResp >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSucResp);
      *(outbuffer + offset + 0) = (this->curWaitTime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->curWaitTime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->curWaitTime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->curWaitTime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->curWaitTime);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->numSucResp =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSucResp);
      this->curWaitTime =  ((uint32_t) (*(inbuffer + offset)));
      this->curWaitTime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->curWaitTime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->curWaitTime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->curWaitTime);
     return offset;
    }

    virtual const char * getType() override { return "uwb/TimingResp"; };
    virtual const char * getMD5() override { return "a60808f20d86cc38a1e358ddbab0f494"; };

  };

}
#endif
