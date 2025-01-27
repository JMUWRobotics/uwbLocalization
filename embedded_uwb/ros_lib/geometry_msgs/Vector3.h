#ifndef _ROS_geometry_msgs_Vector3_h
#define _ROS_geometry_msgs_Vector3_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "matlib/api/vector.h"

namespace geometry_msgs
{

  template <typename TYPE = float>
  class Vector3_ : public ros::Msg, public RODOS::Vector3D_<TYPE>
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      uint32_t offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = (TYPE) this->x;
      *(outbuffer + offset + 0) = (uint8_t) (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (uint8_t) (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (uint8_t) (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (uint8_t) (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (uint8_t) (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (uint8_t) (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (uint8_t) (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (uint8_t) (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(u_x.real);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = (TYPE) this->y;
      *(outbuffer + offset + 0) = (uint8_t) (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (uint8_t) (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (uint8_t) (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (uint8_t) (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (uint8_t) (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (uint8_t) (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (uint8_t) (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (uint8_t) (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(u_y.real);
      union {
        double real;
        uint64_t base;
      } u_z;
      u_z.real = (TYPE) this->z;
      *(outbuffer + offset + 0) = (uint8_t) (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (uint8_t) (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (uint8_t) (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (uint8_t) (u_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (uint8_t) (u_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (uint8_t) (u_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (uint8_t) (u_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (uint8_t) (u_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(u_z.real);
      return (int) offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      uint32_t offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = (TYPE) u_x.real;
      offset += sizeof(u_x.real);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = (TYPE) u_y.real;
      offset += sizeof(u_y.real);
      union {
        double real;
        uint64_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->z = (TYPE) u_z.real;
      offset += sizeof(u_z.real);
     return (int) offset;
    }

    const char * getType(){ return "geometry_msgs/Vector3"; };
    const char * getMD5(){ return "4a842b65f413084dc2b10fb484ea7f17"; };

	template <typename TYPE2>
	Vector3_<TYPE> operator=(const Vector3_<TYPE2> &other) {
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		return *this;
	}

	template <typename TYPE2>
	Vector3_<TYPE> operator=(const RODOS::Vector3D_<TYPE2> &other) {
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		return *this;
	}

	template <typename TYPE2>
	Vector3_<TYPE> operator=(const RODOS::Matrix_<3,1,TYPE2> &other) {
		this->x = other.r[0][0];
		this->y = other.r[1][0];
		this->z = other.r[2][0];
		return *this;
	}

  };

  typedef Vector3_<float> Vector3;

}
#endif


