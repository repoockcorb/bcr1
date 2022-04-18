#ifndef _ROS_bcr1_control_bcr1Telemetry_h
#define _ROS_bcr1_control_bcr1Telemetry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bcr1_control
{

  class bcr1Telemetry : public ros::Msg
  {
    public:
      float angle[6];

    bcr1Telemetry():
      angle()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.base = 0;
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle[i] = u_anglei.real;
      offset += sizeof(this->angle[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "bcr1_control/bcr1Telemetry"; };
    virtual const char * getMD5() override { return "d5646d2d9986672237331f3ea363f45f"; };

  };

}
#endif
