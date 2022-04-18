#ifndef _ROS_bcr1_control_armCmd_h
#define _ROS_bcr1_control_armCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bcr1_control
{

  class armCmd : public ros::Msg
  {
    public:
      float effort[6];
      float angle[6];

    armCmd():
      effort(),
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
      } u_efforti;
      u_efforti.real = this->effort[i];
      *(outbuffer + offset + 0) = (u_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_efforti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort[i]);
      }
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
      } u_efforti;
      u_efforti.base = 0;
      u_efforti.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_efforti.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_efforti.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_efforti.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->effort[i] = u_efforti.real;
      offset += sizeof(this->effort[i]);
      }
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

    virtual const char * getType() override { return "bcr1_control/armCmd"; };
    virtual const char * getMD5() override { return "03eab56b29be0236e78be72f0da0e475"; };

  };

}
#endif
