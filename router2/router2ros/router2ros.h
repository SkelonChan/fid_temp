#ifndef _ROS_router2ros_router2ros_h
#define _ROS_router2ros_router2ros_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace router2ros
{

  class router2ros : public ros::Msg
  {
    public:
      typedef int16_t _motor_value_type;
      _motor_value_type motor_value;
      typedef int16_t _servo_value_type;
      _servo_value_type servo_value;

    router2ros():
      motor_value(0),
      servo_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_motor_value;
      u_motor_value.real = this->motor_value;
      *(outbuffer + offset + 0) = (u_motor_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor_value);
      union {
        int16_t real;
        uint16_t base;
      } u_servo_value;
      u_servo_value.real = this->servo_value;
      *(outbuffer + offset + 0) = (u_servo_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_motor_value;
      u_motor_value.base = 0;
      u_motor_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor_value = u_motor_value.real;
      offset += sizeof(this->motor_value);
      union {
        int16_t real;
        uint16_t base;
      } u_servo_value;
      u_servo_value.base = 0;
      u_servo_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->servo_value = u_servo_value.real;
      offset += sizeof(this->servo_value);
     return offset;
    }

    virtual const char * getType() override { return "router2ros/router2ros"; };
    virtual const char * getMD5() override { return "c59bb685b25a0a34e81a1a8a097b9eae"; };

  };

}
#endif
