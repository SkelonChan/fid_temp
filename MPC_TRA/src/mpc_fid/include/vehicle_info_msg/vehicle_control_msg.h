// Generated by gencpp from file vehicle_info_msg/vehicle_control_msg.msg
// DO NOT EDIT!


#ifndef VEHICLE_INFO_MSG_MESSAGE_VEHICLE_CONTROL_MSG_H
#define VEHICLE_INFO_MSG_MESSAGE_VEHICLE_CONTROL_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vehicle_info_msg
{
template <class ContainerAllocator>
struct vehicle_control_msg_
{
  typedef vehicle_control_msg_<ContainerAllocator> Type;

  vehicle_control_msg_()
    : steer_angle(0.0)
    , vx(0.0)  {
    }
  vehicle_control_msg_(const ContainerAllocator& _alloc)
    : steer_angle(0.0)
    , vx(0.0)  {
  (void)_alloc;
    }



   typedef double _steer_angle_type;
  _steer_angle_type steer_angle;

   typedef double _vx_type;
  _vx_type vx;





  typedef boost::shared_ptr< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> const> ConstPtr;

}; // struct vehicle_control_msg_

typedef ::vehicle_info_msg::vehicle_control_msg_<std::allocator<void> > vehicle_control_msg;

typedef boost::shared_ptr< ::vehicle_info_msg::vehicle_control_msg > vehicle_control_msgPtr;
typedef boost::shared_ptr< ::vehicle_info_msg::vehicle_control_msg const> vehicle_control_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator1> & lhs, const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator2> & rhs)
{
  return lhs.steer_angle == rhs.steer_angle &&
    lhs.vx == rhs.vx;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator1> & lhs, const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_info_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c9e4242dd1163cca3807080936e3edb5";
  }

  static const char* value(const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc9e4242dd1163ccaULL;
  static const uint64_t static_value2 = 0x3807080936e3edb5ULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_info_msg/vehicle_control_msg";
  }

  static const char* value(const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 steer_angle\n"
"float64 vx\n"
;
  }

  static const char* value(const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steer_angle);
      stream.next(m.vx);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct vehicle_control_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_info_msg::vehicle_control_msg_<ContainerAllocator>& v)
  {
    s << indent << "steer_angle: ";
    Printer<double>::stream(s, indent + "  ", v.steer_angle);
    s << indent << "vx: ";
    Printer<double>::stream(s, indent + "  ", v.vx);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_INFO_MSG_MESSAGE_VEHICLE_CONTROL_MSG_H