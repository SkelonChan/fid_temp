// Generated by gencpp from file vehicle_info_msg/GPRdata.msg
// DO NOT EDIT!


#ifndef VEHICLE_INFO_MSG_MESSAGE_GPRDATA_H
#define VEHICLE_INFO_MSG_MESSAGE_GPRDATA_H


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
struct GPRdata_
{
  typedef GPRdata_<ContainerAllocator> Type;

  GPRdata_()
    : mean_vx(0.0)
    , mean_vy(0.0)
    , mean_yaw_rate(0.0)
    , covar(0.0)  {
    }
  GPRdata_(const ContainerAllocator& _alloc)
    : mean_vx(0.0)
    , mean_vy(0.0)
    , mean_yaw_rate(0.0)
    , covar(0.0)  {
  (void)_alloc;
    }



   typedef double _mean_vx_type;
  _mean_vx_type mean_vx;

   typedef double _mean_vy_type;
  _mean_vy_type mean_vy;

   typedef double _mean_yaw_rate_type;
  _mean_yaw_rate_type mean_yaw_rate;

   typedef double _covar_type;
  _covar_type covar;





  typedef boost::shared_ptr< ::vehicle_info_msg::GPRdata_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_info_msg::GPRdata_<ContainerAllocator> const> ConstPtr;

}; // struct GPRdata_

typedef ::vehicle_info_msg::GPRdata_<std::allocator<void> > GPRdata;

typedef boost::shared_ptr< ::vehicle_info_msg::GPRdata > GPRdataPtr;
typedef boost::shared_ptr< ::vehicle_info_msg::GPRdata const> GPRdataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_info_msg::GPRdata_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_info_msg::GPRdata_<ContainerAllocator1> & lhs, const ::vehicle_info_msg::GPRdata_<ContainerAllocator2> & rhs)
{
  return lhs.mean_vx == rhs.mean_vx &&
    lhs.mean_vy == rhs.mean_vy &&
    lhs.mean_yaw_rate == rhs.mean_yaw_rate &&
    lhs.covar == rhs.covar;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_info_msg::GPRdata_<ContainerAllocator1> & lhs, const ::vehicle_info_msg::GPRdata_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_info_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_info_msg::GPRdata_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_info_msg::GPRdata_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_info_msg::GPRdata_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6c1582d3e2c6a1718450a2aec2023e44";
  }

  static const char* value(const ::vehicle_info_msg::GPRdata_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6c1582d3e2c6a171ULL;
  static const uint64_t static_value2 = 0x8450a2aec2023e44ULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_info_msg/GPRdata";
  }

  static const char* value(const ::vehicle_info_msg::GPRdata_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 mean_vx\n"
"float64 mean_vy\n"
"float64 mean_yaw_rate\n"
"float64 covar\n"
;
  }

  static const char* value(const ::vehicle_info_msg::GPRdata_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mean_vx);
      stream.next(m.mean_vy);
      stream.next(m.mean_yaw_rate);
      stream.next(m.covar);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPRdata_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_info_msg::GPRdata_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_info_msg::GPRdata_<ContainerAllocator>& v)
  {
    s << indent << "mean_vx: ";
    Printer<double>::stream(s, indent + "  ", v.mean_vx);
    s << indent << "mean_vy: ";
    Printer<double>::stream(s, indent + "  ", v.mean_vy);
    s << indent << "mean_yaw_rate: ";
    Printer<double>::stream(s, indent + "  ", v.mean_yaw_rate);
    s << indent << "covar: ";
    Printer<double>::stream(s, indent + "  ", v.covar);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_INFO_MSG_MESSAGE_GPRDATA_H
