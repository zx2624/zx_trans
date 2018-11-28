// Generated by gencpp from file speed_debug_msgs/super_point.msg
// DO NOT EDIT!


#ifndef SPEED_DEBUG_MSGS_MESSAGE_SUPER_POINT_H
#define SPEED_DEBUG_MSGS_MESSAGE_SUPER_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <speed_debug_msgs/v.h>
#include <speed_debug_msgs/curv.h>
#include <speed_debug_msgs/speed_time.h>

namespace speed_debug_msgs
{
template <class ContainerAllocator>
struct super_point_
{
  typedef super_point_<ContainerAllocator> Type;

  super_point_()
    : v()
    , curv()
    , time()
    , x(0.0)
    , y(0.0)
    , s(0.0)
    , acc(0.0)  {
    }
  super_point_(const ContainerAllocator& _alloc)
    : v(_alloc)
    , curv(_alloc)
    , time(_alloc)
    , x(0.0)
    , y(0.0)
    , s(0.0)
    , acc(0.0)  {
  (void)_alloc;
    }



   typedef  ::speed_debug_msgs::v_<ContainerAllocator>  _v_type;
  _v_type v;

   typedef  ::speed_debug_msgs::curv_<ContainerAllocator>  _curv_type;
  _curv_type curv;

   typedef  ::speed_debug_msgs::speed_time_<ContainerAllocator>  _time_type;
  _time_type time;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _s_type;
  _s_type s;

   typedef double _acc_type;
  _acc_type acc;





  typedef boost::shared_ptr< ::speed_debug_msgs::super_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::speed_debug_msgs::super_point_<ContainerAllocator> const> ConstPtr;

}; // struct super_point_

typedef ::speed_debug_msgs::super_point_<std::allocator<void> > super_point;

typedef boost::shared_ptr< ::speed_debug_msgs::super_point > super_pointPtr;
typedef boost::shared_ptr< ::speed_debug_msgs::super_point const> super_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::speed_debug_msgs::super_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::speed_debug_msgs::super_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace speed_debug_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'speed_debug_msgs': ['/home/zx/test/ws_zx_trans/src/msgs/speed_debug_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::speed_debug_msgs::super_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::speed_debug_msgs::super_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::speed_debug_msgs::super_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::speed_debug_msgs::super_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::speed_debug_msgs::super_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::speed_debug_msgs::super_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::speed_debug_msgs::super_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fdc128084915a3dc47e2491ff229fb45";
  }

  static const char* value(const ::speed_debug_msgs::super_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfdc128084915a3dcULL;
  static const uint64_t static_value2 = 0x47e2491ff229fb45ULL;
};

template<class ContainerAllocator>
struct DataType< ::speed_debug_msgs::super_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "speed_debug_msgs/super_point";
  }

  static const char* value(const ::speed_debug_msgs::super_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::speed_debug_msgs::super_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "v v\n\
curv curv\n\
speed_time time\n\
float64 x\n\
float64 y\n\
float64 s\n\
float64 acc\n\
================================================================================\n\
MSG: speed_debug_msgs/v\n\
float64 v_max_dynamic\n\
float64 v_lat_acc\n\
float64 v_lon_acc\n\
float64 v_slide_dec\n\
float64 v_lon_dec\n\
float64 v_jerk\n\
float64 v_blind\n\
float64 v_constrained\n\
================================================================================\n\
MSG: speed_debug_msgs/curv\n\
float64 curv_final\n\
================================================================================\n\
MSG: speed_debug_msgs/speed_time\n\
float64 time\n\
";
  }

  static const char* value(const ::speed_debug_msgs::super_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::speed_debug_msgs::super_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.v);
      stream.next(m.curv);
      stream.next(m.time);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.s);
      stream.next(m.acc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct super_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::speed_debug_msgs::super_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::speed_debug_msgs::super_point_<ContainerAllocator>& v)
  {
    s << indent << "v: ";
    s << std::endl;
    Printer< ::speed_debug_msgs::v_<ContainerAllocator> >::stream(s, indent + "  ", v.v);
    s << indent << "curv: ";
    s << std::endl;
    Printer< ::speed_debug_msgs::curv_<ContainerAllocator> >::stream(s, indent + "  ", v.curv);
    s << indent << "time: ";
    s << std::endl;
    Printer< ::speed_debug_msgs::speed_time_<ContainerAllocator> >::stream(s, indent + "  ", v.time);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "s: ";
    Printer<double>::stream(s, indent + "  ", v.s);
    s << indent << "acc: ";
    Printer<double>::stream(s, indent + "  ", v.acc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SPEED_DEBUG_MSGS_MESSAGE_SUPER_POINT_H