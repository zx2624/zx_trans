// Generated by gencpp from file speed_debug_msgs/speed_debug.msg
// DO NOT EDIT!


#ifndef SPEED_DEBUG_MSGS_MESSAGE_SPEED_DEBUG_H
#define SPEED_DEBUG_MSGS_MESSAGE_SPEED_DEBUG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <speed_debug_msgs/super_point.h>
#include <speed_debug_msgs/cur_steer.h>
#include <speed_debug_msgs/speed_issue.h>

namespace speed_debug_msgs
{
template <class ContainerAllocator>
struct speed_debug_
{
  typedef speed_debug_<ContainerAllocator> Type;

  speed_debug_()
    : points()
    , cur_steer()
    , issue()
    , pub_ros_time(0.0)  {
    }
  speed_debug_(const ContainerAllocator& _alloc)
    : points(_alloc)
    , cur_steer(_alloc)
    , issue(_alloc)
    , pub_ros_time(0.0)  {
  (void)_alloc;
    }



   typedef std::vector< ::speed_debug_msgs::super_point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::speed_debug_msgs::super_point_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef  ::speed_debug_msgs::cur_steer_<ContainerAllocator>  _cur_steer_type;
  _cur_steer_type cur_steer;

   typedef  ::speed_debug_msgs::speed_issue_<ContainerAllocator>  _issue_type;
  _issue_type issue;

   typedef double _pub_ros_time_type;
  _pub_ros_time_type pub_ros_time;





  typedef boost::shared_ptr< ::speed_debug_msgs::speed_debug_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::speed_debug_msgs::speed_debug_<ContainerAllocator> const> ConstPtr;

}; // struct speed_debug_

typedef ::speed_debug_msgs::speed_debug_<std::allocator<void> > speed_debug;

typedef boost::shared_ptr< ::speed_debug_msgs::speed_debug > speed_debugPtr;
typedef boost::shared_ptr< ::speed_debug_msgs::speed_debug const> speed_debugConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::speed_debug_msgs::speed_debug_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace speed_debug_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'speed_debug_msgs': ['/home/zx/test/ws_zx_trans/src/msgs/speed_debug_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::speed_debug_msgs::speed_debug_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::speed_debug_msgs::speed_debug_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::speed_debug_msgs::speed_debug_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d834a9927e9daa0494e9fa3070e36b02";
  }

  static const char* value(const ::speed_debug_msgs::speed_debug_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd834a9927e9daa04ULL;
  static const uint64_t static_value2 = 0x94e9fa3070e36b02ULL;
};

template<class ContainerAllocator>
struct DataType< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "speed_debug_msgs/speed_debug";
  }

  static const char* value(const ::speed_debug_msgs::speed_debug_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "super_point[] points\n\
cur_steer cur_steer\n\
speed_issue issue\n\
float64 pub_ros_time\n\
================================================================================\n\
MSG: speed_debug_msgs/super_point\n\
v v\n\
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
================================================================================\n\
MSG: speed_debug_msgs/cur_steer\n\
float64 steer\n\
================================================================================\n\
MSG: speed_debug_msgs/speed_issue\n\
float64 v\n\
float64 acc\n\
";
  }

  static const char* value(const ::speed_debug_msgs::speed_debug_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
      stream.next(m.cur_steer);
      stream.next(m.issue);
      stream.next(m.pub_ros_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct speed_debug_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::speed_debug_msgs::speed_debug_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::speed_debug_msgs::speed_debug_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::speed_debug_msgs::super_point_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "cur_steer: ";
    s << std::endl;
    Printer< ::speed_debug_msgs::cur_steer_<ContainerAllocator> >::stream(s, indent + "  ", v.cur_steer);
    s << indent << "issue: ";
    s << std::endl;
    Printer< ::speed_debug_msgs::speed_issue_<ContainerAllocator> >::stream(s, indent + "  ", v.issue);
    s << indent << "pub_ros_time: ";
    Printer<double>::stream(s, indent + "  ", v.pub_ros_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SPEED_DEBUG_MSGS_MESSAGE_SPEED_DEBUG_H