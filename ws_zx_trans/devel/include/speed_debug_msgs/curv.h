// Generated by gencpp from file speed_debug_msgs/curv.msg
// DO NOT EDIT!


#ifndef SPEED_DEBUG_MSGS_MESSAGE_CURV_H
#define SPEED_DEBUG_MSGS_MESSAGE_CURV_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace speed_debug_msgs
{
template <class ContainerAllocator>
struct curv_
{
  typedef curv_<ContainerAllocator> Type;

  curv_()
    : curv_final(0.0)  {
    }
  curv_(const ContainerAllocator& _alloc)
    : curv_final(0.0)  {
  (void)_alloc;
    }



   typedef double _curv_final_type;
  _curv_final_type curv_final;





  typedef boost::shared_ptr< ::speed_debug_msgs::curv_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::speed_debug_msgs::curv_<ContainerAllocator> const> ConstPtr;

}; // struct curv_

typedef ::speed_debug_msgs::curv_<std::allocator<void> > curv;

typedef boost::shared_ptr< ::speed_debug_msgs::curv > curvPtr;
typedef boost::shared_ptr< ::speed_debug_msgs::curv const> curvConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::speed_debug_msgs::curv_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::speed_debug_msgs::curv_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::speed_debug_msgs::curv_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::speed_debug_msgs::curv_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::speed_debug_msgs::curv_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::speed_debug_msgs::curv_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::speed_debug_msgs::curv_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::speed_debug_msgs::curv_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::speed_debug_msgs::curv_<ContainerAllocator> >
{
  static const char* value()
  {
    return "097b555501edfa9e055b8ced874a3825";
  }

  static const char* value(const ::speed_debug_msgs::curv_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x097b555501edfa9eULL;
  static const uint64_t static_value2 = 0x055b8ced874a3825ULL;
};

template<class ContainerAllocator>
struct DataType< ::speed_debug_msgs::curv_<ContainerAllocator> >
{
  static const char* value()
  {
    return "speed_debug_msgs/curv";
  }

  static const char* value(const ::speed_debug_msgs::curv_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::speed_debug_msgs::curv_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 curv_final\n\
";
  }

  static const char* value(const ::speed_debug_msgs::curv_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::speed_debug_msgs::curv_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.curv_final);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct curv_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::speed_debug_msgs::curv_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::speed_debug_msgs::curv_<ContainerAllocator>& v)
  {
    s << indent << "curv_final: ";
    Printer<double>::stream(s, indent + "  ", v.curv_final);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SPEED_DEBUG_MSGS_MESSAGE_CURV_H