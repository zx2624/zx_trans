// Generated by gencpp from file iv_dynamicobject_msgs/RadarData.msg
// DO NOT EDIT!


#ifndef IV_DYNAMICOBJECT_MSGS_MESSAGE_RADARDATA_H
#define IV_DYNAMICOBJECT_MSGS_MESSAGE_RADARDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <iv_dynamicobject_msgs/RadarPoint.h>

namespace iv_dynamicobject_msgs
{
template <class ContainerAllocator>
struct RadarData_
{
  typedef RadarData_<ContainerAllocator> Type;

  RadarData_()
    : header()
    , delphi_detection_array()
    , ACC_Target_ID(0)  {
    }
  RadarData_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , delphi_detection_array()
    , ACC_Target_ID(0)  {
  (void)_alloc;
      delphi_detection_array.assign( ::iv_dynamicobject_msgs::RadarPoint_<ContainerAllocator> (_alloc));
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array< ::iv_dynamicobject_msgs::RadarPoint_<ContainerAllocator> , 64>  _delphi_detection_array_type;
  _delphi_detection_array_type delphi_detection_array;

   typedef uint8_t _ACC_Target_ID_type;
  _ACC_Target_ID_type ACC_Target_ID;





  typedef boost::shared_ptr< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> const> ConstPtr;

}; // struct RadarData_

typedef ::iv_dynamicobject_msgs::RadarData_<std::allocator<void> > RadarData;

typedef boost::shared_ptr< ::iv_dynamicobject_msgs::RadarData > RadarDataPtr;
typedef boost::shared_ptr< ::iv_dynamicobject_msgs::RadarData const> RadarDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace iv_dynamicobject_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'iv_dynamicobject_msgs': ['/home/zx/test/ws_zx_trans/src/msgs/iv_dynamicobject_msgs/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "11cc650889f4145ca7db11cb95c6e9cb";
  }

  static const char* value(const ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x11cc650889f4145cULL;
  static const uint64_t static_value2 = 0xa7db11cb95c6e9cbULL;
};

template<class ContainerAllocator>
struct DataType< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "iv_dynamicobject_msgs/RadarData";
  }

  static const char* value(const ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
RadarPoint[64] delphi_detection_array\n\
uint8 ACC_Target_ID\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: iv_dynamicobject_msgs/RadarPoint\n\
uint8 target_ID\n\
float32 range\n\
float32 v\n\
float32 angle\n\
float32 x\n\
float32 y\n\
bool valid\n\
uint8 status\n\
uint8 moving\n\
bool moving_fast\n\
bool moving_slow\n\
";
  }

  static const char* value(const ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.delphi_detection_array);
      stream.next(m.ACC_Target_ID);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RadarData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::iv_dynamicobject_msgs::RadarData_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "delphi_detection_array[]" << std::endl;
    for (size_t i = 0; i < v.delphi_detection_array.size(); ++i)
    {
      s << indent << "  delphi_detection_array[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::iv_dynamicobject_msgs::RadarPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.delphi_detection_array[i]);
    }
    s << indent << "ACC_Target_ID: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ACC_Target_ID);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IV_DYNAMICOBJECT_MSGS_MESSAGE_RADARDATA_H