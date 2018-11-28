// Generated by gencpp from file geographic_msgs/GetGeoPath.msg
// DO NOT EDIT!


#ifndef GEOGRAPHIC_MSGS_MESSAGE_GETGEOPATH_H
#define GEOGRAPHIC_MSGS_MESSAGE_GETGEOPATH_H

#include <ros/service_traits.h>


#include <geographic_msgs/GetGeoPathRequest.h>
#include <geographic_msgs/GetGeoPathResponse.h>


namespace geographic_msgs
{

struct GetGeoPath
{

typedef GetGeoPathRequest Request;
typedef GetGeoPathResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetGeoPath
} // namespace geographic_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::geographic_msgs::GetGeoPath > {
  static const char* value()
  {
    return "420e1bc36e077856753254ec0523f53a";
  }

  static const char* value(const ::geographic_msgs::GetGeoPath&) { return value(); }
};

template<>
struct DataType< ::geographic_msgs::GetGeoPath > {
  static const char* value()
  {
    return "geographic_msgs/GetGeoPath";
  }

  static const char* value(const ::geographic_msgs::GetGeoPath&) { return value(); }
};


// service_traits::MD5Sum< ::geographic_msgs::GetGeoPathRequest> should match 
// service_traits::MD5Sum< ::geographic_msgs::GetGeoPath > 
template<>
struct MD5Sum< ::geographic_msgs::GetGeoPathRequest>
{
  static const char* value()
  {
    return MD5Sum< ::geographic_msgs::GetGeoPath >::value();
  }
  static const char* value(const ::geographic_msgs::GetGeoPathRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::geographic_msgs::GetGeoPathRequest> should match 
// service_traits::DataType< ::geographic_msgs::GetGeoPath > 
template<>
struct DataType< ::geographic_msgs::GetGeoPathRequest>
{
  static const char* value()
  {
    return DataType< ::geographic_msgs::GetGeoPath >::value();
  }
  static const char* value(const ::geographic_msgs::GetGeoPathRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::geographic_msgs::GetGeoPathResponse> should match 
// service_traits::MD5Sum< ::geographic_msgs::GetGeoPath > 
template<>
struct MD5Sum< ::geographic_msgs::GetGeoPathResponse>
{
  static const char* value()
  {
    return MD5Sum< ::geographic_msgs::GetGeoPath >::value();
  }
  static const char* value(const ::geographic_msgs::GetGeoPathResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::geographic_msgs::GetGeoPathResponse> should match 
// service_traits::DataType< ::geographic_msgs::GetGeoPath > 
template<>
struct DataType< ::geographic_msgs::GetGeoPathResponse>
{
  static const char* value()
  {
    return DataType< ::geographic_msgs::GetGeoPath >::value();
  }
  static const char* value(const ::geographic_msgs::GetGeoPathResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GEOGRAPHIC_MSGS_MESSAGE_GETGEOPATH_H