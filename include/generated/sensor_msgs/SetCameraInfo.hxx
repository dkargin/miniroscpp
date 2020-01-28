// Generated by gencxx from file sensor_msgs/SetCameraInfo.msg
// DO NOT EDIT!


#pragma once

#include <miniros/traits/service_traits.h>


#include <sensor_msgs/SetCameraInfoRequest.hxx>
#include <sensor_msgs/SetCameraInfoResponse.hxx>


namespace sensor_msgs
{

struct SetCameraInfo
{

typedef SetCameraInfoRequest Request;
typedef SetCameraInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCameraInfo
} // namespace sensor_msgs


namespace miniros
{
namespace service_traits
{


template<>
struct MD5Sum< ::sensor_msgs::SetCameraInfo > {
  static const char* value()
  {
    return "bef1df590ed75ed1f393692395e15482";
  }

  static const char* value(const ::sensor_msgs::SetCameraInfo&) { return value(); }
};

template<>
struct DataType< ::sensor_msgs::SetCameraInfo > {
  static const char* value()
  {
    return "sensor_msgs/SetCameraInfo";
  }

  static const char* value(const ::sensor_msgs::SetCameraInfo&) { return value(); }
};


// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfoRequest> should match 
// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfo > 
template<>
struct MD5Sum< ::sensor_msgs::SetCameraInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::sensor_msgs::SetCameraInfoRequest> should match 
// service_traits::DataType< ::sensor_msgs::SetCameraInfo > 
template<>
struct DataType< ::sensor_msgs::SetCameraInfoRequest>
{
  static const char* value()
  {
    return DataType< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfoResponse> should match 
// service_traits::MD5Sum< ::sensor_msgs::SetCameraInfo > 
template<>
struct MD5Sum< ::sensor_msgs::SetCameraInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::sensor_msgs::SetCameraInfoResponse> should match 
// service_traits::DataType< ::sensor_msgs::SetCameraInfo > 
template<>
struct DataType< ::sensor_msgs::SetCameraInfoResponse>
{
  static const char* value()
  {
    return DataType< ::sensor_msgs::SetCameraInfo >::value();
  }
  static const char* value(const ::sensor_msgs::SetCameraInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace miniros