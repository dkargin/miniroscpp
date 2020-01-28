// Generated by gencxx from file nav_msgs/GetMap.msg
// DO NOT EDIT!


#pragma once

#include <miniros/traits/service_traits.h>


#include <nav_msgs/GetMapRequest.hxx>
#include <nav_msgs/GetMapResponse.hxx>


namespace nav_msgs
{

struct GetMap
{

typedef GetMapRequest Request;
typedef GetMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetMap
} // namespace nav_msgs


namespace miniros
{
namespace service_traits
{


template<>
struct MD5Sum< ::nav_msgs::GetMap > {
  static const char* value()
  {
    return "6cdd0a18e0aff5b0a3ca2326a89b54ff";
  }

  static const char* value(const ::nav_msgs::GetMap&) { return value(); }
};

template<>
struct DataType< ::nav_msgs::GetMap > {
  static const char* value()
  {
    return "nav_msgs/GetMap";
  }

  static const char* value(const ::nav_msgs::GetMap&) { return value(); }
};


// service_traits::MD5Sum< ::nav_msgs::GetMapRequest> should match 
// service_traits::MD5Sum< ::nav_msgs::GetMap > 
template<>
struct MD5Sum< ::nav_msgs::GetMapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::nav_msgs::GetMap >::value();
  }
  static const char* value(const ::nav_msgs::GetMapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::nav_msgs::GetMapRequest> should match 
// service_traits::DataType< ::nav_msgs::GetMap > 
template<>
struct DataType< ::nav_msgs::GetMapRequest>
{
  static const char* value()
  {
    return DataType< ::nav_msgs::GetMap >::value();
  }
  static const char* value(const ::nav_msgs::GetMapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::nav_msgs::GetMapResponse> should match 
// service_traits::MD5Sum< ::nav_msgs::GetMap > 
template<>
struct MD5Sum< ::nav_msgs::GetMapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::nav_msgs::GetMap >::value();
  }
  static const char* value(const ::nav_msgs::GetMapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::nav_msgs::GetMapResponse> should match 
// service_traits::DataType< ::nav_msgs::GetMap > 
template<>
struct DataType< ::nav_msgs::GetMapResponse>
{
  static const char* value()
  {
    return DataType< ::nav_msgs::GetMap >::value();
  }
  static const char* value(const ::nav_msgs::GetMapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace miniros