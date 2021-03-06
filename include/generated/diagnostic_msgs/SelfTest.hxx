// Generated by gencxx from file diagnostic_msgs/SelfTest.msg
// DO NOT EDIT!


#pragma once

#include <miniros/traits/service_traits.h>


#include <diagnostic_msgs/SelfTestRequest.hxx>
#include <diagnostic_msgs/SelfTestResponse.hxx>


namespace diagnostic_msgs
{

struct SelfTest
{

typedef SelfTestRequest Request;
typedef SelfTestResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SelfTest
} // namespace diagnostic_msgs


namespace miniros
{
namespace service_traits
{


template<>
struct MD5Sum< ::diagnostic_msgs::SelfTest > {
  static const char* value()
  {
    return "ac21b1bab7ab17546986536c22eb34e9";
  }

  static const char* value(const ::diagnostic_msgs::SelfTest&) { return value(); }
};

template<>
struct DataType< ::diagnostic_msgs::SelfTest > {
  static const char* value()
  {
    return "diagnostic_msgs/SelfTest";
  }

  static const char* value(const ::diagnostic_msgs::SelfTest&) { return value(); }
};


// service_traits::MD5Sum< ::diagnostic_msgs::SelfTestRequest> should match 
// service_traits::MD5Sum< ::diagnostic_msgs::SelfTest > 
template<>
struct MD5Sum< ::diagnostic_msgs::SelfTestRequest>
{
  static const char* value()
  {
    return MD5Sum< ::diagnostic_msgs::SelfTest >::value();
  }
  static const char* value(const ::diagnostic_msgs::SelfTestRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::diagnostic_msgs::SelfTestRequest> should match 
// service_traits::DataType< ::diagnostic_msgs::SelfTest > 
template<>
struct DataType< ::diagnostic_msgs::SelfTestRequest>
{
  static const char* value()
  {
    return DataType< ::diagnostic_msgs::SelfTest >::value();
  }
  static const char* value(const ::diagnostic_msgs::SelfTestRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::diagnostic_msgs::SelfTestResponse> should match 
// service_traits::MD5Sum< ::diagnostic_msgs::SelfTest > 
template<>
struct MD5Sum< ::diagnostic_msgs::SelfTestResponse>
{
  static const char* value()
  {
    return MD5Sum< ::diagnostic_msgs::SelfTest >::value();
  }
  static const char* value(const ::diagnostic_msgs::SelfTestResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::diagnostic_msgs::SelfTestResponse> should match 
// service_traits::DataType< ::diagnostic_msgs::SelfTest > 
template<>
struct DataType< ::diagnostic_msgs::SelfTestResponse>
{
  static const char* value()
  {
    return DataType< ::diagnostic_msgs::SelfTest >::value();
  }
  static const char* value(const ::diagnostic_msgs::SelfTestResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace miniros
