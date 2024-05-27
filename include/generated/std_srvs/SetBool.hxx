// Generated by gencxx from file std_srvs/SetBool.msg
// DO NOT EDIT!


#pragma once

#include <miniros/traits/service_traits.h>


#include <std_srvs/SetBoolRequest.hxx>
#include <std_srvs/SetBoolResponse.hxx>


namespace std_srvs
{

struct SetBool
{

typedef SetBoolRequest Request;
typedef SetBoolResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetBool
} // namespace std_srvs


namespace miniros
{
namespace service_traits
{


template<>
struct MD5Sum< ::std_srvs::SetBool > {
  static const char* value()
  {
    return "09fb03525b03e7ea1fd3992bafd87e16";
  }

  static const char* value(const ::std_srvs::SetBool&) { return value(); }
};

template<>
struct DataType< ::std_srvs::SetBool > {
  static const char* value()
  {
    return "std_srvs/SetBool";
  }

  static const char* value(const ::std_srvs::SetBool&) { return value(); }
};


// service_traits::MD5Sum< ::std_srvs::SetBoolRequest> should match 
// service_traits::MD5Sum< ::std_srvs::SetBool > 
template<>
struct MD5Sum< ::std_srvs::SetBoolRequest>
{
  static const char* value()
  {
    return MD5Sum< ::std_srvs::SetBool >::value();
  }
  static const char* value(const ::std_srvs::SetBoolRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::std_srvs::SetBoolRequest> should match 
// service_traits::DataType< ::std_srvs::SetBool > 
template<>
struct DataType< ::std_srvs::SetBoolRequest>
{
  static const char* value()
  {
    return DataType< ::std_srvs::SetBool >::value();
  }
  static const char* value(const ::std_srvs::SetBoolRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::std_srvs::SetBoolResponse> should match 
// service_traits::MD5Sum< ::std_srvs::SetBool > 
template<>
struct MD5Sum< ::std_srvs::SetBoolResponse>
{
  static const char* value()
  {
    return MD5Sum< ::std_srvs::SetBool >::value();
  }
  static const char* value(const ::std_srvs::SetBoolResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::std_srvs::SetBoolResponse> should match 
// service_traits::DataType< ::std_srvs::SetBool > 
template<>
struct DataType< ::std_srvs::SetBoolResponse>
{
  static const char* value()
  {
    return DataType< ::std_srvs::SetBool >::value();
  }
  static const char* value(const ::std_srvs::SetBoolResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace miniros