// Generated by gencxx from file std_srvs/Empty.msg
// DO NOT EDIT!


#pragma once

#include <miniros/traits/service_traits.h>


#include <std_srvs/EmptyRequest.hxx>
#include <std_srvs/EmptyResponse.hxx>


namespace std_srvs
{

struct Empty
{

typedef EmptyRequest Request;
typedef EmptyResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Empty
} // namespace std_srvs


namespace miniros
{
namespace service_traits
{


template<>
struct MD5Sum< ::std_srvs::Empty > {
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::std_srvs::Empty&) { return value(); }
};

template<>
struct DataType< ::std_srvs::Empty > {
  static const char* value()
  {
    return "std_srvs/Empty";
  }

  static const char* value(const ::std_srvs::Empty&) { return value(); }
};


// service_traits::MD5Sum< ::std_srvs::EmptyRequest> should match 
// service_traits::MD5Sum< ::std_srvs::Empty > 
template<>
struct MD5Sum< ::std_srvs::EmptyRequest>
{
  static const char* value()
  {
    return MD5Sum< ::std_srvs::Empty >::value();
  }
  static const char* value(const ::std_srvs::EmptyRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::std_srvs::EmptyRequest> should match 
// service_traits::DataType< ::std_srvs::Empty > 
template<>
struct DataType< ::std_srvs::EmptyRequest>
{
  static const char* value()
  {
    return DataType< ::std_srvs::Empty >::value();
  }
  static const char* value(const ::std_srvs::EmptyRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::std_srvs::EmptyResponse> should match 
// service_traits::MD5Sum< ::std_srvs::Empty > 
template<>
struct MD5Sum< ::std_srvs::EmptyResponse>
{
  static const char* value()
  {
    return MD5Sum< ::std_srvs::Empty >::value();
  }
  static const char* value(const ::std_srvs::EmptyResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::std_srvs::EmptyResponse> should match 
// service_traits::DataType< ::std_srvs::Empty > 
template<>
struct DataType< ::std_srvs::EmptyResponse>
{
  static const char* value()
  {
    return DataType< ::std_srvs::Empty >::value();
  }
  static const char* value(const ::std_srvs::EmptyResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace miniros