// Generated by gencxx from file roscpp/Empty.msg
// DO NOT EDIT!


#pragma once

#include <miniros/traits/service_traits.h>


#include <roscpp/EmptyRequest.hxx>
#include <roscpp/EmptyResponse.hxx>


namespace roscpp
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
} // namespace roscpp


namespace miniros
{
namespace service_traits
{


template<>
struct MD5Sum< ::roscpp::Empty > {
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::roscpp::Empty&) { return value(); }
};

template<>
struct DataType< ::roscpp::Empty > {
  static const char* value()
  {
    return "roscpp/Empty";
  }

  static const char* value(const ::roscpp::Empty&) { return value(); }
};


// service_traits::MD5Sum< ::roscpp::EmptyRequest> should match 
// service_traits::MD5Sum< ::roscpp::Empty > 
template<>
struct MD5Sum< ::roscpp::EmptyRequest>
{
  static const char* value()
  {
    return MD5Sum< ::roscpp::Empty >::value();
  }
  static const char* value(const ::roscpp::EmptyRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::roscpp::EmptyRequest> should match 
// service_traits::DataType< ::roscpp::Empty > 
template<>
struct DataType< ::roscpp::EmptyRequest>
{
  static const char* value()
  {
    return DataType< ::roscpp::Empty >::value();
  }
  static const char* value(const ::roscpp::EmptyRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::roscpp::EmptyResponse> should match 
// service_traits::MD5Sum< ::roscpp::Empty > 
template<>
struct MD5Sum< ::roscpp::EmptyResponse>
{
  static const char* value()
  {
    return MD5Sum< ::roscpp::Empty >::value();
  }
  static const char* value(const ::roscpp::EmptyResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::roscpp::EmptyResponse> should match 
// service_traits::DataType< ::roscpp::Empty > 
template<>
struct DataType< ::roscpp::EmptyResponse>
{
  static const char* value()
  {
    return DataType< ::roscpp::Empty >::value();
  }
  static const char* value(const ::roscpp::EmptyResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace miniros
