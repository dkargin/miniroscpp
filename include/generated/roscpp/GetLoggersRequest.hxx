// Generated by gencxx from file roscpp/GetLoggersRequest.msg
// DO NOT EDIT!


#pragma once


#include <string>
#include <vector>
#include <map>
#include <array>
#include <memory>

#include <miniros/types.h>
#include <miniros/serialization.h>
#include <miniros/traits/builtin_message_traits.h>
#include <miniros/message_operations.h>


namespace roscpp
{
template <class ContainerAllocator>
struct GetLoggersRequest_
{
  typedef GetLoggersRequest_<ContainerAllocator> Type;

  GetLoggersRequest_()
    {
    }
  GetLoggersRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef std::shared_ptr< ::roscpp::GetLoggersRequest_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::roscpp::GetLoggersRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetLoggersRequest_

typedef ::roscpp::GetLoggersRequest_<std::allocator<void> > GetLoggersRequest;

typedef std::shared_ptr< ::roscpp::GetLoggersRequest > GetLoggersRequestPtr;
typedef std::shared_ptr< ::roscpp::GetLoggersRequest const> GetLoggersRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp::GetLoggersRequest_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::roscpp::GetLoggersRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'roscpp': ['.../roscpp/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp::GetLoggersRequest_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp::GetLoggersRequest_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp::GetLoggersRequest_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::roscpp::GetLoggersRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp/GetLoggersRequest";
  }

  static const char* value(const ::roscpp::GetLoggersRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::roscpp::GetLoggersRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetLoggersRequest_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp::GetLoggersRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::roscpp::GetLoggersRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace miniros
