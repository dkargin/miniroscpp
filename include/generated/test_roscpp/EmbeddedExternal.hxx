// Generated by gencxx from file test_roscpp/EmbeddedExternal.msg
// DO NOT EDIT!


#pragma once


#include <string>
#include <vector>
#include <map>
#include <array>

#include <miniros/types.h>
#include <miniros/serialization.h>
#include <miniros/traits/builtin_message_traits.h>
#include <miniros/message_operations.h>

#include <std_msgs/UInt8.hxx>

namespace test_roscpp
{
template <class ContainerAllocator>
struct EmbeddedExternal_
{
  typedef EmbeddedExternal_<ContainerAllocator> Type;

  EmbeddedExternal_()
    : a()  {
    }
  EmbeddedExternal_(const ContainerAllocator& _alloc)
    : a(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::UInt8_<ContainerAllocator>  _a_type;
  _a_type a;





  typedef std::shared_ptr< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> const> ConstPtr;

}; // struct EmbeddedExternal_

typedef ::test_roscpp::EmbeddedExternal_<std::allocator<void> > EmbeddedExternal;

typedef std::shared_ptr< ::test_roscpp::EmbeddedExternal > EmbeddedExternalPtr;
typedef std::shared_ptr< ::test_roscpp::EmbeddedExternal const> EmbeddedExternalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::EmbeddedExternal_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace test_roscpp

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'test_roscpp': ['/home/vrobot/miniros_ws/src/test_roscpp/test/msg', '/home/vrobot/miniros_ws/src/test_roscpp/test_serialization/msg', '/home/vrobot/miniros_ws/src/test_roscpp/perf/msg', '/home/vrobot/miniros_ws/src/test_roscpp/perf_serialization/msg'], 'rosgraph_msgs': ['/opt/ros/noetic/share/rosgraph_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/noetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a09b324865f98bbf605e59ed0cefbc1d";
  }

  static const char* value(const ::test_roscpp::EmbeddedExternal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa09b324865f98bbfULL;
  static const uint64_t static_value2 = 0x605e59ed0cefbc1dULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/EmbeddedExternal";
  }

  static const char* value(const ::test_roscpp::EmbeddedExternal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/UInt8 a\n\
================================================================================\n\
MSG: std_msgs/UInt8\n\
uint8 data\n\
";
  }

  static const char* value(const ::test_roscpp::EmbeddedExternal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EmbeddedExternal_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::EmbeddedExternal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::EmbeddedExternal_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    s << std::endl;
    Printer< ::std_msgs::UInt8_<ContainerAllocator> >::stream(s, indent + "  ", v.a);
  }
};

} // namespace message_operations
} // namespace miniros
