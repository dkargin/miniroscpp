// Generated by gencxx from file test_roscpp/ArrayOfFixedLength.msg
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

#include <test_roscpp/FixedLength.hxx>

namespace test_roscpp
{
template <class ContainerAllocator>
struct ArrayOfFixedLength_
{
  typedef ArrayOfFixedLength_<ContainerAllocator> Type;

  ArrayOfFixedLength_()
    : a()  {
    }
  ArrayOfFixedLength_(const ContainerAllocator& _alloc)
    : a()  {
  (void)_alloc;
      a.assign( ::test_roscpp::FixedLength_<ContainerAllocator> (_alloc));
  }



   typedef std::array< ::test_roscpp::FixedLength_<ContainerAllocator> , 4>  _a_type;
  _a_type a;





  typedef std::shared_ptr< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> const> ConstPtr;

}; // struct ArrayOfFixedLength_

typedef ::test_roscpp::ArrayOfFixedLength_<std::allocator<void> > ArrayOfFixedLength;

typedef std::shared_ptr< ::test_roscpp::ArrayOfFixedLength > ArrayOfFixedLengthPtr;
typedef std::shared_ptr< ::test_roscpp::ArrayOfFixedLength const> ArrayOfFixedLengthConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >::stream(s, "", v);
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
struct IsMessage< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "770e15962592d1fbea70b6b820ba16d9";
  }

  static const char* value(const ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x770e15962592d1fbULL;
  static const uint64_t static_value2 = 0xea70b6b820ba16d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/ArrayOfFixedLength";
  }

  static const char* value(const ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This comment has \"quotes\" in it\n\
FixedLength[4] a\n\
================================================================================\n\
MSG: test_roscpp/FixedLength\n\
uint32 a\n\
float32 b\n\
";
  }

  static const char* value(const ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArrayOfFixedLength_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::ArrayOfFixedLength_<ContainerAllocator>& v)
  {
    s << indent << "a[]" << std::endl;
    for (size_t i = 0; i < v.a.size(); ++i)
    {
      s << indent << "  a[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::test_roscpp::FixedLength_<ContainerAllocator> >::stream(s, indent + "    ", v.a[i]);
    }
  }
};

} // namespace message_operations
} // namespace miniros
