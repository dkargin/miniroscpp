// Generated by gencxx from file test_roscpp/FixedLength.msg
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


namespace test_roscpp
{
template <class ContainerAllocator>
struct FixedLength_
{
  typedef FixedLength_<ContainerAllocator> Type;

  FixedLength_()
    : a(0)
    , b(0.0)  {
    }
  FixedLength_(const ContainerAllocator& _alloc)
    : a(0)
    , b(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _a_type;
  _a_type a;

   typedef float _b_type;
  _b_type b;





  typedef std::shared_ptr< ::test_roscpp::FixedLength_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::FixedLength_<ContainerAllocator> const> ConstPtr;

}; // struct FixedLength_

typedef ::test_roscpp::FixedLength_<std::allocator<void> > FixedLength;

typedef std::shared_ptr< ::test_roscpp::FixedLength > FixedLengthPtr;
typedef std::shared_ptr< ::test_roscpp::FixedLength const> FixedLengthConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::FixedLength_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::FixedLength_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace test_roscpp

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'test_roscpp': ['.../test_roscpp/test/msg', '.../test_roscpp/test_serialization/msg', '.../test_roscpp/perf/msg', '.../test_roscpp/perf_serialization/msg'], 'rosgraph_msgs': ['.../ros_comm_msgs/rosgraph_msgs/msg'], 'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::FixedLength_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::FixedLength_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::FixedLength_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::FixedLength_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::FixedLength_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::FixedLength_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "74143e1090cf694294f589605908b555";
  }

  static const char* value(const ::test_roscpp::FixedLength_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x74143e1090cf6942ULL;
  static const uint64_t static_value2 = 0x94f589605908b555ULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/FixedLength";
  }

  static const char* value(const ::test_roscpp::FixedLength_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 a\n\
float32 b\n\
";
  }

  static const char* value(const ::test_roscpp::FixedLength_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::FixedLength_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FixedLength_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::FixedLength_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<float>::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace miniros
