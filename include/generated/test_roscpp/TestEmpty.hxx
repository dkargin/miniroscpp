// Generated by gencxx from file test_roscpp/TestEmpty.msg
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
struct TestEmpty_
{
  typedef TestEmpty_<ContainerAllocator> Type;

  TestEmpty_()
    {
    }
  TestEmpty_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef std::shared_ptr< ::test_roscpp::TestEmpty_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::TestEmpty_<ContainerAllocator> const> ConstPtr;

}; // struct TestEmpty_

typedef ::test_roscpp::TestEmpty_<std::allocator<void> > TestEmpty;

typedef std::shared_ptr< ::test_roscpp::TestEmpty > TestEmptyPtr;
typedef std::shared_ptr< ::test_roscpp::TestEmpty const> TestEmptyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::TestEmpty_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::TestEmpty_<ContainerAllocator> >::stream(s, "", v);
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
struct IsMessage< ::test_roscpp::TestEmpty_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::TestEmpty_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::TestEmpty_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::TestEmpty_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::TestEmpty_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::TestEmpty_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::TestEmpty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::test_roscpp::TestEmpty_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::TestEmpty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/TestEmpty";
  }

  static const char* value(const ::test_roscpp::TestEmpty_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::TestEmpty_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::test_roscpp::TestEmpty_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::TestEmpty_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestEmpty_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::TestEmpty_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::test_roscpp::TestEmpty_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace miniros
