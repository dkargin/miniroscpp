// Generated by gencxx from file test_roscpp/TestStringInt.msg
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
struct TestStringInt_
{
  typedef TestStringInt_<ContainerAllocator> Type;

  TestStringInt_()
    : str()
    , counter(0)  {
    }
  TestStringInt_(const ContainerAllocator& _alloc)
    : str(_alloc)
    , counter(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _str_type;
  _str_type str;

   typedef int32_t _counter_type;
  _counter_type counter;





  typedef std::shared_ptr< ::test_roscpp::TestStringInt_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::TestStringInt_<ContainerAllocator> const> ConstPtr;

}; // struct TestStringInt_

typedef ::test_roscpp::TestStringInt_<std::allocator<void> > TestStringInt;

typedef std::shared_ptr< ::test_roscpp::TestStringInt > TestStringIntPtr;
typedef std::shared_ptr< ::test_roscpp::TestStringInt const> TestStringIntConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::TestStringInt_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::TestStringInt_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace test_roscpp

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': False}
// {'test_roscpp': ['.../test_roscpp/test/msg', '.../test_roscpp/test_serialization/msg', '.../test_roscpp/perf/msg', '.../test_roscpp/perf_serialization/msg'], 'rosgraph_msgs': ['.../ros_comm_msgs/rosgraph_msgs/msg'], 'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::TestStringInt_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::TestStringInt_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::TestStringInt_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f0ceb8aa4bbf4dbd240039d0bf240ca";
  }

  static const char* value(const ::test_roscpp::TestStringInt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f0ceb8aa4bbf4dbULL;
  static const uint64_t static_value2 = 0xd240039d0bf240caULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/TestStringInt";
  }

  static const char* value(const ::test_roscpp::TestStringInt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string str\n\
int32 counter\n\
";
  }

  static const char* value(const ::test_roscpp::TestStringInt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::TestStringInt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.str);
      stream.next(m.counter);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestStringInt_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::TestStringInt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::TestStringInt_<ContainerAllocator>& v)
  {
    s << indent << "str: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.str);
    s << indent << "counter: ";
    Printer<int32_t>::stream(s, indent + "  ", v.counter);
  }
};

} // namespace message_operations
} // namespace miniros
