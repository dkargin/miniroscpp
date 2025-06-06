// Generated by gencxx from file test_roscpp/VariableLengthStringArray.msg
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
struct VariableLengthStringArray_
{
  typedef VariableLengthStringArray_<ContainerAllocator> Type;

  VariableLengthStringArray_()
    : foo()  {
    }
  VariableLengthStringArray_(const ContainerAllocator& _alloc)
    : foo(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >> > _foo_type;
  _foo_type foo;





  typedef std::shared_ptr< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> const> ConstPtr;

}; // struct VariableLengthStringArray_

typedef ::test_roscpp::VariableLengthStringArray_<std::allocator<void> > VariableLengthStringArray;

typedef std::shared_ptr< ::test_roscpp::VariableLengthStringArray > VariableLengthStringArrayPtr;
typedef std::shared_ptr< ::test_roscpp::VariableLengthStringArray const> VariableLengthStringArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >::stream(s, "", v);
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
struct IsMessage< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fa992b5cca963995275d2a2f3ee7615f";
  }

  static const char* value(const ::test_roscpp::VariableLengthStringArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfa992b5cca963995ULL;
  static const uint64_t static_value2 = 0x275d2a2f3ee7615fULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/VariableLengthStringArray";
  }

  static const char* value(const ::test_roscpp::VariableLengthStringArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] foo\n\
";
  }

  static const char* value(const ::test_roscpp::VariableLengthStringArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.foo);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VariableLengthStringArray_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::VariableLengthStringArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::VariableLengthStringArray_<ContainerAllocator>& v)
  {
    s << indent << "foo[]" << std::endl;
    for (size_t i = 0; i < v.foo.size(); ++i)
    {
      s << indent << "  foo[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.foo[i]);
    }
  }
};

} // namespace message_operations
} // namespace miniros
