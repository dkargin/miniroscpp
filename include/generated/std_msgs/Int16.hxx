// Generated by gencxx from file std_msgs/Int16.msg
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


namespace std_msgs
{
template <class ContainerAllocator>
struct Int16_
{
  typedef Int16_<ContainerAllocator> Type;

  Int16_()
    : data(0)  {
    }
  Int16_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef int16_t _data_type;
  _data_type data;





  typedef std::shared_ptr< ::std_msgs::Int16_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::std_msgs::Int16_<ContainerAllocator> const> ConstPtr;

}; // struct Int16_

typedef ::std_msgs::Int16_<std::allocator<void> > Int16;

typedef std::shared_ptr< ::std_msgs::Int16 > Int16Ptr;
typedef std::shared_ptr< ::std_msgs::Int16 const> Int16ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Int16_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::std_msgs::Int16_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace std_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Int16_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Int16_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Int16_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Int16_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Int16_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Int16_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Int16_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8524586e34fbd7cb1c08c5f5f1ca0e57";
  }

  static const char* value(const ::std_msgs::Int16_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8524586e34fbd7cbULL;
  static const uint64_t static_value2 = 0x1c08c5f5f1ca0e57ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Int16_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Int16";
  }

  static const char* value(const ::std_msgs::Int16_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Int16_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 data\n\
";
  }

  static const char* value(const ::std_msgs::Int16_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::Int16_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Int16_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Int16_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::Int16_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<int16_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace miniros
