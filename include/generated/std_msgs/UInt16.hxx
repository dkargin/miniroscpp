// Generated by gencxx from file std_msgs/UInt16.msg
// DO NOT EDIT!


#pragma once


#include <string>
#include <vector>
#include <map>

#include <miniros/types.h>
#include <miniros/serialization.h>
#include <miniros/traits/builtin_message_traits.h>
#include <miniros/message_operations.h>


namespace std_msgs
{
template <class ContainerAllocator>
struct UInt16_
{
  typedef UInt16_<ContainerAllocator> Type;

  UInt16_()
    : data(0)  {
    }
  UInt16_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef uint16_t _data_type;
  _data_type data;





  typedef std::shared_ptr< ::std_msgs::UInt16_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::std_msgs::UInt16_<ContainerAllocator> const> ConstPtr;

}; // struct UInt16_

typedef ::std_msgs::UInt16_<std::allocator<void> > UInt16;

typedef std::shared_ptr< ::std_msgs::UInt16 > UInt16Ptr;
typedef std::shared_ptr< ::std_msgs::UInt16 const> UInt16ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::UInt16_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::std_msgs::UInt16_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace std_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/home/vrobot/ros_ws/src/std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::UInt16_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::UInt16_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::UInt16_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::UInt16_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::UInt16_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::UInt16_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::UInt16_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1df79edf208b629fe6b81923a544552d";
  }

  static const char* value(const ::std_msgs::UInt16_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1df79edf208b629fULL;
  static const uint64_t static_value2 = 0xe6b81923a544552dULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::UInt16_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/UInt16";
  }

  static const char* value(const ::std_msgs::UInt16_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::UInt16_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 data\n\
";
  }

  static const char* value(const ::std_msgs::UInt16_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::UInt16_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UInt16_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::UInt16_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::UInt16_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace miniros