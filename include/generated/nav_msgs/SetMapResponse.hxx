// Generated by gencxx from file nav_msgs/SetMapResponse.msg
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


namespace nav_msgs
{
template <class ContainerAllocator>
struct SetMapResponse_
{
  typedef SetMapResponse_<ContainerAllocator> Type;

  SetMapResponse_()
    : success(false)  {
    }
  SetMapResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef std::shared_ptr< ::nav_msgs::SetMapResponse_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::nav_msgs::SetMapResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetMapResponse_

typedef ::nav_msgs::SetMapResponse_<std::allocator<void> > SetMapResponse;

typedef std::shared_ptr< ::nav_msgs::SetMapResponse > SetMapResponsePtr;
typedef std::shared_ptr< ::nav_msgs::SetMapResponse const> SetMapResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav_msgs::SetMapResponse_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::nav_msgs::SetMapResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'nav_msgs': ['.../common_msgs/nav_msgs/msg', '/home/vrobot/miniros_ws/devel/share/nav_msgs/msg'], 'geometry_msgs': ['.../common_msgs/geometry_msgs/msg'], 'std_msgs': ['.../std_msgs/msg'], 'actionlib_msgs': ['.../common_msgs/actionlib_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::SetMapResponse_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::SetMapResponse_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::SetMapResponse_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::nav_msgs::SetMapResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav_msgs/SetMapResponse";
  }

  static const char* value(const ::nav_msgs::SetMapResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n\
\n\
\n\
";
  }

  static const char* value(const ::nav_msgs::SetMapResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMapResponse_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav_msgs::SetMapResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav_msgs::SetMapResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace miniros
