// Generated by gencxx from file geometry_msgs/AccelStamped.msg
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

#include <std_msgs/Header.hxx>
#include <geometry_msgs/Accel.hxx>

namespace geometry_msgs
{
template <class ContainerAllocator>
struct AccelStamped_
{
  typedef AccelStamped_<ContainerAllocator> Type;

  AccelStamped_()
    : header()
    , accel()  {
    }
  AccelStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , accel(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Accel_<ContainerAllocator>  _accel_type;
  _accel_type accel;





  typedef std::shared_ptr< ::geometry_msgs::AccelStamped_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::geometry_msgs::AccelStamped_<ContainerAllocator> const> ConstPtr;

}; // struct AccelStamped_

typedef ::geometry_msgs::AccelStamped_<std::allocator<void> > AccelStamped;

typedef std::shared_ptr< ::geometry_msgs::AccelStamped > AccelStampedPtr;
typedef std::shared_ptr< ::geometry_msgs::AccelStamped const> AccelStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::AccelStamped_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::geometry_msgs::AccelStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace geometry_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': True}
// {'geometry_msgs': ['.../common_msgs/geometry_msgs/msg'], 'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::AccelStamped_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::AccelStamped_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::AccelStamped_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8a98a5d81351b6eb0578c78557e7659";
  }

  static const char* value(const ::geometry_msgs::AccelStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8a98a5d81351b6eULL;
  static const uint64_t static_value2 = 0xb0578c78557e7659ULL;
};

template<class ContainerAllocator>
struct DataType< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/AccelStamped";
  }

  static const char* value(const ::geometry_msgs::AccelStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# An accel with reference coordinate frame and timestamp\n\
Header header\n\
Accel accel\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Accel\n\
# This expresses acceleration in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::geometry_msgs::AccelStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.accel);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AccelStamped_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geometry_msgs::AccelStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geometry_msgs::AccelStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "accel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Accel_<ContainerAllocator> >::stream(s, indent + "  ", v.accel);
  }
};

} // namespace message_operations
} // namespace miniros
