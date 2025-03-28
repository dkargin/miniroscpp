// Generated by gencxx from file sensor_msgs/TimeReference.msg
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

namespace sensor_msgs
{
template <class ContainerAllocator>
struct TimeReference_
{
  typedef TimeReference_<ContainerAllocator> Type;

  TimeReference_()
    : header()
    , time_ref()
    , source()  {
    }
  TimeReference_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time_ref()
    , source(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef miniros::Time _time_ref_type;
  _time_ref_type time_ref;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _source_type;
  _source_type source;





  typedef std::shared_ptr< ::sensor_msgs::TimeReference_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::sensor_msgs::TimeReference_<ContainerAllocator> const> ConstPtr;

}; // struct TimeReference_

typedef ::sensor_msgs::TimeReference_<std::allocator<void> > TimeReference;

typedef std::shared_ptr< ::sensor_msgs::TimeReference > TimeReferencePtr;
typedef std::shared_ptr< ::sensor_msgs::TimeReference const> TimeReferenceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_msgs::TimeReference_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::sensor_msgs::TimeReference_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sensor_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': True}
// {'sensor_msgs': ['.../common_msgs/sensor_msgs/msg'], 'geometry_msgs': ['.../common_msgs/geometry_msgs/msg'], 'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::TimeReference_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::TimeReference_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::TimeReference_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::TimeReference_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::TimeReference_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::TimeReference_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensor_msgs::TimeReference_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fded64a0265108ba86c3d38fb11c0c16";
  }

  static const char* value(const ::sensor_msgs::TimeReference_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfded64a0265108baULL;
  static const uint64_t static_value2 = 0x86c3d38fb11c0c16ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_msgs::TimeReference_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/TimeReference";
  }

  static const char* value(const ::sensor_msgs::TimeReference_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_msgs::TimeReference_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Measurement from an external time source not actively synchronized with the system clock.\n\
\n\
Header header    # stamp is system time for which measurement was valid\n\
                 # frame_id is not used \n\
\n\
time   time_ref  # corresponding time from this external source\n\
string source    # (optional) name of time source\n\
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
";
  }

  static const char* value(const ::sensor_msgs::TimeReference_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensor_msgs::TimeReference_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time_ref);
      stream.next(m.source);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimeReference_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_msgs::TimeReference_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_msgs::TimeReference_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time_ref: ";
    Printer<miniros::Time>::stream(s, indent + "  ", v.time_ref);
    s << indent << "source: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.source);
  }
};

} // namespace message_operations
} // namespace miniros
