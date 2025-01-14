// Generated by gencxx from file test_roscpp/FixedLengthArrayOfExternal.msg
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

#include <rosgraph_msgs/Log.hxx>

namespace test_roscpp
{
template <class ContainerAllocator>
struct FixedLengthArrayOfExternal_
{
  typedef FixedLengthArrayOfExternal_<ContainerAllocator> Type;

  FixedLengthArrayOfExternal_()
    : a()  {
    }
  FixedLengthArrayOfExternal_(const ContainerAllocator& _alloc)
    : a()  {
  (void)_alloc;
      a.assign( ::rosgraph_msgs::Log_<ContainerAllocator> (_alloc));
  }



   typedef std::array< ::rosgraph_msgs::Log_<ContainerAllocator> , 4>  _a_type;
  _a_type a;





  typedef std::shared_ptr< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> const> ConstPtr;

}; // struct FixedLengthArrayOfExternal_

typedef ::test_roscpp::FixedLengthArrayOfExternal_<std::allocator<void> > FixedLengthArrayOfExternal;

typedef std::shared_ptr< ::test_roscpp::FixedLengthArrayOfExternal > FixedLengthArrayOfExternalPtr;
typedef std::shared_ptr< ::test_roscpp::FixedLengthArrayOfExternal const> FixedLengthArrayOfExternalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace test_roscpp

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': False}
// {'test_roscpp': ['/home/vrobot/miniros_ws/src/test_roscpp/test/msg', '/home/vrobot/miniros_ws/src/test_roscpp/test_serialization/msg', '/home/vrobot/miniros_ws/src/test_roscpp/perf/msg', '/home/vrobot/miniros_ws/src/test_roscpp/perf_serialization/msg'], 'rosgraph_msgs': ['/opt/ros/noetic/share/rosgraph_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/noetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc431047757f431ecd2754e03aa592f8";
  }

  static const char* value(const ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc431047757f431eULL;
  static const uint64_t static_value2 = 0xcd2754e03aa592f8ULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/FixedLengthArrayOfExternal";
  }

  static const char* value(const ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This comment has \"quotes\" in it and \\slashes\\\n\
rosgraph_msgs/Log[4] a\n\
================================================================================\n\
MSG: rosgraph_msgs/Log\n\
##\n\
## Severity level constants\n\
##\n\
byte DEBUG=1 #debug level\n\
byte INFO=2  #general level\n\
byte WARN=4  #warning level\n\
byte ERROR=8 #error level\n\
byte FATAL=16 #fatal/critical level\n\
##\n\
## Fields\n\
##\n\
Header header\n\
byte level\n\
string name # name of the node\n\
string msg # message \n\
string file # file the message came from\n\
string function # function the message came from\n\
uint32 line # line the message came from\n\
string[] topics # topic names that the node publishes\n\
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

  static const char* value(const ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FixedLengthArrayOfExternal_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::FixedLengthArrayOfExternal_<ContainerAllocator>& v)
  {
    s << indent << "a[]" << std::endl;
    for (size_t i = 0; i < v.a.size(); ++i)
    {
      s << indent << "  a[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rosgraph_msgs::Log_<ContainerAllocator> >::stream(s, indent + "    ", v.a[i]);
    }
  }
};

} // namespace message_operations
} // namespace miniros
