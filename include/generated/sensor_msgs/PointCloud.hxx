// Generated by gencxx from file sensor_msgs/PointCloud.msg
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
#include <geometry_msgs/Point32.hxx>
#include <sensor_msgs/ChannelFloat32.hxx>

namespace sensor_msgs
{
template <class ContainerAllocator>
struct PointCloud_
{
  typedef PointCloud_<ContainerAllocator> Type;

  PointCloud_()
    : header()
    , points()
    , channels()  {
    }
  PointCloud_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , points(_alloc)
    , channels(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point32_<ContainerAllocator> > > _points_type;
  _points_type points;

   typedef std::vector< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> > > _channels_type;
  _channels_type channels;





  typedef std::shared_ptr< ::sensor_msgs::PointCloud_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::sensor_msgs::PointCloud_<ContainerAllocator> const> ConstPtr;

}; // struct PointCloud_

typedef ::sensor_msgs::PointCloud_<std::allocator<void> > PointCloud;

typedef std::shared_ptr< ::sensor_msgs::PointCloud > PointCloudPtr;
typedef std::shared_ptr< ::sensor_msgs::PointCloud const> PointCloudConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor_msgs::PointCloud_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::sensor_msgs::PointCloud_<ContainerAllocator> >::stream(s, "", v);
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
struct IsMessage< ::sensor_msgs::PointCloud_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor_msgs::PointCloud_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::PointCloud_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor_msgs::PointCloud_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::PointCloud_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor_msgs::PointCloud_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensor_msgs::PointCloud_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8e9c3f5afbdd8a130fd1d2763945fca";
  }

  static const char* value(const ::sensor_msgs::PointCloud_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8e9c3f5afbdd8a1ULL;
  static const uint64_t static_value2 = 0x30fd1d2763945fcaULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor_msgs::PointCloud_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/PointCloud";
  }

  static const char* value(const ::sensor_msgs::PointCloud_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor_msgs::PointCloud_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message holds a collection of 3d points, plus optional additional\n\
# information about each point.\n\
\n\
# Time of sensor data acquisition, coordinate frame ID.\n\
Header header\n\
\n\
# Array of 3d points. Each Point32 should be interpreted as a 3d point\n\
# in the frame given in the header.\n\
geometry_msgs/Point32[] points\n\
\n\
# Each channel should have the same number of elements as points array,\n\
# and the data in each channel should correspond 1:1 with each point.\n\
# Channel names in common practice are listed in ChannelFloat32.msg.\n\
ChannelFloat32[] channels\n\
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
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
================================================================================\n\
MSG: sensor_msgs/ChannelFloat32\n\
# This message is used by the PointCloud message to hold optional data\n\
# associated with each point in the cloud. The length of the values\n\
# array should be the same as the length of the points array in the\n\
# PointCloud, and each value should be associated with the corresponding\n\
# point.\n\
\n\
# Channel names in existing practice include:\n\
#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n\
#              This is opposite to usual conventions but remains for\n\
#              historical reasons. The newer PointCloud2 message has no\n\
#              such problem.\n\
#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n\
#           (R,G,B) values packed into the least significant 24 bits,\n\
#           in order.\n\
#   \"intensity\" - laser or pixel intensity.\n\
#   \"distance\"\n\
\n\
# The channel name should give semantics of the channel (e.g.\n\
# \"intensity\" instead of \"value\").\n\
string name\n\
\n\
# The values array should be 1-1 with the elements of the associated\n\
# PointCloud.\n\
float32[] values\n\
";
  }

  static const char* value(const ::sensor_msgs::PointCloud_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensor_msgs::PointCloud_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.points);
      stream.next(m.channels);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointCloud_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor_msgs::PointCloud_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor_msgs::PointCloud_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "channels[]" << std::endl;
    for (size_t i = 0; i < v.channels.size(); ++i)
    {
      s << indent << "  channels[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> >::stream(s, indent + "    ", v.channels[i]);
    }
  }
};

} // namespace message_operations
} // namespace miniros
