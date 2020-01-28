// Generated by gencxx from file nav_msgs/SetMapRequest.msg
// DO NOT EDIT!


#pragma once


#include <string>
#include <vector>
#include <map>

#include <miniros/types.h>
#include <miniros/serialization.h>
#include <miniros/traits/builtin_message_traits.h>
#include <miniros/message_operations.h>

#include <nav_msgs/OccupancyGrid.hxx>
#include <geometry_msgs/PoseWithCovarianceStamped.hxx>

namespace nav_msgs
{
template <class ContainerAllocator>
struct SetMapRequest_
{
  typedef SetMapRequest_<ContainerAllocator> Type;

  SetMapRequest_()
    : map()
    , initial_pose()  {
    }
  SetMapRequest_(const ContainerAllocator& _alloc)
    : map(_alloc)
    , initial_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::nav_msgs::OccupancyGrid_<ContainerAllocator>  _map_type;
  _map_type map;

   typedef  ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator>  _initial_pose_type;
  _initial_pose_type initial_pose;





  typedef std::shared_ptr< ::nav_msgs::SetMapRequest_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::nav_msgs::SetMapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetMapRequest_

typedef ::nav_msgs::SetMapRequest_<std::allocator<void> > SetMapRequest;

typedef std::shared_ptr< ::nav_msgs::SetMapRequest > SetMapRequestPtr;
typedef std::shared_ptr< ::nav_msgs::SetMapRequest const> SetMapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav_msgs::SetMapRequest_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::nav_msgs::SetMapRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/home/vrobot/ros_ws/src/common_msgs/nav_msgs/msg', '/home/vrobot/ros_ws/devel/share/nav_msgs/msg'], 'std_msgs': ['/home/vrobot/ros_ws/src/std_msgs/msg'], 'actionlib_msgs': ['/home/vrobot/ros_ws/src/common_msgs/actionlib_msgs/msg'], 'geometry_msgs': ['/home/vrobot/ros_ws/src/common_msgs/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::SetMapRequest_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::SetMapRequest_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::SetMapRequest_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "91149a20d7be299b87c340df8cc94fd4";
  }

  static const char* value(const ::nav_msgs::SetMapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x91149a20d7be299bULL;
  static const uint64_t static_value2 = 0x87c340df8cc94fd4ULL;
};

template<class ContainerAllocator>
struct DataType< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav_msgs/SetMapRequest";
  }

  static const char* value(const ::nav_msgs::SetMapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
nav_msgs/OccupancyGrid map\n\
geometry_msgs/PoseWithCovarianceStamped initial_pose\n\
\n\
================================================================================\n\
MSG: nav_msgs/OccupancyGrid\n\
# This represents a 2-D grid map, in which each cell represents the probability of\n\
# occupancy.\n\
\n\
Header header \n\
\n\
#MetaData for the map\n\
MapMetaData info\n\
\n\
# The map data, in row-major order, starting with (0,0).  Occupancy\n\
# probabilities are in the range [0,100].  Unknown is -1.\n\
int8[] data\n\
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
MSG: nav_msgs/MapMetaData\n\
# This hold basic information about the characterists of the OccupancyGrid\n\
\n\
# The time at which the map was loaded\n\
time map_load_time\n\
# The map resolution [m/cell]\n\
float32 resolution\n\
# Map width [cells]\n\
uint32 width\n\
# Map height [cells]\n\
uint32 height\n\
# The origin of the map [m, m, rad].  This is the real-world pose of the\n\
# cell (0,0) in the map.\n\
geometry_msgs/Pose origin\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovarianceStamped\n\
# This expresses an estimated pose with a reference coordinate frame and timestamp\n\
\n\
Header header\n\
PoseWithCovariance pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
";
  }

  static const char* value(const ::nav_msgs::SetMapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.map);
      stream.next(m.initial_pose);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMapRequest_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav_msgs::SetMapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav_msgs::SetMapRequest_<ContainerAllocator>& v)
  {
    s << indent << "map: ";
    s << std::endl;
    Printer< ::nav_msgs::OccupancyGrid_<ContainerAllocator> >::stream(s, indent + "  ", v.map);
    s << indent << "initial_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.initial_pose);
  }
};

} // namespace message_operations
} // namespace miniros