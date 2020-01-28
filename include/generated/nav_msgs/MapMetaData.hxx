// Generated by gencxx from file nav_msgs/MapMetaData.msg
// DO NOT EDIT!


#pragma once


#include <string>
#include <vector>
#include <map>

#include <miniros/types.h>
#include <miniros/serialization.h>
#include <miniros/traits/builtin_message_traits.h>
#include <miniros/message_operations.h>

#include <geometry_msgs/Pose.hxx>

namespace nav_msgs
{
template <class ContainerAllocator>
struct MapMetaData_
{
  typedef MapMetaData_<ContainerAllocator> Type;

  MapMetaData_()
    : map_load_time()
    , resolution(0.0)
    , width(0)
    , height(0)
    , origin()  {
    }
  MapMetaData_(const ContainerAllocator& _alloc)
    : map_load_time()
    , resolution(0.0)
    , width(0)
    , height(0)
    , origin(_alloc)  {
  (void)_alloc;
    }



   typedef miniros::Time _map_load_time_type;
  _map_load_time_type map_load_time;

   typedef float _resolution_type;
  _resolution_type resolution;

   typedef uint32_t _width_type;
  _width_type width;

   typedef uint32_t _height_type;
  _height_type height;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _origin_type;
  _origin_type origin;





  typedef std::shared_ptr< ::nav_msgs::MapMetaData_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::nav_msgs::MapMetaData_<ContainerAllocator> const> ConstPtr;

}; // struct MapMetaData_

typedef ::nav_msgs::MapMetaData_<std::allocator<void> > MapMetaData;

typedef std::shared_ptr< ::nav_msgs::MapMetaData > MapMetaDataPtr;
typedef std::shared_ptr< ::nav_msgs::MapMetaData const> MapMetaDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav_msgs::MapMetaData_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::nav_msgs::MapMetaData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/home/vrobot/ros_ws/src/common_msgs/nav_msgs/msg', '/home/vrobot/ros_ws/devel/share/nav_msgs/msg'], 'std_msgs': ['/home/vrobot/ros_ws/src/std_msgs/msg'], 'actionlib_msgs': ['/home/vrobot/ros_ws/src/common_msgs/actionlib_msgs/msg'], 'geometry_msgs': ['/home/vrobot/ros_ws/src/common_msgs/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::MapMetaData_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::MapMetaData_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::MapMetaData_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::MapMetaData_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::MapMetaData_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::MapMetaData_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav_msgs::MapMetaData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10cfc8a2818024d3248802c00c95f11b";
  }

  static const char* value(const ::nav_msgs::MapMetaData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x10cfc8a2818024d3ULL;
  static const uint64_t static_value2 = 0x248802c00c95f11bULL;
};

template<class ContainerAllocator>
struct DataType< ::nav_msgs::MapMetaData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav_msgs/MapMetaData";
  }

  static const char* value(const ::nav_msgs::MapMetaData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav_msgs::MapMetaData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This hold basic information about the characterists of the OccupancyGrid\n\
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
";
  }

  static const char* value(const ::nav_msgs::MapMetaData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav_msgs::MapMetaData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.map_load_time);
      stream.next(m.resolution);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.origin);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MapMetaData_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav_msgs::MapMetaData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav_msgs::MapMetaData_<ContainerAllocator>& v)
  {
    s << indent << "map_load_time: ";
    Printer<miniros::Time>::stream(s, indent + "  ", v.map_load_time);
    s << indent << "resolution: ";
    Printer<float>::stream(s, indent + "  ", v.resolution);
    s << indent << "width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.height);
    s << indent << "origin: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.origin);
  }
};

} // namespace message_operations
} // namespace miniros
