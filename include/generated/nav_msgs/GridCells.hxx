// Generated by gencxx from file nav_msgs/GridCells.msg
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
#include <geometry_msgs/Point.hxx>

namespace nav_msgs
{
template <class ContainerAllocator>
struct GridCells_
{
  typedef GridCells_<ContainerAllocator> Type;

  GridCells_()
    : header()
    , cell_width(0.0)
    , cell_height(0.0)
    , cells()  {
    }
  GridCells_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cell_width(0.0)
    , cell_height(0.0)
    , cells(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _cell_width_type;
  _cell_width_type cell_width;

   typedef float _cell_height_type;
  _cell_height_type cell_height;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point_<ContainerAllocator> > > _cells_type;
  _cells_type cells;





  typedef std::shared_ptr< ::nav_msgs::GridCells_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::nav_msgs::GridCells_<ContainerAllocator> const> ConstPtr;

}; // struct GridCells_

typedef ::nav_msgs::GridCells_<std::allocator<void> > GridCells;

typedef std::shared_ptr< ::nav_msgs::GridCells > GridCellsPtr;
typedef std::shared_ptr< ::nav_msgs::GridCells const> GridCellsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav_msgs::GridCells_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::nav_msgs::GridCells_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': True}
// {'nav_msgs': ['.../common_msgs/nav_msgs/msg', '/home/vrobot/miniros_ws/devel/share/nav_msgs/msg'], 'geometry_msgs': ['.../common_msgs/geometry_msgs/msg'], 'std_msgs': ['.../std_msgs/msg'], 'actionlib_msgs': ['.../common_msgs/actionlib_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::GridCells_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_msgs::GridCells_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::GridCells_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_msgs::GridCells_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::GridCells_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_msgs::GridCells_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav_msgs::GridCells_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b9e4f5df6d28e272ebde00a3994830f5";
  }

  static const char* value(const ::nav_msgs::GridCells_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb9e4f5df6d28e272ULL;
  static const uint64_t static_value2 = 0xebde00a3994830f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::nav_msgs::GridCells_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav_msgs/GridCells";
  }

  static const char* value(const ::nav_msgs::GridCells_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav_msgs::GridCells_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#an array of cells in a 2D grid\n\
Header header\n\
float32 cell_width\n\
float32 cell_height\n\
geometry_msgs/Point[] cells\n\
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
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::nav_msgs::GridCells_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav_msgs::GridCells_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cell_width);
      stream.next(m.cell_height);
      stream.next(m.cells);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GridCells_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav_msgs::GridCells_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav_msgs::GridCells_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cell_width: ";
    Printer<float>::stream(s, indent + "  ", v.cell_width);
    s << indent << "cell_height: ";
    Printer<float>::stream(s, indent + "  ", v.cell_height);
    s << indent << "cells[]" << std::endl;
    for (size_t i = 0; i < v.cells.size(); ++i)
    {
      s << indent << "  cells[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.cells[i]);
    }
  }
};

} // namespace message_operations
} // namespace miniros
