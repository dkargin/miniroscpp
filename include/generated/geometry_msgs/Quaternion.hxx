// Generated by gencxx from file geometry_msgs/Quaternion.msg
// DO NOT EDIT!


#pragma once


#include <string>
#include <vector>
#include <map>

#include <miniros/types.h>
#include <miniros/serialization.h>
#include <miniros/traits/builtin_message_traits.h>
#include <miniros/message_operations.h>


namespace geometry_msgs
{
template <class ContainerAllocator>
struct Quaternion_
{
  typedef Quaternion_<ContainerAllocator> Type;

  Quaternion_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , w(0.0)  {
    }
  Quaternion_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , w(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _w_type;
  _w_type w;





  typedef std::shared_ptr< ::geometry_msgs::Quaternion_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::geometry_msgs::Quaternion_<ContainerAllocator> const> ConstPtr;

}; // struct Quaternion_

typedef ::geometry_msgs::Quaternion_<std::allocator<void> > Quaternion;

typedef std::shared_ptr< ::geometry_msgs::Quaternion > QuaternionPtr;
typedef std::shared_ptr< ::geometry_msgs::Quaternion const> QuaternionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Quaternion_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace geometry_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/home/vrobot/ros_ws/src/std_msgs/msg'], 'geometry_msgs': ['/home/vrobot/ros_ws/src/common_msgs/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::Quaternion_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::Quaternion_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::Quaternion_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::Quaternion_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::Quaternion_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::Quaternion_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geometry_msgs::Quaternion_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a779879fadf0160734f906b8c19c7004";
  }

  static const char* value(const ::geometry_msgs::Quaternion_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa779879fadf01607ULL;
  static const uint64_t static_value2 = 0x34f906b8c19c7004ULL;
};

template<class ContainerAllocator>
struct DataType< ::geometry_msgs::Quaternion_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Quaternion";
  }

  static const char* value(const ::geometry_msgs::Quaternion_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geometry_msgs::Quaternion_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::geometry_msgs::Quaternion_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geometry_msgs::Quaternion_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.w);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Quaternion_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geometry_msgs::Quaternion_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "w: ";
    Printer<double>::stream(s, indent + "  ", v.w);
  }
};

} // namespace message_operations
} // namespace miniros
