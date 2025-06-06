// Generated by gencxx from file trajectory_msgs/JointTrajectoryPoint.msg
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


namespace trajectory_msgs
{
template <class ContainerAllocator>
struct JointTrajectoryPoint_
{
  typedef JointTrajectoryPoint_<ContainerAllocator> Type;

  JointTrajectoryPoint_()
    : positions()
    , velocities()
    , accelerations()
    , effort()
    , time_from_start()  {
    }
  JointTrajectoryPoint_(const ContainerAllocator& _alloc)
    : positions(_alloc)
    , velocities(_alloc)
    , accelerations(_alloc)
    , effort(_alloc)
    , time_from_start()  {
  (void)_alloc;
    }



   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double> > _positions_type;
  _positions_type positions;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double> > _velocities_type;
  _velocities_type velocities;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double> > _accelerations_type;
  _accelerations_type accelerations;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double> > _effort_type;
  _effort_type effort;

   typedef miniros::Duration _time_from_start_type;
  _time_from_start_type time_from_start;





  typedef std::shared_ptr< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const> ConstPtr;

}; // struct JointTrajectoryPoint_

typedef ::trajectory_msgs::JointTrajectoryPoint_<std::allocator<void> > JointTrajectoryPoint;

typedef std::shared_ptr< ::trajectory_msgs::JointTrajectoryPoint > JointTrajectoryPointPtr;
typedef std::shared_ptr< ::trajectory_msgs::JointTrajectoryPoint const> JointTrajectoryPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace trajectory_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': False}
// {'trajectory_msgs': ['.../common_msgs/trajectory_msgs/msg'], 'std_msgs': ['.../std_msgs/msg'], 'geometry_msgs': ['.../common_msgs/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f3cd1e1c4d320c79d6985c904ae5dcd3";
  }

  static const char* value(const ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf3cd1e1c4d320c79ULL;
  static const uint64_t static_value2 = 0xd6985c904ae5dcd3ULL;
};

template<class ContainerAllocator>
struct DataType< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "trajectory_msgs/JointTrajectoryPoint";
  }

  static const char* value(const ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Each trajectory point specifies either positions[, velocities[, accelerations]]\n\
# or positions[, effort] for the trajectory to be executed.\n\
# All specified values are in the same order as the joint names in JointTrajectory.msg\n\
\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
float64[] effort\n\
duration time_from_start\n\
";
  }

  static const char* value(const ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.positions);
      stream.next(m.velocities);
      stream.next(m.accelerations);
      stream.next(m.effort);
      stream.next(m.time_from_start);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointTrajectoryPoint_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>& v)
  {
    s << indent << "positions[]" << std::endl;
    for (size_t i = 0; i < v.positions.size(); ++i)
    {
      s << indent << "  positions[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.positions[i]);
    }
    s << indent << "velocities[]" << std::endl;
    for (size_t i = 0; i < v.velocities.size(); ++i)
    {
      s << indent << "  velocities[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocities[i]);
    }
    s << indent << "accelerations[]" << std::endl;
    for (size_t i = 0; i < v.accelerations.size(); ++i)
    {
      s << indent << "  accelerations[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.accelerations[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
    s << indent << "time_from_start: ";
    Printer<miniros::Duration>::stream(s, indent + "  ", v.time_from_start);
  }
};

} // namespace message_operations
} // namespace miniros
