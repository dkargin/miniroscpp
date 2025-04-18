// Generated by gencxx from file actionlib_msgs/GoalStatusArray.msg
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
#include <actionlib_msgs/GoalStatus.hxx>

namespace actionlib_msgs
{
template <class ContainerAllocator>
struct GoalStatusArray_
{
  typedef GoalStatusArray_<ContainerAllocator> Type;

  GoalStatusArray_()
    : header()
    , status_list()  {
    }
  GoalStatusArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status_list(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::actionlib_msgs::GoalStatus_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::actionlib_msgs::GoalStatus_<ContainerAllocator> > > _status_list_type;
  _status_list_type status_list;





  typedef std::shared_ptr< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> const> ConstPtr;

}; // struct GoalStatusArray_

typedef ::actionlib_msgs::GoalStatusArray_<std::allocator<void> > GoalStatusArray;

typedef std::shared_ptr< ::actionlib_msgs::GoalStatusArray > GoalStatusArrayPtr;
typedef std::shared_ptr< ::actionlib_msgs::GoalStatusArray const> GoalStatusArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace actionlib_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': True}
// {'actionlib_msgs': ['.../common_msgs/actionlib_msgs/msg'], 'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b2b82f13216d0a8ea88bd3af735e619";
  }

  static const char* value(const ::actionlib_msgs::GoalStatusArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b2b82f13216d0a8ULL;
  static const uint64_t static_value2 = 0xea88bd3af735e619ULL;
};

template<class ContainerAllocator>
struct DataType< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "actionlib_msgs/GoalStatusArray";
  }

  static const char* value(const ::actionlib_msgs::GoalStatusArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Stores the statuses for goals that are currently being tracked\n\
# by an action server\n\
Header header\n\
GoalStatus[] status_list\n\
\n\
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
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
";
  }

  static const char* value(const ::actionlib_msgs::GoalStatusArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status_list);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalStatusArray_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::actionlib_msgs::GoalStatusArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::actionlib_msgs::GoalStatusArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status_list[]" << std::endl;
    for (size_t i = 0; i < v.status_list.size(); ++i)
    {
      s << indent << "  status_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::actionlib_msgs::GoalStatus_<ContainerAllocator> >::stream(s, indent + "    ", v.status_list[i]);
    }
  }
};

} // namespace message_operations
} // namespace miniros
