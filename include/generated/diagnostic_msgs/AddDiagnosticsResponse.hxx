// Generated by gencxx from file diagnostic_msgs/AddDiagnosticsResponse.msg
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


namespace diagnostic_msgs
{
template <class ContainerAllocator>
struct AddDiagnosticsResponse_
{
  typedef AddDiagnosticsResponse_<ContainerAllocator> Type;

  AddDiagnosticsResponse_()
    : success(false)
    , message()  {
    }
  AddDiagnosticsResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _message_type;
  _message_type message;





  typedef std::shared_ptr< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct AddDiagnosticsResponse_

typedef ::diagnostic_msgs::AddDiagnosticsResponse_<std::allocator<void> > AddDiagnosticsResponse;

typedef std::shared_ptr< ::diagnostic_msgs::AddDiagnosticsResponse > AddDiagnosticsResponsePtr;
typedef std::shared_ptr< ::diagnostic_msgs::AddDiagnosticsResponse const> AddDiagnosticsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace diagnostic_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': False}
// {'diagnostic_msgs': ['.../common_msgs/diagnostic_msgs/msg'], 'std_msgs': ['.../std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x937c9679a518e3a1ULL;
  static const uint64_t static_value2 = 0x8d831e57125ea522ULL;
};

template<class ContainerAllocator>
struct DataType< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "diagnostic_msgs/AddDiagnosticsResponse";
  }

  static const char* value(const ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
# True if diagnostic aggregator was updated with new diagnostics, False\n\
# otherwise. A false return value means that either there is a bond in the\n\
# aggregator which already used the requested namespace, or the initialization\n\
# of analyzers failed.\n\
bool success\n\
\n\
# Message with additional information about the success or failure\n\
string message\n\
\n\
";
  }

  static const char* value(const ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.message);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddDiagnosticsResponse_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::diagnostic_msgs::AddDiagnosticsResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace miniros
