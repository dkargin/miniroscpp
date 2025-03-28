// Generated by gencxx from file roscpp/SetLoggerLevelRequest.msg
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


namespace roscpp
{
template <class ContainerAllocator>
struct SetLoggerLevelRequest_
{
  typedef SetLoggerLevelRequest_<ContainerAllocator> Type;

  SetLoggerLevelRequest_()
    : logger()
    , level()  {
    }
  SetLoggerLevelRequest_(const ContainerAllocator& _alloc)
    : logger(_alloc)
    , level(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _logger_type;
  _logger_type logger;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _level_type;
  _level_type level;





  typedef std::shared_ptr< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetLoggerLevelRequest_

typedef ::roscpp::SetLoggerLevelRequest_<std::allocator<void> > SetLoggerLevelRequest;

typedef std::shared_ptr< ::roscpp::SetLoggerLevelRequest > SetLoggerLevelRequestPtr;
typedef std::shared_ptr< ::roscpp::SetLoggerLevelRequest const> SetLoggerLevelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': False}
// {'roscpp': ['.../roscpp/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "51da076440d78ca1684d36c868df61ea";
  }

  static const char* value(const ::roscpp::SetLoggerLevelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x51da076440d78ca1ULL;
  static const uint64_t static_value2 = 0x684d36c868df61eaULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp/SetLoggerLevelRequest";
  }

  static const char* value(const ::roscpp::SetLoggerLevelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string logger\n\
string level\n\
";
  }

  static const char* value(const ::roscpp::SetLoggerLevelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.logger);
      stream.next(m.level);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetLoggerLevelRequest_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp::SetLoggerLevelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp::SetLoggerLevelRequest_<ContainerAllocator>& v)
  {
    s << indent << "logger: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.logger);
    s << indent << "level: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.level);
  }
};

} // namespace message_operations
} // namespace miniros
