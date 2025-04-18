// Generated by gencxx from file roscpp/Logger.msg
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
struct Logger_
{
  typedef Logger_<ContainerAllocator> Type;

  Logger_()
    : name()
    , level()  {
    }
  Logger_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , level(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _name_type;
  _name_type name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> > _level_type;
  _level_type level;





  typedef std::shared_ptr< ::roscpp::Logger_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::roscpp::Logger_<ContainerAllocator> const> ConstPtr;

}; // struct Logger_

typedef ::roscpp::Logger_<std::allocator<void> > Logger;

typedef std::shared_ptr< ::roscpp::Logger > LoggerPtr;
typedef std::shared_ptr< ::roscpp::Logger const> LoggerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp::Logger_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::roscpp::Logger_<ContainerAllocator> >::stream(s, "", v);
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
struct IsMessage< ::roscpp::Logger_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp::Logger_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp::Logger_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp::Logger_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp::Logger_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp::Logger_<ContainerAllocator> const>
  : std::false_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp::Logger_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6069a2ff40db7bd32143dd66e1f408e";
  }

  static const char* value(const ::roscpp::Logger_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6069a2ff40db7bdULL;
  static const uint64_t static_value2 = 0x32143dd66e1f408eULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp::Logger_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp/Logger";
  }

  static const char* value(const ::roscpp::Logger_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp::Logger_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n\
string level\n\
";
  }

  static const char* value(const ::roscpp::Logger_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp::Logger_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.level);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Logger_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp::Logger_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp::Logger_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.name);
    s << indent << "level: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char> >>::stream(s, indent + "  ", v.level);
  }
};

} // namespace message_operations
} // namespace miniros
