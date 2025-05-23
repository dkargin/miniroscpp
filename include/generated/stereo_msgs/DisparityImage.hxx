// Generated by gencxx from file stereo_msgs/DisparityImage.msg
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
#include <sensor_msgs/Image.hxx>
#include <sensor_msgs/RegionOfInterest.hxx>

namespace stereo_msgs
{
template <class ContainerAllocator>
struct DisparityImage_
{
  typedef DisparityImage_<ContainerAllocator> Type;

  DisparityImage_()
    : header()
    , image()
    , f(0.0)
    , T(0.0)
    , valid_window()
    , min_disparity(0.0)
    , max_disparity(0.0)
    , delta_d(0.0)  {
    }
  DisparityImage_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , image(_alloc)
    , f(0.0)
    , T(0.0)
    , valid_window(_alloc)
    , min_disparity(0.0)
    , max_disparity(0.0)
    , delta_d(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _image_type;
  _image_type image;

   typedef float _f_type;
  _f_type f;

   typedef float _T_type;
  _T_type T;

   typedef  ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  _valid_window_type;
  _valid_window_type valid_window;

   typedef float _min_disparity_type;
  _min_disparity_type min_disparity;

   typedef float _max_disparity_type;
  _max_disparity_type max_disparity;

   typedef float _delta_d_type;
  _delta_d_type delta_d;





  typedef std::shared_ptr< ::stereo_msgs::DisparityImage_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::stereo_msgs::DisparityImage_<ContainerAllocator> const> ConstPtr;

}; // struct DisparityImage_

typedef ::stereo_msgs::DisparityImage_<std::allocator<void> > DisparityImage;

typedef std::shared_ptr< ::stereo_msgs::DisparityImage > DisparityImagePtr;
typedef std::shared_ptr< ::stereo_msgs::DisparityImage const> DisparityImageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stereo_msgs::DisparityImage_<ContainerAllocator> & v)
{
miniros::message_operations::Printer< ::stereo_msgs::DisparityImage_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace stereo_msgs

namespace miniros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': True}
// {'stereo_msgs': ['.../common_msgs/stereo_msgs/msg'], 'sensor_msgs': ['.../common_msgs/sensor_msgs/msg'], 'std_msgs': ['.../std_msgs/msg'], 'geometry_msgs': ['.../common_msgs/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::stereo_msgs::DisparityImage_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stereo_msgs::DisparityImage_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::stereo_msgs::DisparityImage_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "04a177815f75271039fa21f16acad8c9";
  }

  static const char* value(const ::stereo_msgs::DisparityImage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x04a177815f752710ULL;
  static const uint64_t static_value2 = 0x39fa21f16acad8c9ULL;
};

template<class ContainerAllocator>
struct DataType< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stereo_msgs/DisparityImage";
  }

  static const char* value(const ::stereo_msgs::DisparityImage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Separate header for compatibility with current TimeSynchronizer.\n\
# Likely to be removed in a later release, use image.header instead.\n\
Header header\n\
\n\
# Floating point disparity image. The disparities are pre-adjusted for any\n\
# x-offset between the principal points of the two cameras (in the case\n\
# that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)\n\
sensor_msgs/Image image\n\
\n\
# Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.\n\
float32 f # Focal length, pixels\n\
float32 T # Baseline, world units\n\
\n\
# Subwindow of (potentially) valid disparity values.\n\
sensor_msgs/RegionOfInterest valid_window\n\
\n\
# The range of disparities searched.\n\
# In the disparity image, any disparity less than min_disparity is invalid.\n\
# The disparity search range defines the horopter, or 3D volume that the\n\
# stereo algorithm can \"see\". Points with Z outside of:\n\
#     Z_min = fT / max_disparity\n\
#     Z_max = fT / min_disparity\n\
# could not be found.\n\
float32 min_disparity\n\
float32 max_disparity\n\
\n\
# Smallest allowed disparity increment. The smallest achievable depth range\n\
# resolution is delta_Z = (Z^2/fT)*delta_d.\n\
float32 delta_d\n\
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
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of camera\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: sensor_msgs/RegionOfInterest\n\
# This message is used to specify a region of interest within an image.\n\
#\n\
# When used to specify the ROI setting of the camera when the image was\n\
# taken, the height and width fields should either match the height and\n\
# width fields for the associated image; or height = width = 0\n\
# indicates that the full resolution image was captured.\n\
\n\
uint32 x_offset  # Leftmost pixel of the ROI\n\
                 # (0 if the ROI includes the left edge of the image)\n\
uint32 y_offset  # Topmost pixel of the ROI\n\
                 # (0 if the ROI includes the top edge of the image)\n\
uint32 height    # Height of ROI\n\
uint32 width     # Width of ROI\n\
\n\
# True if a distinct rectified ROI should be calculated from the \"raw\"\n\
# ROI in this message. Typically this should be False if the full image\n\
# is captured (ROI not used), and True if a subwindow is captured (ROI\n\
# used).\n\
bool do_rectify\n\
";
  }

  static const char* value(const ::stereo_msgs::DisparityImage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace miniros

namespace miniros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.image);
      stream.next(m.f);
      stream.next(m.T);
      stream.next(m.valid_window);
      stream.next(m.min_disparity);
      stream.next(m.max_disparity);
      stream.next(m.delta_d);
    }

    MINIROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DisparityImage_

} // namespace serialization
} // namespace miniros

namespace miniros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stereo_msgs::DisparityImage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stereo_msgs::DisparityImage_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.image);
    s << indent << "f: ";
    Printer<float>::stream(s, indent + "  ", v.f);
    s << indent << "T: ";
    Printer<float>::stream(s, indent + "  ", v.T);
    s << indent << "valid_window: ";
    s << std::endl;
    Printer< ::sensor_msgs::RegionOfInterest_<ContainerAllocator> >::stream(s, indent + "  ", v.valid_window);
    s << indent << "min_disparity: ";
    Printer<float>::stream(s, indent + "  ", v.min_disparity);
    s << indent << "max_disparity: ";
    Printer<float>::stream(s, indent + "  ", v.max_disparity);
    s << indent << "delta_d: ";
    Printer<float>::stream(s, indent + "  ", v.delta_d);
  }
};

} // namespace message_operations
} // namespace miniros
