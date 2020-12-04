// Generated by gencpp from file pal_detection_msgs/Detection2d.msg
// DO NOT EDIT!


#ifndef PAL_DETECTION_MSGS_MESSAGE_DETECTION2D_H
#define PAL_DETECTION_MSGS_MESSAGE_DETECTION2D_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_detection_msgs
{
template <class ContainerAllocator>
struct Detection2d_
{
  typedef Detection2d_<ContainerAllocator> Type;

  Detection2d_()
    : x(0)
    , y(0)
    , width(0)
    , height(0)  {
    }
  Detection2d_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)
    , width(0)
    , height(0)  {
  (void)_alloc;
    }



   typedef int64_t _x_type;
  _x_type x;

   typedef int64_t _y_type;
  _y_type y;

   typedef int64_t _width_type;
  _width_type width;

   typedef int64_t _height_type;
  _height_type height;





  typedef boost::shared_ptr< ::pal_detection_msgs::Detection2d_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_detection_msgs::Detection2d_<ContainerAllocator> const> ConstPtr;

}; // struct Detection2d_

typedef ::pal_detection_msgs::Detection2d_<std::allocator<void> > Detection2d;

typedef boost::shared_ptr< ::pal_detection_msgs::Detection2d > Detection2dPtr;
typedef boost::shared_ptr< ::pal_detection_msgs::Detection2d const> Detection2dConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_detection_msgs::Detection2d_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_detection_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'pal_detection_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_detection_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_detection_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_detection_msgs::Detection2d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_detection_msgs::Detection2d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_detection_msgs::Detection2d_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da3a0b875a11573348524040e5637f14";
  }

  static const char* value(const ::pal_detection_msgs::Detection2d_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda3a0b875a115733ULL;
  static const uint64_t static_value2 = 0x48524040e5637f14ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_detection_msgs/Detection2d";
  }

  static const char* value(const ::pal_detection_msgs::Detection2d_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## Rectangle defined by a point, its width and height\n"
"# corresponds to the openCV struct : CvRect\n"
"\n"
"# coordinates of the top left corner of the box\n"
"int64 x\n"
"int64 y\n"
"\n"
"# width of the box\n"
"int64 width\n"
"\n"
"# height of the box\n"
"int64 height\n"
"\n"
;
  }

  static const char* value(const ::pal_detection_msgs::Detection2d_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.width);
      stream.next(m.height);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Detection2d_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_detection_msgs::Detection2d_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_detection_msgs::Detection2d_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<int64_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int64_t>::stream(s, indent + "  ", v.y);
    s << indent << "width: ";
    Printer<int64_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<int64_t>::stream(s, indent + "  ", v.height);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_DETECTION_MSGS_MESSAGE_DETECTION2D_H
