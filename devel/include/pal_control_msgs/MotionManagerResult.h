// Generated by gencpp from file pal_control_msgs/MotionManagerResult.msg
// DO NOT EDIT!


#ifndef PAL_CONTROL_MSGS_MESSAGE_MOTIONMANAGERRESULT_H
#define PAL_CONTROL_MSGS_MESSAGE_MOTIONMANAGERRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_control_msgs
{
template <class ContainerAllocator>
struct MotionManagerResult_
{
  typedef MotionManagerResult_<ContainerAllocator> Type;

  MotionManagerResult_()
    : message()  {
    }
  MotionManagerResult_(const ContainerAllocator& _alloc)
    : message(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> const> ConstPtr;

}; // struct MotionManagerResult_

typedef ::pal_control_msgs::MotionManagerResult_<std::allocator<void> > MotionManagerResult;

typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerResult > MotionManagerResultPtr;
typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerResult const> MotionManagerResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_control_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'pal_control_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_control_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_control_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5f003d6bcc824cbd51361d66d8e4f76c";
  }

  static const char* value(const ::pal_control_msgs::MotionManagerResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5f003d6bcc824cbdULL;
  static const uint64_t static_value2 = 0x51361d66d8e4f76cULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_control_msgs/MotionManagerResult";
  }

  static const char* value(const ::pal_control_msgs::MotionManagerResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#Message in result, can contain information if goal failed\n"
"string message\n"
;
  }

  static const char* value(const ::pal_control_msgs::MotionManagerResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionManagerResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_control_msgs::MotionManagerResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_control_msgs::MotionManagerResult_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_CONTROL_MSGS_MESSAGE_MOTIONMANAGERRESULT_H
