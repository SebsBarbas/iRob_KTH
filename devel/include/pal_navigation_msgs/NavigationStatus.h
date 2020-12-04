// Generated by gencpp from file pal_navigation_msgs/NavigationStatus.msg
// DO NOT EDIT!


#ifndef PAL_NAVIGATION_MSGS_MESSAGE_NAVIGATIONSTATUS_H
#define PAL_NAVIGATION_MSGS_MESSAGE_NAVIGATIONSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>
#include <std_msgs/String.h>

namespace pal_navigation_msgs
{
template <class ContainerAllocator>
struct NavigationStatus_
{
  typedef NavigationStatus_<ContainerAllocator> Type;

  NavigationStatus_()
    : status()
    , msg()  {
    }
  NavigationStatus_(const ContainerAllocator& _alloc)
    : status(_alloc)
    , msg(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef  ::std_msgs::String_<ContainerAllocator>  _msg_type;
  _msg_type msg;





  typedef boost::shared_ptr< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> const> ConstPtr;

}; // struct NavigationStatus_

typedef ::pal_navigation_msgs::NavigationStatus_<std::allocator<void> > NavigationStatus;

typedef boost::shared_ptr< ::pal_navigation_msgs::NavigationStatus > NavigationStatusPtr;
typedef boost::shared_ptr< ::pal_navigation_msgs::NavigationStatus const> NavigationStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_navigation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/melodic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'pal_navigation_msgs': ['/home/s/s/ssbl/catkin_ws/devel/share/pal_navigation_msgs/msg', '/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_navigation_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "37841f68121395f14fd0a4c8470eb55c";
  }

  static const char* value(const ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x37841f68121395f1ULL;
  static const uint64_t static_value2 = 0x4fd0a4c8470eb55cULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_navigation_msgs/NavigationStatus";
  }

  static const char* value(const ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/String  status\n"
"std_msgs/String  msg\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigationStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_navigation_msgs::NavigationStatus_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "msg: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_NAVIGATION_MSGS_MESSAGE_NAVIGATIONSTATUS_H
