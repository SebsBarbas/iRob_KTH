// Generated by gencpp from file pal_navigation_msgs/RenameMapRequest.msg
// DO NOT EDIT!


#ifndef PAL_NAVIGATION_MSGS_MESSAGE_RENAMEMAPREQUEST_H
#define PAL_NAVIGATION_MSGS_MESSAGE_RENAMEMAPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_navigation_msgs
{
template <class ContainerAllocator>
struct RenameMapRequest_
{
  typedef RenameMapRequest_<ContainerAllocator> Type;

  RenameMapRequest_()
    : current_map_name()
    , new_map_name()  {
    }
  RenameMapRequest_(const ContainerAllocator& _alloc)
    : current_map_name(_alloc)
    , new_map_name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _current_map_name_type;
  _current_map_name_type current_map_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _new_map_name_type;
  _new_map_name_type new_map_name;





  typedef boost::shared_ptr< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RenameMapRequest_

typedef ::pal_navigation_msgs::RenameMapRequest_<std::allocator<void> > RenameMapRequest;

typedef boost::shared_ptr< ::pal_navigation_msgs::RenameMapRequest > RenameMapRequestPtr;
typedef boost::shared_ptr< ::pal_navigation_msgs::RenameMapRequest const> RenameMapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "18e17ff8673092bf2b0ad3d839b9943d";
  }

  static const char* value(const ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x18e17ff8673092bfULL;
  static const uint64_t static_value2 = 0x2b0ad3d839b9943dULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_navigation_msgs/RenameMapRequest";
  }

  static const char* value(const ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"string current_map_name\n"
"string new_map_name\n"
;
  }

  static const char* value(const ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_map_name);
      stream.next(m.new_map_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RenameMapRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_navigation_msgs::RenameMapRequest_<ContainerAllocator>& v)
  {
    s << indent << "current_map_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.current_map_name);
    s << indent << "new_map_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.new_map_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_NAVIGATION_MSGS_MESSAGE_RENAMEMAPREQUEST_H
