// Generated by gencpp from file pal_navigation_msgs/TabletPOI.msg
// DO NOT EDIT!


#ifndef PAL_NAVIGATION_MSGS_MESSAGE_TABLETPOI_H
#define PAL_NAVIGATION_MSGS_MESSAGE_TABLETPOI_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/String.h>

namespace pal_navigation_msgs
{
template <class ContainerAllocator>
struct TabletPOI_
{
  typedef TabletPOI_<ContainerAllocator> Type;

  TabletPOI_()
    : POIs()
    , IDs()
    , map_id()  {
    }
  TabletPOI_(const ContainerAllocator& _alloc)
    : POIs(_alloc)
    , IDs(_alloc)
    , map_id(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _POIs_type;
  _POIs_type POIs;

   typedef std::vector< ::std_msgs::String_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::String_<ContainerAllocator> >::other >  _IDs_type;
  _IDs_type IDs;

   typedef  ::std_msgs::String_<ContainerAllocator>  _map_id_type;
  _map_id_type map_id;





  typedef boost::shared_ptr< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> const> ConstPtr;

}; // struct TabletPOI_

typedef ::pal_navigation_msgs::TabletPOI_<std::allocator<void> > TabletPOI;

typedef boost::shared_ptr< ::pal_navigation_msgs::TabletPOI > TabletPOIPtr;
typedef boost::shared_ptr< ::pal_navigation_msgs::TabletPOI const> TabletPOIConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2ec4b22e981d882e9e77e8562cb8c5fe";
  }

  static const char* value(const ::pal_navigation_msgs::TabletPOI_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2ec4b22e981d882eULL;
  static const uint64_t static_value2 = 0x9e77e8562cb8c5feULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_navigation_msgs/TabletPOI";
  }

  static const char* value(const ::pal_navigation_msgs::TabletPOI_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point[] POIs\n"
"\n"
"std_msgs/String[] IDs\n"
"\n"
"std_msgs/String map_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::pal_navigation_msgs::TabletPOI_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.POIs);
      stream.next(m.IDs);
      stream.next(m.map_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TabletPOI_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_navigation_msgs::TabletPOI_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_navigation_msgs::TabletPOI_<ContainerAllocator>& v)
  {
    s << indent << "POIs[]" << std::endl;
    for (size_t i = 0; i < v.POIs.size(); ++i)
    {
      s << indent << "  POIs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.POIs[i]);
    }
    s << indent << "IDs[]" << std::endl;
    for (size_t i = 0; i < v.IDs.size(); ++i)
    {
      s << indent << "  IDs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "    ", v.IDs[i]);
    }
    s << indent << "map_id: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.map_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_NAVIGATION_MSGS_MESSAGE_TABLETPOI_H
