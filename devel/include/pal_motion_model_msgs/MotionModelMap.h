// Generated by gencpp from file pal_motion_model_msgs/MotionModelMap.msg
// DO NOT EDIT!


#ifndef PAL_MOTION_MODEL_MSGS_MESSAGE_MOTIONMODELMAP_H
#define PAL_MOTION_MODEL_MSGS_MESSAGE_MOTIONMODELMAP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <pal_motion_model_msgs/MotionModelList.h>

namespace pal_motion_model_msgs
{
template <class ContainerAllocator>
struct MotionModelMap_
{
  typedef MotionModelMap_<ContainerAllocator> Type;

  MotionModelMap_()
    : header()
    , info()
    , data()  {
    }
  MotionModelMap_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , info(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::nav_msgs::MapMetaData_<ContainerAllocator>  _info_type;
  _info_type info;

   typedef std::vector< ::pal_motion_model_msgs::MotionModelList_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pal_motion_model_msgs::MotionModelList_<ContainerAllocator> >::other >  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> const> ConstPtr;

}; // struct MotionModelMap_

typedef ::pal_motion_model_msgs::MotionModelMap_<std::allocator<void> > MotionModelMap;

typedef boost::shared_ptr< ::pal_motion_model_msgs::MotionModelMap > MotionModelMapPtr;
typedef boost::shared_ptr< ::pal_motion_model_msgs::MotionModelMap const> MotionModelMapConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_motion_model_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/melodic/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'pal_motion_model_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_motion_model_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b6bcccd2088596e98ba83bc65b2b504e";
  }

  static const char* value(const ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb6bcccd2088596e9ULL;
  static const uint64_t static_value2 = 0x8ba83bc65b2b504eULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_motion_model_msgs/MotionModelMap";
  }

  static const char* value(const ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#This represents a 2-D motion model map, in which each cell has the probabilistic motion models for the robot.\n"
"\n"
"Header header \n"
"\n"
"#MetaData for the map\n"
"nav_msgs/MapMetaData info\n"
"\n"
"# The map data, in row-major order, starting with (0,0). A list of  models is stored for every cell. \n"
"MotionModelList[] data\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: nav_msgs/MapMetaData\n"
"# This hold basic information about the characterists of the OccupancyGrid\n"
"\n"
"# The time at which the map was loaded\n"
"time map_load_time\n"
"# The map resolution [m/cell]\n"
"float32 resolution\n"
"# Map width [cells]\n"
"uint32 width\n"
"# Map height [cells]\n"
"uint32 height\n"
"# The origin of the map [m, m, rad].  This is the real-world pose of the\n"
"# cell (0,0) in the map.\n"
"geometry_msgs/Pose origin\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: pal_motion_model_msgs/MotionModelList\n"
"#list of motion models learnt in a specific place\n"
"\n"
"MotionModel[] models\n"
"\n"
"\n"
"\n"
"\n"
"================================================================================\n"
"MSG: pal_motion_model_msgs/MotionModel\n"
"## Contains the 2D motion model of a robot at a specific location\n"
"\n"
"#Heading direction is represented through a gaussian pdf.\n"
"float32  heading_mean\n"
"float32  heading_std_dev\n"
"\n"
"#For statistics we store a pdf over the robot speeds\n"
"float32  linear_speed_mean\n"
"float32  linear_speed_std_dev\n"
"float32  angular_speed_mean\n"
"float32  angular_speed_std_dev\n"
"\n"
;
  }

  static const char* value(const ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.info);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionModelMap_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_motion_model_msgs::MotionModelMap_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "info: ";
    s << std::endl;
    Printer< ::nav_msgs::MapMetaData_<ContainerAllocator> >::stream(s, indent + "  ", v.info);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pal_motion_model_msgs::MotionModelList_<ContainerAllocator> >::stream(s, indent + "    ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_MOTION_MODEL_MSGS_MESSAGE_MOTIONMODELMAP_H
