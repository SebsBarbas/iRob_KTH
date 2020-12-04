// Generated by gencpp from file irob_assignment_1/GetSetpointResponse.msg
// DO NOT EDIT!


#ifndef IROB_ASSIGNMENT_1_MESSAGE_GETSETPOINTRESPONSE_H
#define IROB_ASSIGNMENT_1_MESSAGE_GETSETPOINTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

namespace irob_assignment_1
{
template <class ContainerAllocator>
struct GetSetpointResponse_
{
  typedef GetSetpointResponse_<ContainerAllocator> Type;

  GetSetpointResponse_()
    : setpoint()
    , new_path()  {
    }
  GetSetpointResponse_(const ContainerAllocator& _alloc)
    : setpoint(_alloc)
    , new_path(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PointStamped_<ContainerAllocator>  _setpoint_type;
  _setpoint_type setpoint;

   typedef  ::nav_msgs::Path_<ContainerAllocator>  _new_path_type;
  _new_path_type new_path;





  typedef boost::shared_ptr< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetSetpointResponse_

typedef ::irob_assignment_1::GetSetpointResponse_<std::allocator<void> > GetSetpointResponse;

typedef boost::shared_ptr< ::irob_assignment_1::GetSetpointResponse > GetSetpointResponsePtr;
typedef boost::shared_ptr< ::irob_assignment_1::GetSetpointResponse const> GetSetpointResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace irob_assignment_1

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/melodic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'irob_assignment_1': ['/home/s/s/ssbl/catkin_ws/devel/share/irob_assignment_1/msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e894795084873db62c71525b1022e0a";
  }

  static const char* value(const ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e894795084873dbULL;
  static const uint64_t static_value2 = 0x62c71525b1022e0aULL;
};

template<class ContainerAllocator>
struct DataType< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "irob_assignment_1/GetSetpointResponse";
  }

  static const char* value(const ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"geometry_msgs/PointStamped setpoint\n"
"nav_msgs/Path new_path\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PointStamped\n"
"# This represents a Point with reference coordinate frame and timestamp\n"
"Header header\n"
"Point point\n"
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
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: nav_msgs/Path\n"
"#An array of poses that represents a Path for a robot to follow\n"
"Header header\n"
"geometry_msgs/PoseStamped[] poses\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.setpoint);
      stream.next(m.new_path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetSetpointResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::irob_assignment_1::GetSetpointResponse_<ContainerAllocator>& v)
  {
    s << indent << "setpoint: ";
    s << std::endl;
    Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.setpoint);
    s << indent << "new_path: ";
    s << std::endl;
    Printer< ::nav_msgs::Path_<ContainerAllocator> >::stream(s, indent + "  ", v.new_path);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IROB_ASSIGNMENT_1_MESSAGE_GETSETPOINTRESPONSE_H