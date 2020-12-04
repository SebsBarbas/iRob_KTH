// Generated by gencpp from file pal_control_msgs/RigidBodyTrajectoryFeedback.msg
// DO NOT EDIT!


#ifndef PAL_CONTROL_MSGS_MESSAGE_RIGIDBODYTRAJECTORYFEEDBACK_H
#define PAL_CONTROL_MSGS_MESSAGE_RIGIDBODYTRAJECTORYFEEDBACK_H


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
struct RigidBodyTrajectoryFeedback_
{
  typedef RigidBodyTrajectoryFeedback_<ContainerAllocator> Type;

  RigidBodyTrajectoryFeedback_()
    {
    }
  RigidBodyTrajectoryFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct RigidBodyTrajectoryFeedback_

typedef ::pal_control_msgs::RigidBodyTrajectoryFeedback_<std::allocator<void> > RigidBodyTrajectoryFeedback;

typedef boost::shared_ptr< ::pal_control_msgs::RigidBodyTrajectoryFeedback > RigidBodyTrajectoryFeedbackPtr;
typedef boost::shared_ptr< ::pal_control_msgs::RigidBodyTrajectoryFeedback const> RigidBodyTrajectoryFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_control_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'pal_control_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_control_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_control_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_control_msgs/RigidBodyTrajectoryFeedback";
  }

  static const char* value(const ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
;
  }

  static const char* value(const ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RigidBodyTrajectoryFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::pal_control_msgs::RigidBodyTrajectoryFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // PAL_CONTROL_MSGS_MESSAGE_RIGIDBODYTRAJECTORYFEEDBACK_H
