// Generated by gencpp from file humanoid_nav_msgs/StepTargetServiceResponse.msg
// DO NOT EDIT!


#ifndef HUMANOID_NAV_MSGS_MESSAGE_STEPTARGETSERVICERESPONSE_H
#define HUMANOID_NAV_MSGS_MESSAGE_STEPTARGETSERVICERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace humanoid_nav_msgs
{
template <class ContainerAllocator>
struct StepTargetServiceResponse_
{
  typedef StepTargetServiceResponse_<ContainerAllocator> Type;

  StepTargetServiceResponse_()
    {
    }
  StepTargetServiceResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct StepTargetServiceResponse_

typedef ::humanoid_nav_msgs::StepTargetServiceResponse_<std::allocator<void> > StepTargetServiceResponse;

typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTargetServiceResponse > StepTargetServiceResponsePtr;
typedef boost::shared_ptr< ::humanoid_nav_msgs::StepTargetServiceResponse const> StepTargetServiceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace humanoid_nav_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'humanoid_nav_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/humanoid_msgs/humanoid_nav_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/humanoid_nav_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "humanoid_nav_msgs/StepTargetServiceResponse";
  }

  static const char* value(const ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StepTargetServiceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::humanoid_nav_msgs::StepTargetServiceResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // HUMANOID_NAV_MSGS_MESSAGE_STEPTARGETSERVICERESPONSE_H
