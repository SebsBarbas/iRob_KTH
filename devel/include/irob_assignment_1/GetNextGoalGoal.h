// Generated by gencpp from file irob_assignment_1/GetNextGoalGoal.msg
// DO NOT EDIT!


#ifndef IROB_ASSIGNMENT_1_MESSAGE_GETNEXTGOALGOAL_H
#define IROB_ASSIGNMENT_1_MESSAGE_GETNEXTGOALGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace irob_assignment_1
{
template <class ContainerAllocator>
struct GetNextGoalGoal_
{
  typedef GetNextGoalGoal_<ContainerAllocator> Type;

  GetNextGoalGoal_()
    {
    }
  GetNextGoalGoal_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> const> ConstPtr;

}; // struct GetNextGoalGoal_

typedef ::irob_assignment_1::GetNextGoalGoal_<std::allocator<void> > GetNextGoalGoal;

typedef boost::shared_ptr< ::irob_assignment_1::GetNextGoalGoal > GetNextGoalGoalPtr;
typedef boost::shared_ptr< ::irob_assignment_1::GetNextGoalGoal const> GetNextGoalGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace irob_assignment_1

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/melodic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'irob_assignment_1': ['/home/s/s/ssbl/catkin_ws/devel/share/irob_assignment_1/msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "irob_assignment_1/GetNextGoalGoal";
  }

  static const char* value(const ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Goal definition\n"
;
  }

  static const char* value(const ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetNextGoalGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::irob_assignment_1::GetNextGoalGoal_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // IROB_ASSIGNMENT_1_MESSAGE_GETNEXTGOALGOAL_H