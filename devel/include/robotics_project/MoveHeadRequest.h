// Generated by gencpp from file robotics_project/MoveHeadRequest.msg
// DO NOT EDIT!


#ifndef ROBOTICS_PROJECT_MESSAGE_MOVEHEADREQUEST_H
#define ROBOTICS_PROJECT_MESSAGE_MOVEHEADREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotics_project
{
template <class ContainerAllocator>
struct MoveHeadRequest_
{
  typedef MoveHeadRequest_<ContainerAllocator> Type;

  MoveHeadRequest_()
    : motion()  {
    }
  MoveHeadRequest_(const ContainerAllocator& _alloc)
    : motion(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _motion_type;
  _motion_type motion;





  typedef boost::shared_ptr< ::robotics_project::MoveHeadRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotics_project::MoveHeadRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MoveHeadRequest_

typedef ::robotics_project::MoveHeadRequest_<std::allocator<void> > MoveHeadRequest;

typedef boost::shared_ptr< ::robotics_project::MoveHeadRequest > MoveHeadRequestPtr;
typedef boost::shared_ptr< ::robotics_project::MoveHeadRequest const> MoveHeadRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotics_project::MoveHeadRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robotics_project

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'robotics_project': ['/home/s/s/ssbl/catkin_ws/devel/share/robotics_project/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotics_project::MoveHeadRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotics_project::MoveHeadRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotics_project::MoveHeadRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "56c5b2babfc4b27e5a596091bb964bc2";
  }

  static const char* value(const ::robotics_project::MoveHeadRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x56c5b2babfc4b27eULL;
  static const uint64_t static_value2 = 0x5a596091bb964bc2ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotics_project/MoveHeadRequest";
  }

  static const char* value(const ::robotics_project::MoveHeadRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string motion\n"
;
  }

  static const char* value(const ::robotics_project::MoveHeadRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.motion);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveHeadRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotics_project::MoveHeadRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotics_project::MoveHeadRequest_<ContainerAllocator>& v)
  {
    s << indent << "motion: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.motion);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTICS_PROJECT_MESSAGE_MOVEHEADREQUEST_H
