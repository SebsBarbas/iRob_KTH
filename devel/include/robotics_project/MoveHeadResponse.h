// Generated by gencpp from file robotics_project/MoveHeadResponse.msg
// DO NOT EDIT!


#ifndef ROBOTICS_PROJECT_MESSAGE_MOVEHEADRESPONSE_H
#define ROBOTICS_PROJECT_MESSAGE_MOVEHEADRESPONSE_H


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
struct MoveHeadResponse_
{
  typedef MoveHeadResponse_<ContainerAllocator> Type;

  MoveHeadResponse_()
    : success(false)  {
    }
  MoveHeadResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::robotics_project::MoveHeadResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotics_project::MoveHeadResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MoveHeadResponse_

typedef ::robotics_project::MoveHeadResponse_<std::allocator<void> > MoveHeadResponse;

typedef boost::shared_ptr< ::robotics_project::MoveHeadResponse > MoveHeadResponsePtr;
typedef boost::shared_ptr< ::robotics_project::MoveHeadResponse const> MoveHeadResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotics_project::MoveHeadResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robotics_project

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'robotics_project': ['/home/s/s/ssbl/catkin_ws/devel/share/robotics_project/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotics_project::MoveHeadResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotics_project::MoveHeadResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotics_project::MoveHeadResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::robotics_project::MoveHeadResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotics_project/MoveHeadResponse";
  }

  static const char* value(const ::robotics_project::MoveHeadResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
;
  }

  static const char* value(const ::robotics_project::MoveHeadResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveHeadResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotics_project::MoveHeadResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotics_project::MoveHeadResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTICS_PROJECT_MESSAGE_MOVEHEADRESPONSE_H
