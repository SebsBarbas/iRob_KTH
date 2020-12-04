// Generated by gencpp from file pal_device_msgs/TimedFadeEffectResponse.msg
// DO NOT EDIT!


#ifndef PAL_DEVICE_MSGS_MESSAGE_TIMEDFADEEFFECTRESPONSE_H
#define PAL_DEVICE_MSGS_MESSAGE_TIMEDFADEEFFECTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_device_msgs
{
template <class ContainerAllocator>
struct TimedFadeEffectResponse_
{
  typedef TimedFadeEffectResponse_<ContainerAllocator> Type;

  TimedFadeEffectResponse_()
    : effectId(0)  {
    }
  TimedFadeEffectResponse_(const ContainerAllocator& _alloc)
    : effectId(0)  {
  (void)_alloc;
    }



   typedef uint32_t _effectId_type;
  _effectId_type effectId;





  typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TimedFadeEffectResponse_

typedef ::pal_device_msgs::TimedFadeEffectResponse_<std::allocator<void> > TimedFadeEffectResponse;

typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectResponse > TimedFadeEffectResponsePtr;
typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectResponse const> TimedFadeEffectResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_device_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'pal_device_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_device_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_device_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f6602e24c29b84632fa113b15b20645d";
  }

  static const char* value(const ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf6602e24c29b8463ULL;
  static const uint64_t static_value2 = 0x2fa113b15b20645dULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_device_msgs/TimedFadeEffectResponse";
  }

  static const char* value(const ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 effectId\n"
"\n"
"\n"
"\n"
"\n"
;
  }

  static const char* value(const ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.effectId);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimedFadeEffectResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_device_msgs::TimedFadeEffectResponse_<ContainerAllocator>& v)
  {
    s << indent << "effectId: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.effectId);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_DEVICE_MSGS_MESSAGE_TIMEDFADEEFFECTRESPONSE_H
