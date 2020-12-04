// Generated by gencpp from file pal_interaction_msgs/AudioPlayerState.msg
// DO NOT EDIT!


#ifndef PAL_INTERACTION_MSGS_MESSAGE_AUDIOPLAYERSTATE_H
#define PAL_INTERACTION_MSGS_MESSAGE_AUDIOPLAYERSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace pal_interaction_msgs
{
template <class ContainerAllocator>
struct AudioPlayerState_
{
  typedef AudioPlayerState_<ContainerAllocator> Type;

  AudioPlayerState_()
    : header()
    , isPlaying(false)
    , msg()  {
    }
  AudioPlayerState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , isPlaying(false)
    , msg(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _isPlaying_type;
  _isPlaying_type isPlaying;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;





  typedef boost::shared_ptr< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> const> ConstPtr;

}; // struct AudioPlayerState_

typedef ::pal_interaction_msgs::AudioPlayerState_<std::allocator<void> > AudioPlayerState;

typedef boost::shared_ptr< ::pal_interaction_msgs::AudioPlayerState > AudioPlayerStatePtr;
typedef boost::shared_ptr< ::pal_interaction_msgs::AudioPlayerState const> AudioPlayerStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_interaction_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'pal_interaction_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_interaction_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_interaction_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "56d08544761f7625c1fa9ada542abb32";
  }

  static const char* value(const ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x56d08544761f7625ULL;
  static const uint64_t static_value2 = 0xc1fa9ada542abb32ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_interaction_msgs/AudioPlayerState";
  }

  static const char* value(const ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Message used to publish when the internal player is playing something on the speakers\n"
"# isPlayer is true when playing.\n"
"Header header\n"
"bool isPlaying\n"
"string msg\n"
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
;
  }

  static const char* value(const ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.isPlaying);
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AudioPlayerState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_interaction_msgs::AudioPlayerState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "isPlaying: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isPlaying);
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_INTERACTION_MSGS_MESSAGE_AUDIOPLAYERSTATE_H
