// Generated by gencpp from file humanoid_nav_msgs/ExecFootstepsActionFeedback.msg
// DO NOT EDIT!


#ifndef HUMANOID_NAV_MSGS_MESSAGE_EXECFOOTSTEPSACTIONFEEDBACK_H
#define HUMANOID_NAV_MSGS_MESSAGE_EXECFOOTSTEPSACTIONFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalStatus.h>
#include <humanoid_nav_msgs/ExecFootstepsFeedback.h>

namespace humanoid_nav_msgs
{
template <class ContainerAllocator>
struct ExecFootstepsActionFeedback_
{
  typedef ExecFootstepsActionFeedback_<ContainerAllocator> Type;

  ExecFootstepsActionFeedback_()
    : header()
    , status()
    , feedback()  {
    }
  ExecFootstepsActionFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)
    , feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalStatus_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef  ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator>  _feedback_type;
  _feedback_type feedback;





  typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct ExecFootstepsActionFeedback_

typedef ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<std::allocator<void> > ExecFootstepsActionFeedback;

typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsActionFeedback > ExecFootstepsActionFeedbackPtr;
typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsActionFeedback const> ExecFootstepsActionFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace humanoid_nav_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'humanoid_nav_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/humanoid_msgs/humanoid_nav_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/humanoid_nav_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "01da67223a0592f0de40186a03f441ff";
  }

  static const char* value(const ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x01da67223a0592f0ULL;
  static const uint64_t static_value2 = 0xde40186a03f441ffULL;
};

template<class ContainerAllocator>
struct DataType< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "humanoid_nav_msgs/ExecFootstepsActionFeedback";
  }

  static const char* value(const ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"ExecFootstepsFeedback feedback\n"
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
"MSG: actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: humanoid_nav_msgs/ExecFootstepsFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define a feedback message\n"
"humanoid_nav_msgs/StepTarget[] executed_footsteps\n"
"\n"
"\n"
"================================================================================\n"
"MSG: humanoid_nav_msgs/StepTarget\n"
"# Target for a single stepping motion of a humanoid's leg\n"
"\n"
"geometry_msgs/Pose2D pose   # step pose as relative offset to last leg\n"
"uint8 leg                   # which leg to use (left/right, see below)\n"
"\n"
"uint8 right=0               # right leg constant\n"
"uint8 left=1                # left leg constant\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
;
  }

  static const char* value(const ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
      stream.next(m.feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ExecFootstepsActionFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::humanoid_nav_msgs::ExecFootstepsActionFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "feedback: ";
    s << std::endl;
    Printer< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HUMANOID_NAV_MSGS_MESSAGE_EXECFOOTSTEPSACTIONFEEDBACK_H
