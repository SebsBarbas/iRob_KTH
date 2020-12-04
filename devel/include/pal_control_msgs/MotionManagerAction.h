// Generated by gencpp from file pal_control_msgs/MotionManagerAction.msg
// DO NOT EDIT!


#ifndef PAL_CONTROL_MSGS_MESSAGE_MOTIONMANAGERACTION_H
#define PAL_CONTROL_MSGS_MESSAGE_MOTIONMANAGERACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pal_control_msgs/MotionManagerActionGoal.h>
#include <pal_control_msgs/MotionManagerActionResult.h>
#include <pal_control_msgs/MotionManagerActionFeedback.h>

namespace pal_control_msgs
{
template <class ContainerAllocator>
struct MotionManagerAction_
{
  typedef MotionManagerAction_<ContainerAllocator> Type;

  MotionManagerAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  MotionManagerAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::pal_control_msgs::MotionManagerActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::pal_control_msgs::MotionManagerActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::pal_control_msgs::MotionManagerActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> const> ConstPtr;

}; // struct MotionManagerAction_

typedef ::pal_control_msgs::MotionManagerAction_<std::allocator<void> > MotionManagerAction;

typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerAction > MotionManagerActionPtr;
typedef boost::shared_ptr< ::pal_control_msgs::MotionManagerAction const> MotionManagerActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_control_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'pal_control_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_control_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_control_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "42689d3bf9c1135e4da2202787f92626";
  }

  static const char* value(const ::pal_control_msgs::MotionManagerAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x42689d3bf9c1135eULL;
  static const uint64_t static_value2 = 0x4da2202787f92626ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_control_msgs/MotionManagerAction";
  }

  static const char* value(const ::pal_control_msgs::MotionManagerAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"MotionManagerActionGoal action_goal\n"
"MotionManagerActionResult action_result\n"
"MotionManagerActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: pal_control_msgs/MotionManagerActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"MotionManagerGoal goal\n"
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
"MSG: pal_control_msgs/MotionManagerGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Path to XML file containing motions for the robot\n"
"string filename\n"
"\n"
"# True if a collision-free approach motion and trajectory validation are to be performed.\n"
"# If set to true but an approach motion is not required, it will not be computed.\n"
"bool plan\n"
"\n"
"#True if safety around the robot must be checked using sensors such as the sonars and lasers\n"
"bool checkSafety\n"
"\n"
"#True if the motion must be repeated until a new goal has been received\n"
"bool repeat\n"
"\n"
"#priority of the motion, 0 is no priority, 100 is max priority\n"
"uint8 priority\n"
"\n"
"#Specifies how long in miliseconds should the goal wait before forcing an execution. If a movement is being executed when the goal is received, it will wait the specified time or until the movement finishes to execute it.\n"
"# -1 Means wait forever until the previous movement has finished\n"
"int32 queueTimeout \n"
"\n"
"\n"
"================================================================================\n"
"MSG: pal_control_msgs/MotionManagerActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"MotionManagerResult result\n"
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
"MSG: pal_control_msgs/MotionManagerResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#Message in result, can contain information if goal failed\n"
"string message\n"
"\n"
"================================================================================\n"
"MSG: pal_control_msgs/MotionManagerActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"MotionManagerFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: pal_control_msgs/MotionManagerFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# feedback message\n"
"# no feedback for the moment. could be progress, or final position\n"
"\n"
;
  }

  static const char* value(const ::pal_control_msgs::MotionManagerAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionManagerAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_control_msgs::MotionManagerAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_control_msgs::MotionManagerAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::pal_control_msgs::MotionManagerActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::pal_control_msgs::MotionManagerActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::pal_control_msgs::MotionManagerActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_CONTROL_MSGS_MESSAGE_MOTIONMANAGERACTION_H
