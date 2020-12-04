// Generated by gencpp from file pal_detection_msgs/RecognizeObjectsGoal.msg
// DO NOT EDIT!


#ifndef PAL_DETECTION_MSGS_MESSAGE_RECOGNIZEOBJECTSGOAL_H
#define PAL_DETECTION_MSGS_MESSAGE_RECOGNIZEOBJECTSGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/CompressedImage.h>

namespace pal_detection_msgs
{
template <class ContainerAllocator>
struct RecognizeObjectsGoal_
{
  typedef RecognizeObjectsGoal_<ContainerAllocator> Type;

  RecognizeObjectsGoal_()
    : input_image()  {
    }
  RecognizeObjectsGoal_(const ContainerAllocator& _alloc)
    : input_image(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::CompressedImage_<ContainerAllocator>  _input_image_type;
  _input_image_type input_image;





  typedef boost::shared_ptr< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> const> ConstPtr;

}; // struct RecognizeObjectsGoal_

typedef ::pal_detection_msgs::RecognizeObjectsGoal_<std::allocator<void> > RecognizeObjectsGoal;

typedef boost::shared_ptr< ::pal_detection_msgs::RecognizeObjectsGoal > RecognizeObjectsGoalPtr;
typedef boost::shared_ptr< ::pal_detection_msgs::RecognizeObjectsGoal const> RecognizeObjectsGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_detection_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'pal_detection_msgs': ['/home/s/s/ssbl/catkin_ws/src/robi_final_project/pal_msgs/pal_detection_msgs/msg', '/home/s/s/ssbl/catkin_ws/devel/share/pal_detection_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "59fdbada6b96559166a5cd7d802609a1";
  }

  static const char* value(const ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x59fdbada6b965591ULL;
  static const uint64_t static_value2 = 0x66a5cd7d802609a1ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_detection_msgs/RecognizeObjectsGoal";
  }

  static const char* value(const ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# input_image: An optional image field, when given performs the recognition on the input_image\n"
"# else, will perform the recognition on the recent captured image from robot's camera\n"
"sensor_msgs/CompressedImage input_image\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/CompressedImage\n"
"# This message contains a compressed image\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"\n"
"string format        # Specifies the format of the data\n"
"                     #   Acceptable values:\n"
"                     #     jpeg, png\n"
"uint8[] data         # Compressed image buffer\n"
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
;
  }

  static const char* value(const ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input_image);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RecognizeObjectsGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_detection_msgs::RecognizeObjectsGoal_<ContainerAllocator>& v)
  {
    s << indent << "input_image: ";
    s << std::endl;
    Printer< ::sensor_msgs::CompressedImage_<ContainerAllocator> >::stream(s, indent + "  ", v.input_image);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_DETECTION_MSGS_MESSAGE_RECOGNIZEOBJECTSGOAL_H
