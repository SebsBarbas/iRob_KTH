// Generated by gencpp from file tf_lookup/TfLookupGoal.msg
// DO NOT EDIT!


#ifndef TF_LOOKUP_MESSAGE_TFLOOKUPGOAL_H
#define TF_LOOKUP_MESSAGE_TFLOOKUPGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tf_lookup
{
template <class ContainerAllocator>
struct TfLookupGoal_
{
  typedef TfLookupGoal_<ContainerAllocator> Type;

  TfLookupGoal_()
    : target_frame()
    , source_frame()
    , transform_time()  {
    }
  TfLookupGoal_(const ContainerAllocator& _alloc)
    : target_frame(_alloc)
    , source_frame(_alloc)
    , transform_time()  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _target_frame_type;
  _target_frame_type target_frame;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _source_frame_type;
  _source_frame_type source_frame;

   typedef ros::Time _transform_time_type;
  _transform_time_type transform_time;





  typedef boost::shared_ptr< ::tf_lookup::TfLookupGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tf_lookup::TfLookupGoal_<ContainerAllocator> const> ConstPtr;

}; // struct TfLookupGoal_

typedef ::tf_lookup::TfLookupGoal_<std::allocator<void> > TfLookupGoal;

typedef boost::shared_ptr< ::tf_lookup::TfLookupGoal > TfLookupGoalPtr;
typedef boost::shared_ptr< ::tf_lookup::TfLookupGoal const> TfLookupGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tf_lookup::TfLookupGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tf_lookup

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'tf_lookup': ['/home/s/s/ssbl/catkin_ws/devel/share/tf_lookup/msg', '/home/s/s/ssbl/catkin_ws/src/robi_final_project/tf_lookup/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfLookupGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfLookupGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfLookupGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bb9d983758e61f286b43546ac9c0b080";
  }

  static const char* value(const ::tf_lookup::TfLookupGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbb9d983758e61f28ULL;
  static const uint64_t static_value2 = 0x6b43546ac9c0b080ULL;
};

template<class ContainerAllocator>
struct DataType< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tf_lookup/TfLookupGoal";
  }

  static const char* value(const ::tf_lookup::TfLookupGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the goal\n"
"string target_frame #The frame to which data should be transformed\n"
"string source_frame #The frame where the data originated\n"
"time transform_time\n"
;
  }

  static const char* value(const ::tf_lookup::TfLookupGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_frame);
      stream.next(m.source_frame);
      stream.next(m.transform_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TfLookupGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tf_lookup::TfLookupGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tf_lookup::TfLookupGoal_<ContainerAllocator>& v)
  {
    s << indent << "target_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.target_frame);
    s << indent << "source_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.source_frame);
    s << indent << "transform_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.transform_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TF_LOOKUP_MESSAGE_TFLOOKUPGOAL_H
