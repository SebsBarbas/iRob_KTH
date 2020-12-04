// Generated by gencpp from file tf_lookup/TfStreamResult.msg
// DO NOT EDIT!


#ifndef TF_LOOKUP_MESSAGE_TFSTREAMRESULT_H
#define TF_LOOKUP_MESSAGE_TFSTREAMRESULT_H


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
struct TfStreamResult_
{
  typedef TfStreamResult_<ContainerAllocator> Type;

  TfStreamResult_()
    : subscription_id()
    , topic()  {
    }
  TfStreamResult_(const ContainerAllocator& _alloc)
    : subscription_id(_alloc)
    , topic(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _subscription_id_type;
  _subscription_id_type subscription_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_type;
  _topic_type topic;





  typedef boost::shared_ptr< ::tf_lookup::TfStreamResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tf_lookup::TfStreamResult_<ContainerAllocator> const> ConstPtr;

}; // struct TfStreamResult_

typedef ::tf_lookup::TfStreamResult_<std::allocator<void> > TfStreamResult;

typedef boost::shared_ptr< ::tf_lookup::TfStreamResult > TfStreamResultPtr;
typedef boost::shared_ptr< ::tf_lookup::TfStreamResult const> TfStreamResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tf_lookup::TfStreamResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tf_lookup::TfStreamResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tf_lookup::TfStreamResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf_lookup::TfStreamResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf_lookup::TfStreamResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8f3e325856c12da00435a78bf464739";
  }

  static const char* value(const ::tf_lookup::TfStreamResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8f3e325856c12daULL;
  static const uint64_t static_value2 = 0x00435a78bf464739ULL;
};

template<class ContainerAllocator>
struct DataType< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tf_lookup/TfStreamResult";
  }

  static const char* value(const ::tf_lookup::TfStreamResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the result\n"
"string subscription_id\n"
"string topic\n"
;
  }

  static const char* value(const ::tf_lookup::TfStreamResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.subscription_id);
      stream.next(m.topic);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TfStreamResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tf_lookup::TfStreamResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tf_lookup::TfStreamResult_<ContainerAllocator>& v)
  {
    s << indent << "subscription_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.subscription_id);
    s << indent << "topic: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TF_LOOKUP_MESSAGE_TFSTREAMRESULT_H
