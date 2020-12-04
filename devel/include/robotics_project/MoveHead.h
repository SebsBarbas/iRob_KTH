// Generated by gencpp from file robotics_project/MoveHead.msg
// DO NOT EDIT!


#ifndef ROBOTICS_PROJECT_MESSAGE_MOVEHEAD_H
#define ROBOTICS_PROJECT_MESSAGE_MOVEHEAD_H

#include <ros/service_traits.h>


#include <robotics_project/MoveHeadRequest.h>
#include <robotics_project/MoveHeadResponse.h>


namespace robotics_project
{

struct MoveHead
{

typedef MoveHeadRequest Request;
typedef MoveHeadResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MoveHead
} // namespace robotics_project


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::robotics_project::MoveHead > {
  static const char* value()
  {
    return "3ceff7a00b3f71368b0b0f59df1b9c42";
  }

  static const char* value(const ::robotics_project::MoveHead&) { return value(); }
};

template<>
struct DataType< ::robotics_project::MoveHead > {
  static const char* value()
  {
    return "robotics_project/MoveHead";
  }

  static const char* value(const ::robotics_project::MoveHead&) { return value(); }
};


// service_traits::MD5Sum< ::robotics_project::MoveHeadRequest> should match 
// service_traits::MD5Sum< ::robotics_project::MoveHead > 
template<>
struct MD5Sum< ::robotics_project::MoveHeadRequest>
{
  static const char* value()
  {
    return MD5Sum< ::robotics_project::MoveHead >::value();
  }
  static const char* value(const ::robotics_project::MoveHeadRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::robotics_project::MoveHeadRequest> should match 
// service_traits::DataType< ::robotics_project::MoveHead > 
template<>
struct DataType< ::robotics_project::MoveHeadRequest>
{
  static const char* value()
  {
    return DataType< ::robotics_project::MoveHead >::value();
  }
  static const char* value(const ::robotics_project::MoveHeadRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::robotics_project::MoveHeadResponse> should match 
// service_traits::MD5Sum< ::robotics_project::MoveHead > 
template<>
struct MD5Sum< ::robotics_project::MoveHeadResponse>
{
  static const char* value()
  {
    return MD5Sum< ::robotics_project::MoveHead >::value();
  }
  static const char* value(const ::robotics_project::MoveHeadResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::robotics_project::MoveHeadResponse> should match 
// service_traits::DataType< ::robotics_project::MoveHead > 
template<>
struct DataType< ::robotics_project::MoveHeadResponse>
{
  static const char* value()
  {
    return DataType< ::robotics_project::MoveHead >::value();
  }
  static const char* value(const ::robotics_project::MoveHeadResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROBOTICS_PROJECT_MESSAGE_MOVEHEAD_H