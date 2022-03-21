// Generated by gencpp from file dji_osdk_ros/SetJoystickMode.msg
// DO NOT EDIT!


#ifndef DJI_OSDK_ROS_MESSAGE_SETJOYSTICKMODE_H
#define DJI_OSDK_ROS_MESSAGE_SETJOYSTICKMODE_H

#include <ros/service_traits.h>


#include <dji_osdk_ros/SetJoystickModeRequest.h>
#include <dji_osdk_ros/SetJoystickModeResponse.h>


namespace dji_osdk_ros
{

struct SetJoystickMode
{

typedef SetJoystickModeRequest Request;
typedef SetJoystickModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetJoystickMode
} // namespace dji_osdk_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dji_osdk_ros::SetJoystickMode > {
  static const char* value()
  {
    return "c55e9c9164f1a634e9bf9721aa4cb437";
  }

  static const char* value(const ::dji_osdk_ros::SetJoystickMode&) { return value(); }
};

template<>
struct DataType< ::dji_osdk_ros::SetJoystickMode > {
  static const char* value()
  {
    return "dji_osdk_ros/SetJoystickMode";
  }

  static const char* value(const ::dji_osdk_ros::SetJoystickMode&) { return value(); }
};


// service_traits::MD5Sum< ::dji_osdk_ros::SetJoystickModeRequest> should match
// service_traits::MD5Sum< ::dji_osdk_ros::SetJoystickMode >
template<>
struct MD5Sum< ::dji_osdk_ros::SetJoystickModeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dji_osdk_ros::SetJoystickMode >::value();
  }
  static const char* value(const ::dji_osdk_ros::SetJoystickModeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_osdk_ros::SetJoystickModeRequest> should match
// service_traits::DataType< ::dji_osdk_ros::SetJoystickMode >
template<>
struct DataType< ::dji_osdk_ros::SetJoystickModeRequest>
{
  static const char* value()
  {
    return DataType< ::dji_osdk_ros::SetJoystickMode >::value();
  }
  static const char* value(const ::dji_osdk_ros::SetJoystickModeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dji_osdk_ros::SetJoystickModeResponse> should match
// service_traits::MD5Sum< ::dji_osdk_ros::SetJoystickMode >
template<>
struct MD5Sum< ::dji_osdk_ros::SetJoystickModeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dji_osdk_ros::SetJoystickMode >::value();
  }
  static const char* value(const ::dji_osdk_ros::SetJoystickModeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_osdk_ros::SetJoystickModeResponse> should match
// service_traits::DataType< ::dji_osdk_ros::SetJoystickMode >
template<>
struct DataType< ::dji_osdk_ros::SetJoystickModeResponse>
{
  static const char* value()
  {
    return DataType< ::dji_osdk_ros::SetJoystickMode >::value();
  }
  static const char* value(const ::dji_osdk_ros::SetJoystickModeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DJI_OSDK_ROS_MESSAGE_SETJOYSTICKMODE_H