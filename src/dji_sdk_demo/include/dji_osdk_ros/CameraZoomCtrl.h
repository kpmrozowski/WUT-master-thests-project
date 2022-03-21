// Generated by gencpp from file dji_osdk_ros/CameraZoomCtrl.msg
// DO NOT EDIT!


#ifndef DJI_OSDK_ROS_MESSAGE_CAMERAZOOMCTRL_H
#define DJI_OSDK_ROS_MESSAGE_CAMERAZOOMCTRL_H

#include <ros/service_traits.h>


#include <dji_osdk_ros/CameraZoomCtrlRequest.h>
#include <dji_osdk_ros/CameraZoomCtrlResponse.h>


namespace dji_osdk_ros
{

struct CameraZoomCtrl
{

typedef CameraZoomCtrlRequest Request;
typedef CameraZoomCtrlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CameraZoomCtrl
} // namespace dji_osdk_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dji_osdk_ros::CameraZoomCtrl > {
  static const char* value()
  {
    return "15c6fe8ef5cfbafd183e87752b1029b1";
  }

  static const char* value(const ::dji_osdk_ros::CameraZoomCtrl&) { return value(); }
};

template<>
struct DataType< ::dji_osdk_ros::CameraZoomCtrl > {
  static const char* value()
  {
    return "dji_osdk_ros/CameraZoomCtrl";
  }

  static const char* value(const ::dji_osdk_ros::CameraZoomCtrl&) { return value(); }
};


// service_traits::MD5Sum< ::dji_osdk_ros::CameraZoomCtrlRequest> should match
// service_traits::MD5Sum< ::dji_osdk_ros::CameraZoomCtrl >
template<>
struct MD5Sum< ::dji_osdk_ros::CameraZoomCtrlRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dji_osdk_ros::CameraZoomCtrl >::value();
  }
  static const char* value(const ::dji_osdk_ros::CameraZoomCtrlRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_osdk_ros::CameraZoomCtrlRequest> should match
// service_traits::DataType< ::dji_osdk_ros::CameraZoomCtrl >
template<>
struct DataType< ::dji_osdk_ros::CameraZoomCtrlRequest>
{
  static const char* value()
  {
    return DataType< ::dji_osdk_ros::CameraZoomCtrl >::value();
  }
  static const char* value(const ::dji_osdk_ros::CameraZoomCtrlRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dji_osdk_ros::CameraZoomCtrlResponse> should match
// service_traits::MD5Sum< ::dji_osdk_ros::CameraZoomCtrl >
template<>
struct MD5Sum< ::dji_osdk_ros::CameraZoomCtrlResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dji_osdk_ros::CameraZoomCtrl >::value();
  }
  static const char* value(const ::dji_osdk_ros::CameraZoomCtrlResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dji_osdk_ros::CameraZoomCtrlResponse> should match
// service_traits::DataType< ::dji_osdk_ros::CameraZoomCtrl >
template<>
struct DataType< ::dji_osdk_ros::CameraZoomCtrlResponse>
{
  static const char* value()
  {
    return DataType< ::dji_osdk_ros::CameraZoomCtrl >::value();
  }
  static const char* value(const ::dji_osdk_ros::CameraZoomCtrlResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DJI_OSDK_ROS_MESSAGE_CAMERAZOOMCTRL_H