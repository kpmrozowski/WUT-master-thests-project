// Generated by gencpp from file dji_osdk_ros/DownloadWaypointV2MissionResponse.msg
// DO NOT EDIT!


#ifndef DJI_OSDK_ROS_MESSAGE_DOWNLOADWAYPOINTV2MISSIONRESPONSE_H
#define DJI_OSDK_ROS_MESSAGE_DOWNLOADWAYPOINTV2MISSIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dji_osdk_ros/WaypointV2.h>

namespace dji_osdk_ros
{
template <class ContainerAllocator>
struct DownloadWaypointV2MissionResponse_
{
  typedef DownloadWaypointV2MissionResponse_<ContainerAllocator> Type;

  DownloadWaypointV2MissionResponse_()
    : mission()
    , result(false)  {
    }
  DownloadWaypointV2MissionResponse_(const ContainerAllocator& _alloc)
    : mission(_alloc)
    , result(false)  {
  (void)_alloc;
    }



   typedef std::vector< ::dji_osdk_ros::WaypointV2_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::dji_osdk_ros::WaypointV2_<ContainerAllocator> >::other >  _mission_type;
  _mission_type mission;

   typedef uint8_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DownloadWaypointV2MissionResponse_

typedef ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<std::allocator<void> > DownloadWaypointV2MissionResponse;

typedef boost::shared_ptr< ::dji_osdk_ros::DownloadWaypointV2MissionResponse > DownloadWaypointV2MissionResponsePtr;
typedef boost::shared_ptr< ::dji_osdk_ros::DownloadWaypointV2MissionResponse const> DownloadWaypointV2MissionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator1> & lhs, const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.mission == rhs.mission &&
    lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator1> & lhs, const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dji_osdk_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b9a6e8c58bda1e8d747c69006bbc940";
  }

  static const char* value(const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b9a6e8c58bda1e8ULL;
  static const uint64_t static_value2 = 0xd747c69006bbc940ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_osdk_ros/DownloadWaypointV2MissionResponse";
  }

  static const char* value(const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#response\n"
"WaypointV2[] mission\n"
"bool result\n"
"\n"
"================================================================================\n"
"MSG: dji_osdk_ros/WaypointV2\n"
"# The struct represents a target point in the waypoint mission. For a waypoint\n"
"\n"
"#constant for flightpathMode\n"
"uint8 DJIWaypointV2FlightPathModeGoToPointAlongACurve = 0  #In the mission, the aircraft will go to the waypoint along a curve and fly past the waypoint.\n"
"uint8 DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop = 1 #In the mission, the aircraft will go to the waypoint along a curve and stop at the waypoint.\n"
"uint8 DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop = 2  #In the mission, the aircraft will go to the waypoint along a straight line and stop at the waypoint.\n"
"uint8 DJIWaypointV2FlightPathModeCoordinateTurn = 3   #In the mission, the aircraft will fly from the previous waypoint to the next waypoint along a smooth curve without stopping at this waypoint.\n"
"                                                      #the next in a curved motion,  adhering to the ``DJIWaypointV2_dampingDistance``, which is\n"
"                                                      #set in ``DJIWaypointV2``.\n"
"uint8 DJIWaypointV2FlightPathModeGoToFirstPointAlongAStraightLine = 4  # In the mission, the aircraft will go to the first waypoint along a straight line.\n"
"                                                                       # This is only valid for the first waypoint.\n"
"uint8 DJIWaypointV2FlightPathModeStraightOut = 5   # Straight exit the Last waypoint, Only valid for last waypoint.\n"
"uint8 DJIWaypointV2FlightPathModeUnknown = 255    # Unknown\n"
"\n"
"#constant for headMode\n"
"uint8 DJIWaypointV2HeadingModeAuto = 0    # Aircraft's heading will always be in the direction of flight.\n"
"uint8 DJIWaypointV2HeadingFixed    = 1    # Aircraft's heading will be set to the heading when reaching the first waypoint.\n"
"                                          # Before reaching the first waypoint, the aircraft's heading can be controlled by\n"
"                                          # the remote controller. When the aircraft reaches the first waypoint, its\n"
"                                          # heading will be fixed.\n"
"uint8 DJIWaypointV2HeadingManual   = 2    # The aircraft's heading in the mission can be controlled by the remote controller.\n"
"uint8 DJIWaypointV2HeadingWaypointCustom = 3  # In the mission, the aircraft's heading will change dynamically and adapt to the heading set at the next waypoint.\n"
"                                               # See ``DJIWaypointV2_heading`` to preset the heading.\n"
"uint8 DJIWaypointV2HeadingTowardPointOfInterest = 4 # Aircraft's heading will always toward point of interest.\n"
"                                                    # using ``DJIWaypointV2_pointOfInterest`` setting point of interest coordiate and ``DJIWaypointV2_pointOfInterestAltitude``\n"
"                                                    # setting point of interset altitute.\n"
"uint8 DJIWaypointV2HeadingGimbalYawFollow  = 5   # The aircraft's heading rotate simultaneously with its gimbal's yaw.\n"
"uint8 DJIWaypointV2HeadingUnknown = 255         # Unknown.\n"
"\n"
"#constant for turnMode\n"
"uint8 DJIWaypointV2TurnModeClockwise = 0  # The aircraft's heading rotates clockwise.\n"
"uint8 DJIWaypointV2TurnModeCounterClockwise = 1   # The aircraft's heading rotates counterclockwise.\n"
"uint8 DJIWaypointV2TurnModeUnknown = 255    # Changes the heading of the aircraft by rotating the aircraft anti-clockwise.\n"
"\n"
"\n"
"\n"
"#  mission, a flight route  consists of multiple `WaypointV2` objects.\n"
"float64 longitude  # waypoint position relative to WayPointV2InitSettings's reference point.unit: m\n"
"float64 latitude\n"
"float32 relativeHeight  # relative to takeoff height\n"
"uint8   waypointType    # Waypoint flight path mode\n"
"uint8   headingMode     # Represents current aircraft's heading mode on current waypoint.\n"
"WaypointV2Config  config          # Represents current waypoint's speed config.\n"
"uint16  dampingDistance\n"
"float32 heading         # The heading to which the aircraft will rotate by the time it reaches the\n"
"                        # waypoint. The aircraft heading  will gradually change between two waypoints with\n"
"                        # different headings if the waypoint  mission's `headingMode` is set  to\n"
"                        # 'DJIWaypointV2_DJIWaypointV2HeadingMode_WaypointCustom`. A heading has a range of\n"
"                        # [-180, 180] degrees, where 0 represents True North.\n"
"uint8 turnMode          # Determines whether the aircraft will turn clockwise or anticlockwise when\n"
"                        # changing its heading.\n"
"# Property is used when ``DJIWaypointV2_headingMode`` is\n"
"# ``DJIWaypointV2_DJIWaypointV2HeadingMode_TowardPointOfInterest``.\n"
"# Aircraft will always be heading to point while executing mission. Default is\n"
"# \"kCLLocationCoordinate2DInvalid\".\n"
"float32 positionX       # X distance to reference point, North is positive\n"
"float32 positionY       # Y distance to reference point, East is positive\n"
"float32 positionZ       # Z distance to reference point, UP is positive\n"
"\n"
"# While the aircraft is travelling between waypoints, you can offset its speed by\n"
"# using the throttle joystick on the remote controller. \"maxFlightSpeed\" is this\n"
"# offset when the joystick is pushed to maximum deflection. For example, If\n"
"# maxFlightSpeed is 10 m/s, then pushing the throttle joystick all the way up will\n"
"# add 10 m/s to the aircraft speed, while pushing down will subtract 10 m/s from\n"
"# the aircraft speed. If the remote controller stick is not at maximum deflection,\n"
"# then the offset speed will be interpolated between \"[0, maxFlightSpeed]\"\" with a\n"
"# resolution of 1000 steps. If the offset speed is negative, then the aircraft\n"
"# will fly backwards to previous waypoints. When it reaches the first waypoint, it\n"
"# will then hover in place until a positive speed is applied. \"maxFlightSpeed\" has\n"
"# a range of [2,15] m/s.\n"
"float32 maxFlightSpeed\n"
"\n"
"# The base automatic speed of the aircraft as it moves between waypoints with\n"
"# range [-15, 15] m/s. The aircraft's actual speed is a combination of the base\n"
"# automatic speed, and the speed control given by the throttle joystick on the\n"
"# remote controller. If \"autoFlightSpeed >0\": Actual speed is \"autoFlightSpeed\" +\n"
"# Joystick Speed (with combined max of \"maxFlightSpeed\") If \"autoFlightSpeed =0\":\n"
"# Actual speed is controlled only by the remote controller joystick. If\n"
"# autoFlightSpeed <0\" and the aircraft is at the first waypoint, the aircraft\n"
"# will hover in place until the speed is made positive by the remote controller\n"
"# joystick. In flight controller firmware 3.2.10.0 or above, different speeds\n"
"# between individual waypoints can also be set in waypoint objects which will\n"
"# overwrite \"autoFlightSpeed\".\n"
"float32 autoFlightSpeed\n"
"================================================================================\n"
"MSG: dji_osdk_ros/WaypointV2Config\n"
"# Represents current waypoint's speed config.\n"
"# 0: set local waypoint's cruise speed,\n"
"# 1: unset global waypoint's cruise speed*/\n"
"uint8  useLocalCruiseVel\n"
"# 0: set local waypoint's max speed,\n"
"# 1: unset global waypoint's max speed*/\n"
"uint8  useLocalMaxVel\n"
;
  }

  static const char* value(const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mission);
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DownloadWaypointV2MissionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_osdk_ros::DownloadWaypointV2MissionResponse_<ContainerAllocator>& v)
  {
    s << indent << "mission[]" << std::endl;
    for (size_t i = 0; i < v.mission.size(); ++i)
    {
      s << indent << "  mission[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dji_osdk_ros::WaypointV2_<ContainerAllocator> >::stream(s, indent + "    ", v.mission[i]);
    }
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_OSDK_ROS_MESSAGE_DOWNLOADWAYPOINTV2MISSIONRESPONSE_H