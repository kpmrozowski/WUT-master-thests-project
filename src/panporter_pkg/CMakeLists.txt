cmake_minimum_required(VERSION 2.8.3)
project(panporter_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /usr/lib/x86_64-linux-gnu/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  robot_state_publisher
  control_toolbox
  geometry_msgs
  joy
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)



## Declare a C++ executable
add_executable(teleop_joy src/teleop_joy.cpp)
add_executable(odom src/odom.cpp)
add_executable(my_motion src/my_motion.cpp src/pid.cpp)
add_library(ContactPlugin SHARED src/ContactPlugin.cc)
# add_executable(panporter_control src/panporter_control.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(teleop_joy
  ${catkin_LIBRARIES} )
target_link_libraries(odom
  ${catkin_LIBRARIES} )
target_link_libraries(my_motion
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
# target_link_libraries(panporter_control
#   ${catkin_LIBRARIES} )
  
