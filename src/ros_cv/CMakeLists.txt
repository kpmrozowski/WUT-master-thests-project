cmake_minimum_required(VERSION 2.8)
project(ros_cv)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package (catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   cv_bridge
   image_transport
   tf
   sensor_msgs
   message_generation
)

find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

catkin_package (CATKIN_DEPENDS
   roscpp
   std_msgs
   cv_bridge
   opencv2
   image_transport
   sensor_msgs
   LIBRARIES {PROJECT_NAME}
)

include_directories(
   ${PROJECT_SOURCE_DIR}/include
   ${catkin_INCLUDE_DIRS}
)

# Build ROS nodes
set(LIBS
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

# mono node
add_executable (${PROJECT_NAME}_to_grayscale
   src/Grayscaler.cpp
)
target_link_libraries(${PROJECT_NAME}_to_grayscale
   ${LIBS}
)
