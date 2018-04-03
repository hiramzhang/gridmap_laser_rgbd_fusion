cmake_minimum_required(VERSION 2.8.3)
project(laser_rgbd_sonar_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scan_lrs
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#lib scan_lrs
add_library(scan_lrs src/scan_lrs.cpp)
target_link_libraries(scan_lrs ${catkin_LIBRARIES} )

add_executable(main src/main.cpp)
target_link_libraries(main scan_lrs ${catkin_LIBRARIES} )

