cmake_minimum_required(VERSION 2.8.3)
project(map_img_proccess)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  message_filters
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map_img_load image_match_fusion key_data_collect
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


#lib map_img_load
add_library(map_img_load src/map_img_load.cpp)
target_link_libraries(map_img_load ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#lib image_match_fusion
add_library(image_match_fusion src/image_match_fusion.cpp)
target_link_libraries(image_match_fusion map_img_load ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#lib key_data_collect
add_library(key_data_collect src/key_data_collect.cpp)
target_link_libraries(key_data_collect image_match_fusion ${catkin_LIBRARIES} )
#exe main
add_executable(main src/main.cpp)
target_link_libraries(main key_data_collect ${catkin_LIBRARIES} )