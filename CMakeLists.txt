cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorials_ros)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(filtering src/filtering.cpp)

target_link_libraries(filtering
  ${catkin_LIBRARIES}
)
