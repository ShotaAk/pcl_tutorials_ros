cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorials_ros)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  roslint
  sensor_msgs
  tf2_ros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(filtering src/filtering.cpp)
add_executable(segmentation src/segmentation.cpp)
add_executable(kdtree src/kdtree.cpp)

target_link_libraries(filtering
  ${catkin_LIBRARIES}
)
target_link_libraries(segmentation
  ${catkin_LIBRARIES}
)
target_link_libraries(kdtree
  ${catkin_LIBRARIES}
)

roslint_cpp()
roslint_add_test()
