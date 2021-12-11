cmake_minimum_required(VERSION 2.8.3)
project(pseudo_lidar_tools)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf
    rosbag
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tmp nodes/tools/tmp.cpp)
target_link_libraries(tmp ${catkin_LIBRARIES})

add_executable(gt_mapping nodes/tools/gt_mapping.cpp)
target_link_libraries(gt_mapping ${catkin_LIBRARIES})

install(
  TARGETS
    tmp
    gt_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
