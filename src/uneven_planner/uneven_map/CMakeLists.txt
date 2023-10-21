cmake_minimum_required(VERSION 3.0.2)
project(uneven_map CXX)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ENABLE_CUDA false)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uneven_map
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs visualization_msgs
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(uneven_map
  src/uneven_map.cpp
)

add_executable (uneven_map_node src/uneven_map_node.cpp )

target_link_libraries(uneven_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(uneven_map_node
  uneven_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)