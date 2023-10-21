cmake_minimum_required(VERSION 2.8.3)
project(mpc_controller)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_message_files(FILES
  SE2Traj.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  message_runtime
)

include_directories( 
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
)

add_library(mpc_controller
  src/mpc.cpp
)

add_dependencies(mpc_controller mpc_controller_gencpp)

target_link_libraries(mpc_controller
  OsqpEigen::OsqpEigen
  ${catkin_LIBRARIES}
)

add_executable(mpc_controller_node
  src/mpc_node.cpp
)
target_link_libraries(mpc_controller_node
  mpc_controller
  ${catkin_LIBRARIES}
)