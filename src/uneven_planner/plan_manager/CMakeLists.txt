cmake_minimum_required(VERSION 2.8.3)
project(plan_manager)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  uneven_map
  front_end
  back_end
  mpc_controller
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manager
 CATKIN_DEPENDS roscpp std_msgs uneven_map visualization_msgs front_end back_end mpc_controller
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_library(plan_manager 
    src/plan_manager.cpp
)

target_link_libraries(plan_manager
    ${catkin_LIBRARIES} 
)

add_executable(manager_node src/manager_node.cpp )

target_link_libraries(manager_node
    plan_manager
    ${catkin_LIBRARIES} 
)
