cmake_minimum_required(VERSION 2.8.3)
project(back_end)

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
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES back_end
 CATKIN_DEPENDS roscpp std_msgs uneven_map visualization_msgs front_end
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_library(back_end 
    src/alm_traj_opt.cpp
)

target_link_libraries(back_end
    ${catkin_LIBRARIES} 
)

add_executable(back_end_node src/back_end_node.cpp )

target_link_libraries(back_end_node
    back_end
    ${catkin_LIBRARIES} 
)
