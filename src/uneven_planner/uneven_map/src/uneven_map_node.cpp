#include "uneven_map/uneven_map.h"
#include <ros/ros.h>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{ 
  ros::init(argc, argv, "uneven_map_node");
  ros::NodeHandle nh("~");

  UnevenMap uneven_map;

  uneven_map.init(nh);

  ros::spin();

  return 0;
}