#include "mpc/mpc.h"
#include <ros/ros.h>

int main( int argc, char * argv[] )
{ 
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  MPC mpc_tracker;

  mpc_tracker.init(nh);

  ros::spin();

  return 0;
}