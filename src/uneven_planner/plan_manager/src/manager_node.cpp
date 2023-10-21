#include "plan_manager/plan_manager.h"
#include <ros/ros.h>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "manager_node");
    ros::NodeHandle nh("~");

    PlanManager plan_manager;
    plan_manager.init(nh);

    ros::spin();

    return 0;
}