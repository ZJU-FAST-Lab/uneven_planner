#include "front_end/kino_astar.h"
#include <ros/ros.h>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh("~");

    KinoAstar kino_astar;
    UnevenMap uneven_map;
    UnevenMap::Ptr uneven_map_ptr = make_shared<UnevenMap>(uneven_map);
    
    uneven_map_ptr->init(nh);
    kino_astar.init(nh);
    kino_astar.setEnvironment(uneven_map_ptr);

    ros::spin();

    return 0;
}