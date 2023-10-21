#include "back_end/alm_traj_opt.h"
#include <ros/ros.h>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh("~");

    ALMTrajOpt traj_opt;
    KinoAstar kino_astar;
    UnevenMap uneven_map;
    UnevenMap::Ptr uneven_map_ptr = make_shared<UnevenMap>(uneven_map);
    KinoAstar::Ptr kino_astar_ptr = make_shared<KinoAstar>(kino_astar);
    
    uneven_map_ptr->init(nh);
    kino_astar_ptr->init(nh);
    kino_astar_ptr->setEnvironment(uneven_map_ptr);
    traj_opt.init(nh);
    traj_opt.setFrontend(kino_astar_ptr);
    traj_opt.setEnvironment(uneven_map_ptr);

    ros::spin();

    return 0;
}