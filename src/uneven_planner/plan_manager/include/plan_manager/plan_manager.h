#pragma once

#include <fstream>
#include <string.h>
#include <random>
#include <time.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "uneven_map/uneven_map.h"
#include "front_end/kino_astar.h"
#include "back_end/alm_traj_opt.h"
#include "mpc_controller/SE2Traj.h"

namespace uneven_planner
{
    class PlanManager
    {
        private:
            bool in_plan = false;
            double piece_len;
            double mean_vel;
            double init_time_times;
            double yaw_piece_times;
            double init_sig_vel;
            Eigen::Vector3d odom_pos;
            string bk_dir;

            UnevenMap::Ptr uneven_map;
            KinoAstar::Ptr kino_astar;
            ALMTrajOpt traj_opt;
            SE2Trajectory opted_traj;

            ros::Publisher traj_pub;
            ros::Subscriber odom_sub;
            ros::Subscriber target_sub;
            
        public:
            void init(ros::NodeHandle& nh);
            void rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg);
            void rcvWpsCallBack(const geometry_msgs::PoseStamped msg);
    };
}
