#include "plan_manager/plan_manager.h"

namespace uneven_planner
{
    void PlanManager::init(ros::NodeHandle& nh)
    {
        nh.getParam("manager/piece_len", piece_len);
        nh.getParam("manager/mean_vel", mean_vel);
        nh.getParam("manager/init_time_times", init_time_times);
        nh.getParam("manager/yaw_piece_times", yaw_piece_times);
        nh.getParam("manager/init_sig_vel", init_sig_vel);
        nh.param<string>("manager/bk_dir", bk_dir, "xxx");

        uneven_map.reset(new UnevenMap);
        kino_astar.reset(new KinoAstar);
        
        uneven_map->init(nh);
        kino_astar->init(nh);
        kino_astar->setEnvironment(uneven_map);
        traj_opt.init(nh);
        traj_opt.setFrontend(kino_astar);
        traj_opt.setEnvironment(uneven_map);

        traj_pub = nh.advertise<mpc_controller::SE2Traj>("traj", 1);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &PlanManager::rcvOdomCallBack, this);
        target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &PlanManager::rcvWpsCallBack, this);
        
        return;
    }

    void PlanManager::rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg)
    {
        odom_pos(0) = msg->pose.pose.position.x;
        odom_pos(1) = msg->pose.pose.position.y;
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, \
                             msg->pose.pose.orientation.x, \
                             msg->pose.pose.orientation.y, \
                             msg->pose.pose.orientation.z  );
        Eigen::Matrix3d R(q);
        odom_pos(2) = UnevenMap::calYawFromR(R);
    }

    void PlanManager::rcvWpsCallBack(const geometry_msgs::PoseStamped msg)
    {
        if (in_plan || !uneven_map->mapReady())
            return;

        in_plan = true;
        
        Eigen::Vector3d end_state(msg.pose.position.x, \
                                  msg.pose.position.y, \
                                  atan2(2.0*msg.pose.orientation.z*msg.pose.orientation.w, \
                                        2.0*pow(msg.pose.orientation.w, 2)-1.0)             );
        
        std::vector<Eigen::Vector3d> init_path = kino_astar->plan(odom_pos, end_state);
        if (init_path.empty())
        {
            in_plan = false;
            return;
        }

        // smooth yaw
        double dyaw;
        for (size_t i=0; i<init_path.size()-1; i++)
        {
            dyaw = init_path[i+1].z() - init_path[i].z();
            while (dyaw >= M_PI / 2)
            {
                init_path[i+1].z() -= M_PI * 2;
                dyaw = init_path[i+1].z() - init_path[i].z();
            }
            while (dyaw <= -M_PI / 2)
            {
                init_path[i+1].z() += M_PI * 2;
                dyaw = init_path[i+1].z() - init_path[i].z();
            }
        }

        // init solution
        Eigen::Matrix<double, 2, 3> init_xy, end_xy;
        Eigen::Vector3d init_yaw, end_yaw;
        Eigen::MatrixXd inner_xy;
        Eigen::VectorXd inner_yaw;
        double total_time;
    
        init_xy << init_path[0].x(), 0.0, 0.0, \
                   init_path[0].y(), 0.0, 0.0;
        end_xy << init_path.back().x(), 0.0, 0.0, \
                   init_path.back().y(), 0.0, 0.0;
        init_yaw << init_path[0].z(), 0.0, 0.0;
        end_yaw << init_path.back().z(), 0.0, 0.0;

        init_xy.col(1) << init_sig_vel * cos(init_yaw(0)), init_sig_vel * sin(init_yaw(0));
        end_xy.col(1) << init_sig_vel * cos(end_yaw(0)), init_sig_vel * sin(end_yaw(0));
        
        double temp_len_yaw = 0.0;
        double temp_len_pos = 0.0;
        double total_len = 0.0;
        double piece_len_yaw = piece_len / yaw_piece_times;
        std::vector<Eigen::Vector2d> inner_xy_node;
        std::vector<double> inner_yaw_node;
        for (int k=0; k<init_path.size()-1; k++)
        {
            double temp_seg = (init_path[k+1] - init_path[k]).head(2).norm();
            temp_len_yaw += temp_seg;
            temp_len_pos += temp_seg;
            total_len += temp_seg;
            while (temp_len_yaw > piece_len_yaw)
            {
                double temp_yaw = init_path[k].z() + (1.0 - (temp_len_yaw-piece_len_yaw) / temp_seg) * (init_path[k+1] - init_path[k]).z();
                inner_yaw_node.push_back(temp_yaw);
                temp_len_yaw -= piece_len_yaw;
            }
            while (temp_len_pos > piece_len)
            {
                Eigen::Vector3d temp_node = init_path[k] + (1.0 - (temp_len_pos-piece_len) / temp_seg) * (init_path[k+1] - init_path[k]);
                inner_xy_node.push_back(temp_node.head(2));
                // inner_yaw_node.push_back(temp_node.z());
                temp_len_pos -= piece_len;
            }
        }
        total_time = total_len / mean_vel * init_time_times;
        inner_xy.resize(2, inner_xy_node.size());
        inner_yaw.resize(inner_yaw_node.size());
        for (int i=0; i<inner_xy_node.size(); i++)
        {
            inner_xy.col(i) = inner_xy_node[i];
        }
        for (int i=0; i<inner_yaw_node.size(); i++)
        {
            inner_yaw(i) = inner_yaw_node[i];
        }
    
        traj_opt.optimizeSE2Traj(init_xy, end_xy, inner_xy, \
                        init_yaw, end_yaw, inner_yaw, total_time);
        
        // visualization
        SE2Trajectory back_end_traj = traj_opt.getTraj();
        traj_opt.visSE2Traj(back_end_traj);
        traj_opt.visSE3Traj(back_end_traj);
        std::vector<double> max_terrain_value = traj_opt.getMaxVxAxAyCurAttSig(back_end_traj);
        std::cout << "equal error: "<< back_end_traj.getNonHolError() << std::endl;
        std::cout << "max vx rate: "<< max_terrain_value[0] << std::endl;
        std::cout << "max ax rate: "<< max_terrain_value[1] << std::endl;
        std::cout << "max ay rate: "<< max_terrain_value[2] << std::endl;
        std::cout << "max cur:     "<< max_terrain_value[3] << std::endl;
        std::cout << "min cosxi:   "<< -max_terrain_value[4] << std::endl;
        std::cout << "max sigma:   "<< max_terrain_value[5] << std::endl;

        // publish to mpc controller
        mpc_controller::SE2Traj traj_msg;
        traj_msg.start_time = ros::Time::now();
        traj_msg.init_v.x = 0.0;
        traj_msg.init_v.y = 0.0;
        traj_msg.init_v.z = 0.0;
        traj_msg.init_a.x = 0.0;
        traj_msg.init_a.y = 0.0;
        traj_msg.init_a.z = 0.0;
        for (int i=0; i<back_end_traj.pos_traj.getPieceNum(); i++)
        {
            geometry_msgs::Point pospt;
            Eigen::Vector2d pos = back_end_traj.pos_traj[i].getValue(0.0);
            pospt.x = pos[0];
            pospt.y = pos[1];
            traj_msg.pos_pts.push_back(pospt);
            traj_msg.posT_pts.push_back(back_end_traj.pos_traj[i].getDuration());
        }
        geometry_msgs::Point pospt;
        Eigen::Vector2d pos = back_end_traj.pos_traj.getValue(back_end_traj.pos_traj.getTotalDuration());
        pospt.x = pos[0];
        pospt.y = pos[1];
        traj_msg.pos_pts.push_back(pospt);

        for(int i = 0; i < back_end_traj.yaw_traj.getPieceNum(); i ++){
            geometry_msgs::Point anglept;
            Eigen::VectorXd angle = back_end_traj.yaw_traj[i].getValue(0.0);
            anglept.x = angle[0];
            traj_msg.angle_pts.push_back(anglept);
            traj_msg.angleT_pts.push_back(back_end_traj.yaw_traj[i].getDuration());
        }
        geometry_msgs::Point anglept;
        Eigen::VectorXd angle = back_end_traj.yaw_traj.getValue(back_end_traj.yaw_traj.getTotalDuration());
        anglept.x = angle[0];
        traj_msg.angle_pts.push_back(anglept);
        traj_pub.publish(traj_msg);
        in_plan = false;

        return;
    }
}