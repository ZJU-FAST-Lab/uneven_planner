#include <front_end/kino_astar.h>

namespace uneven_planner
{
    void KinoAstar::init(ros::NodeHandle& nh)
    {
        nh.param("kino_astar/yaw_resolution", yaw_resolution, 3.15);
        nh.param("kino_astar/lambda_heu", lambda_heu, 1.0);
        nh.param("kino_astar/weight_r2", weight_r2, 1.0);
        nh.param("kino_astar/weight_so2", weight_so2, 1.0);
        nh.param("kino_astar/weight_v_change", weight_v_change, 0.0);
        nh.param("kino_astar/weight_delta_change", weight_delta_change, 0.0);
        nh.param("kino_astar/weight_sigma", weight_sigma, 0.0);
        nh.param("kino_astar/time_interval", time_interval, 1.0);
        nh.param("kino_astar/collision_interval", collision_interval, 1.0);
        nh.param("kino_astar/oneshot_range", oneshot_range, 1.0);
        nh.param("kino_astar/wheel_base", wheel_base, 1.0);
        nh.param("kino_astar/max_steer", max_steer, 1.0);
        nh.param("kino_astar/max_vel", max_vel, 1.0);
        nh.param("kino_astar/in_test", in_test, false);

        whole_body_pub = nh.advertise<visualization_msgs::Marker>("/kino_astar/wholebody_path", 0);
        front_end_pub = nh.advertise<nav_msgs::Path>("/kino_astar/path", 0);
        if (in_test)
        {
            wps_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &KinoAstar::rcvWpsCallBack, this);
            odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &KinoAstar::rcvOdomCallBack, this);
            expanded_pub = nh.advertise<sensor_msgs::PointCloud2>("/kino_astar/expanded_points", 0);
        }

        yaw_resolution_inv = 1.0 / yaw_resolution;

        shot_finder = std::make_shared<ompl::base::DubinsStateSpace>(wheel_base / tan(max_steer));

        model.type = visualization_msgs::Marker::LINE_LIST;
        model.header.frame_id = "world";
        model.id = 10;
        model.pose.orientation.w = 1.0;
        model.color.r = 1.0;
        model.color.a = 1.0;
        model.color.b = 1.0;
        model.scale.x = 0.1;
    }

    void KinoAstar::rcvWpsCallBack(const geometry_msgs::PoseStamped msg)
    {
        Eigen::Vector3d end_state(msg.pose.position.x, \
                                  msg.pose.position.y, \
                                  atan2(2.0*msg.pose.orientation.z*msg.pose.orientation.w, \
                                        2.0*pow(msg.pose.orientation.w, 2)-1.0)             );
        
        plan(odom_pos, end_state);
    }

    void KinoAstar::rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg)
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

    std::vector<Eigen::Vector3d> KinoAstar::plan(const Eigen::Vector3d& start_state, const Eigen::Vector3d& end_state)
    {
        // reset
        int use_node_num = 0;
        int iter_num = 0;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set.swap(empty_queue);
        front_end_path.clear();
        expanded_nodes.clear();
        expanded_points.clear();
        for (int i = 0; i < use_node_num; i++)
        {
            PathNodePtr node = path_node_pool[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }
        
        Eigen::Vector3d end_pt(end_state.head(3));
        // if (uneven_map->isOccupancy(end_state) == 1)
        if (uneven_map->isOccupancy(start_state) == 1)
        {
            ROS_ERROR("start is not free!!!");
            return front_end_path;
        }
        if (uneven_map->isOccupancyXY(end_state) == 1)
        {
            ROS_ERROR("goal is not free!!!");
            return front_end_path;
        }
        ros::Time t0 = ros::Time::now();
        PathNodePtr cur_node = path_node_pool[0];
        cur_node->parent = NULL;
        cur_node->state = start_state.head(3);
        cur_node->state(2) = normalizeAngle(cur_node->state(2));
        stateToIndex(cur_node->state, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->input = Eigen::Vector2d(0.0, 0.0);
        cur_node->f_score = lambda_heu * getHeu(cur_node->state, end_pt);
        cur_node->node_state = OPEN;

        open_set.push(cur_node);
        use_node_num += 1;
        expanded_nodes.insert(cur_node->index, cur_node);
        
        while (!open_set.empty())
        {
            cur_node = open_set.top();

            if((cur_node->state.head(2) - end_pt.head(2)).norm() < oneshot_range)
            {
                ros::Time t1 = ros::Time::now();
                asignShotTraj(cur_node->state, end_state);
                if (!shot_path.empty())
                {
                    std::cout << "one-shot time: " << (ros::Time::now()-t1).toSec()*1000 << " ms"<<std::endl;
                    std::cout << "front all time: " << (ros::Time::now()-t0).toSec()*1000 << " ms"<<std::endl;
                    retrievePath(cur_node);
                    visFrontEnd();
                    return front_end_path;
                }
            }

            open_set.pop();
            cur_node->node_state = CLOSE;
            iter_num += 1;

            Eigen::Vector3d cur_state = cur_node->state;
            Eigen::Vector3d pro_state;
            Eigen::Vector2d ctrl_input;
            std::vector<Eigen::Vector2d> inputs;

            for (double v = 0; v <= max_vel + 1e-3; v += 0.5*max_vel)
            {
                for (double steer = -max_steer; steer <= max_steer + 1e-3; steer += 0.5*max_steer)
                {
                    ctrl_input << v, steer;
                    inputs.push_back(ctrl_input);
                }
            }

            for (size_t i=0; i<inputs.size(); i++)
            {
                Eigen::Vector2d input = inputs[i];
                stateTransit(cur_state, pro_state, input, time_interval);
                expanded_points.points.push_back(pcl::PointXYZ(cur_state.x(), cur_state.y(), 0.0));
                // visExpanded();

                if (!uneven_map->isInMap(pro_state))
                {
                    // std::cout << "[Kino Astar]: out of map range" << std::endl;
                    continue;
                }

                Eigen::Vector3i pro_id;
                PathNodePtr pro_node;

                stateToIndex(pro_state, pro_id);
                pro_node = expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == CLOSE)
                {
                    continue;
                }

                Eigen::Vector3d xt;
                int occ = false;
                double arc = input(0) * time_interval;
                double temp_ct = collision_interval / arc * time_interval;
                for (double t = temp_ct; t <= time_interval+1e-3; t+=temp_ct)
                {
                    stateTransit(cur_state, xt, input, t);
                    // occ = uneven_map->isOccupancy(xt);
                    occ = uneven_map->isOccupancyXY(xt);

                    if (occ == 1)
                        break;
                }
                if (occ == 1)
                    continue;

                double tmp_g_score = 0.0;
                double tmp_f_score = 0.0;
                tmp_g_score += weight_r2 * arc;
                tmp_g_score += weight_so2 * fabs(input(1)) * arc;
                tmp_g_score += weight_v_change * std::fabs(input(0)-cur_node->input(0));
                tmp_g_score += weight_delta_change * std::fabs(input(1)-cur_node->input(1));
                tmp_g_score += weight_sigma * uneven_map->getTerrainSig(pro_state);
                tmp_g_score += cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu * getHeu(pro_state, end_pt);

                if (pro_node == NULL)
                {
                    pro_node = path_node_pool[use_node_num];
                    pro_node->index = pro_id;
                    pro_node->state = pro_state;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->input = input;
                    pro_node->parent = cur_node;
                    pro_node->node_state = OPEN;
                    open_set.push(pro_node);

                    expanded_nodes.insert(pro_id, pro_node);
                    use_node_num ++;

                    if (use_node_num == allocate_num)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return front_end_path;
                    }
                }
                else if (pro_node->node_state == OPEN)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        pro_node->index = pro_id;
                        pro_node->state = pro_state;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = input;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }

        std::cout << "Kino Astar Failed, No path!!!" << std::endl;

        return front_end_path;
    }

    void KinoAstar::visFrontEnd()
    {
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped path_point;

        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();
        model.points.clear();
        for (size_t i=0; i<front_end_path.size(); i++)
        {
            path_point.pose.position.x = front_end_path[i].x();
            path_point.pose.position.y = front_end_path[i].y();
            path_point.pose.position.z = 0.0;
            path_point.pose.orientation.w = cos(front_end_path[i].z()/2.0);
            path_point.pose.orientation.x = 0.0;
            path_point.pose.orientation.y = 0.0;
            path_point.pose.orientation.z = sin(front_end_path[i].z()/2.0);
            path_msg.poses.push_back(path_point);

            std::vector<geometry_msgs::Point> model_points = getModel(front_end_path[i]);
            for (size_t j=0; j<model_points.size(); j++)
                model.points.push_back(model_points[j]);
        }

        front_end_pub.publish(path_msg);
        whole_body_pub.publish(model);
    }

    void KinoAstar::visExpanded()
    {
        if (in_test)
        {
            sensor_msgs::PointCloud2 expanded_msg;
            pcl::toROSMsg(expanded_points, expanded_msg);
            expanded_msg.header.stamp = ros::Time::now();
            expanded_msg.header.frame_id = "world";
            expanded_pub.publish(expanded_msg);
        }
    }
}