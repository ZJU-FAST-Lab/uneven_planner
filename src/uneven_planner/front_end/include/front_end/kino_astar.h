#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <pcl/point_cloud.h>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "uneven_map/uneven_map.h"

#define inf 1 >> 30
#define PI_X_2 6.283185307179586
#define CLOSE 'a'
#define OPEN 'b'
#define NOT_EXPAND 'c'

namespace uneven_planner
{
    class PathNode
    {
        public:
            Eigen::Vector3i index;
            Eigen::Vector3d state;
            Eigen::Vector2d input;
            double g_score;
            double f_score;
            char node_state;
            PathNode* parent;
            PathNode(): parent(nullptr), node_state(NOT_EXPAND) {}
            ~PathNode() {}
    };
    typedef PathNode* PathNodePtr;

    class NodeComparator
    {
        public:
            template <class NodePtr>
            bool operator()(NodePtr node1, NodePtr node2) 
            {
                return node1->f_score > node2->f_score;
            }
    };

    template <typename T>
    struct matrix_hash : std::unary_function<T, size_t> 
    {
        std::size_t operator()(T const& matrix) const 
        {
            size_t seed = 0;
            for (long int i = 0; i < matrix.size(); ++i) 
            {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    template <class NodePtr>
    class NodeHashTable 
    {
        private:
            std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_;

        public:
            NodeHashTable() {}
            ~NodeHashTable() {}

            void insert(Eigen::Vector3i idx,NodePtr node )
            {
                data_.insert(std::make_pair(idx, node));
            }

            NodePtr find(Eigen::Vector3i idx) 
            {
                auto iter = data_.find(idx);
                return iter == data_.end() ? NULL : iter->second;
            }

            void clear() { data_.clear(); }
    };


    class KinoAstar
    {
        private:
            // datas
            UnevenMap::Ptr uneven_map;
            std::vector<PathNodePtr> path_node_pool;
            NodeHashTable<PathNodePtr> expanded_nodes;
            pcl::PointCloud<pcl::PointXYZ> expanded_points;
            std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set;
            ompl::base::StateSpacePtr shot_finder;
            std::vector<Eigen::Vector3d> front_end_path;
            std::vector<Eigen::Vector3d> shot_path;
            
            // params
            int allocate_num;
            double yaw_resolution, yaw_resolution_inv;
            double lambda_heu;
            double weight_r2;
            double weight_so2;
            double weight_v_change;
            double weight_delta_change;
            double weight_sigma;
            double time_interval;
            double oneshot_range;
            double collision_interval;
            double wheel_base;
            double max_steer;
            double max_vel;
            double tie_breaker = 1.0 + 1.0 / 10000;
            bool in_test;

            // ros
            ros::Subscriber wps_sub;
            ros::Subscriber odom_sub;
            ros::Publisher front_end_pub;
            ros::Publisher whole_body_pub;
            ros::Publisher expanded_pub;
            visualization_msgs::Marker model;
            Eigen::Vector3d odom_pos;

        public:
            KinoAstar() {}
            ~KinoAstar()
            {
                for (int i = 0; i < allocate_num; i++)
                    delete path_node_pool[i];
            }
            
            void init(ros::NodeHandle& nh);
            void visFrontEnd();
            void visExpanded();
            void rcvWpsCallBack(const geometry_msgs::PoseStamped msg);
            void rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg);
            std::vector<Eigen::Vector3d> plan(const Eigen::Vector3d& start_state, const Eigen::Vector3d& end_state);

            inline void setEnvironment(const UnevenMap::Ptr& env);
            inline void stateToIndex(const Eigen::Vector3d& state, Eigen::Vector3i& idx); 
            inline int yawToIndex(const double &yaw);
            inline double normalizeAngle(const double &angle);
            inline double dAngle(const double &angle1, const double &angle2);
            inline double getHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
            inline void stateTransit(const Eigen::Vector3d &state0, Eigen::Vector3d &state1, \
                                    const Eigen::Vector2d &ctrl_input, const double& T);
            inline void asignShotTraj(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2);
            inline void retrievePath(PathNodePtr end_node);
            inline std::vector<geometry_msgs::Point> getModel(const Eigen::Vector3d state);

            typedef shared_ptr<KinoAstar> Ptr;
            typedef unique_ptr<KinoAstar> UniPtr;
    };

    inline void KinoAstar::setEnvironment(const UnevenMap::Ptr& env)
    {
        this->uneven_map = env;
        allocate_num = uneven_map->getXYNum();
        for (int i = 0; i < allocate_num; i++)
        {
            path_node_pool.push_back(new PathNode());
        }
    }

    inline int KinoAstar::yawToIndex(const double &yaw)
    {
        double nor_yaw = normalizeAngle(yaw);
        int idx = floor((nor_yaw + M_PI) * yaw_resolution_inv);
        return idx;
    }
    
    inline void KinoAstar::stateToIndex(const Eigen::Vector3d& state, Eigen::Vector3i& idx)
    {
        uneven_map->posToIndex(state, idx);
        idx(2) = floor((normalizeAngle(state(2)) + M_PI) * yaw_resolution_inv);
    }

    inline double KinoAstar::normalizeAngle(const double &angle)
    {
        double nor_angle = angle;

        while (nor_angle>M_PI)
            nor_angle -= PI_X_2;

        while (nor_angle<-M_PI)
            nor_angle += PI_X_2;

        return nor_angle;
    }

    inline double KinoAstar::dAngle(const double &angle1, const double &angle2)
    {
        double da = angle1 - angle2;

        return normalizeAngle(da);
    }

    inline double KinoAstar::getHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2)
    {
        return tie_breaker * (x1.head(2) - x2.head(2)).norm();
    }

    inline void KinoAstar::stateTransit(const Eigen::Vector3d &state0, Eigen::Vector3d &state1, \
                                        const Eigen::Vector2d &ctrl_input, const double& T)
    {
        double v = ctrl_input[0];
        double delta = ctrl_input[1];
        double s = v * T;
        double y = s * tan(delta) / wheel_base;
        
        if(fabs(delta) > 1e-4)
        {
            double r = s / y;
            state1[0] = state0[0] + r*(sin(state0[2]+y)-sin(state0[2]));
            state1[1] = state0[1] - r*(cos(state0[2]+y)-cos(state0[2]));
            state1[2] = state0[2] + y;
            state1[2] = normalizeAngle(state1[2]);
        }
        else
        {
            state1[0] = state0[0] + s * cos(state0[2]);
            state1[1] = state0[1] + s * sin(state0[2]);
            state1[2] = state0[2];
        }
    }

    inline void KinoAstar::asignShotTraj(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2)
    {
        shot_path.clear();
        namespace ob = ompl::base;
        namespace og = ompl::geometric;
        ob::ScopedState<> from(shot_finder), to(shot_finder), s(shot_finder);
        from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
        to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
        std::vector<double> reals;
        double len = shot_finder->distance(from(), to());

        for (double l = 0.0; l <=len; l += collision_interval)
        {
            shot_finder->interpolate(from(), to(), l/len, s());
            reals = s.reals();
            shot_path.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
        }

        for (size_t i=0; i<shot_path.size(); i++)
        {
            // if (uneven_map->isOccupancy(shot_path[i])==1)
            if (uneven_map->isOccupancyXY(shot_path[i])==1)
            {
                shot_path.clear();
                break;
            }
        }

        return;
    }

    inline void KinoAstar::retrievePath(PathNodePtr end_node)
    {
        for (int i=shot_path.size()-1; i>=0; i--)
        {
            front_end_path.push_back(shot_path[i]);
        }

        PathNodePtr cur_node = end_node;
        front_end_path.push_back(cur_node->state);

        while (cur_node->parent != NULL)
        {
            cur_node = cur_node->parent;
            front_end_path.push_back(cur_node->state);
        }

        reverse(front_end_path.begin(), front_end_path.end());

        return;
    }

    inline std::vector<geometry_msgs::Point> KinoAstar::getModel(const Eigen::Vector3d state)
    {
        std::vector<geometry_msgs::Point> models;

        geometry_msgs::Point p[4];
        p[0].x = 1.5;
        p[0].y = 1.0;
        p[0].z = 0.0;
        p[1].x = 1.5;
        p[1].y = -1.0;
        p[1].z = 0.0;
        p[2].x = -1.5;
        p[2].y = -1.0;
        p[2].z = 0.0;
        p[3].x = -1.5;
        p[3].y = 1.0;
        p[3].z = 0.0;

        double cyaw = cos(state[2]);
        double syaw = sin(state[2]);
        for (size_t i=0; i<4; i++)
        {
            double x = p[i].x;
            double y = p[i].y;
            p[i].x = state[0] + x * cyaw - y * syaw;
            p[i].y = state[1] + x * syaw + y * cyaw;
        }

        for (size_t i=0; i<4; i++)
        {
            models.push_back(p[i]);
            models.push_back(p[(i+1)%4]);
        }

        return models;
    }

}
