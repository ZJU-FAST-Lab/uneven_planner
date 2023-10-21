#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <random>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

namespace uneven_planner
{
    struct RXS2
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        double z;
        double sigma;
        Eigen::Vector2d zb;
        
        RXS2(): z(0.0), sigma(0.0), zb(Eigen::Vector2d::Zero()) {}
        RXS2(double z_, double sigma_, Eigen::Vector2d zb_): z(z_), sigma(sigma_), zb(zb_) {} 
        inline double getC() { return sqrt(1.0 - zb(0)*zb(0) - zb(1)*zb(1)); }
        inline double getCosXi() { return sqrt(1.0 - zb(0)*zb(0) - zb(1)*zb(1)); }
        inline RXS2 operator+(const RXS2& a)
        {
            return RXS2(z+a.z, sigma+a.sigma, zb+a.zb);
        }
        inline RXS2 operator-(const RXS2& a)
        {
            return RXS2(z-a.z, sigma-a.sigma, zb-a.zb);
        }
        inline RXS2 operator*(const double& a)
        {
            return RXS2(z*a, sigma*a, zb*a);
        }
        inline Eigen::Vector3d toVector()
        {
            return Eigen::Vector3d(sigma, zb.x(), zb.y());
        }
    };

    class UnevenMap
    {
        private:
            // params
            int             iter_num;
            double          ellipsoid_x;
            double          ellipsoid_y;
            double          ellipsoid_z;
            double          xy_resolution, xy_resolution_inv;
            double          yaw_resolution, yaw_resolution_inv;
            double          min_cnormal;
            double          max_rho;
            double          gravity;
            double          mass;
            Eigen::Vector3d map_origin;
            Eigen::Vector3d map_size;
            Eigen::Vector3d min_boundary;
            Eigen::Vector3d max_boundary;
            Eigen::Vector3i min_idx;
            Eigen::Vector3i max_idx;
            Eigen::Vector3i voxel_num;
            string          map_file;
            string          pcd_file;

            //datas
            vector<RXS2>    map_buffer;
            vector<double>  c_buffer;
            vector<char>    occ_buffer;
            vector<char>    occ_r2_buffer;
            pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud;
            pcl::PointCloud<pcl::PointXY>::Ptr world_cloud_plane;
            pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
            pcl::KdTreeFLANN<pcl::PointXY> kd_tree_plane;

            //ros
            ros::Publisher                  origin_pub;
            ros::Publisher                  filtered_pub;
            ros::Publisher                  zb_pub;
            ros::Publisher                  so2_test_pub;
            ros::Timer                      vis_timer;
            sensor_msgs::PointCloud2        origin_cloud_msg;
            sensor_msgs::PointCloud2        filtered_cloud_msg;
            visualization_msgs::MarkerArray so2_test_msg;
            visualization_msgs::Marker      zb_msg;    
            bool                            map_ready = false;

        public:
            UnevenMap() {}
            ~UnevenMap() {}

            static RXS2 filter(Eigen::Vector3d pos, vector<Eigen::Vector3d> points);
            static Eigen::Matrix3d skewSym(Eigen::Vector3d vec);
            static double calYawFromR(Eigen::Matrix3d R);
            static void normSO2(double& yaw);

            void init(ros::NodeHandle& nh);
            bool constructMapInput();
            bool constructMap();
            void visCallback(const ros::TimerEvent& /*event*/);

            inline void getTerrain(const Eigen::Vector3d& pos, RXS2& value);
            inline void getTerrainPos(const Eigen::Vector3d& pos, Eigen::Matrix3d& R, Eigen::Vector3d& p);
            // invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma
            inline void getTerrainVariables(const Eigen::Vector3d& pos, vector<double>& values);
            inline void getTerrainWithGradI(const Eigen::Vector3d& pos, RXS2& value, Eigen::Matrix<double, 4, 3>& grad);
            // invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma
            inline void getAllWithGrad(const Eigen::Vector3d& pos, vector<double>& values, vector<Eigen::Vector3d>& grads);
            
            inline double getGravity(void);
            inline double getMass(void);
            inline double getTerrainSig(const Eigen::Vector3d& pos);
            inline void boundIndex(Eigen::Vector3i& id);
            inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
            inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
            inline int toAddress(const Eigen::Vector3i& id);
            inline int toAddress(const int& x, const int& y, const int& yaw);
            inline bool isInMap(const Eigen::Vector3d& pos);
            inline bool isInMap(const Eigen::Vector3i& idx);
            inline int isOccupancy(const Eigen::Vector3d& pos);
            inline int isOccupancy(const Eigen::Vector3i& id);
            inline int isOccupancyXY(const Eigen::Vector3d& pxy);
            inline int getXYNum();
            inline bool mapReady();

            typedef shared_ptr<UnevenMap> Ptr;
            typedef unique_ptr<UnevenMap> UniPtr;
    };

    inline void UnevenMap::getTerrain(const Eigen::Vector3d& pos, RXS2& value)
    {
        if (!isInMap(pos))
        {
            value = RXS2();
            ROS_WARN("[Uneven Map] pos isn't in map, check it!");
            return;
        }

        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * xy_resolution;
        pos_m(1) -= 0.5 * xy_resolution;
        pos_m(2) -= 0.5 * yaw_resolution;
        normSO2(pos_m(2));

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= xy_resolution_inv;
        diff(1) *= xy_resolution_inv;
        // SO(2) process
        diff(2) = atan2(sin(pos(2)-idx_pos(2)), cos(pos(2)-idx_pos(2))) * yaw_resolution_inv;

        RXS2 values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int yaw = 0; yaw < 2; yaw++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, yaw);
                    boundIndex(current_idx);
                    values[x][y][yaw] = map_buffer[toAddress(current_idx)];
                }

        // value
        RXS2 v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        RXS2 v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        RXS2 v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        RXS2 v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        RXS2 v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        RXS2 v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        value = v0 * (1 - diff[2]) + v1 * diff[2];

        return;
    }

    inline void UnevenMap::getTerrainPos(const Eigen::Vector3d& pos, Eigen::Matrix3d& R, Eigen::Vector3d& p)
    {
        RXS2 rs2;
        getTerrain(pos, rs2);

        Eigen::Vector3d zb(rs2.zb.x(), rs2.zb.y(), rs2.getC());
        Eigen::Vector3d xyaw(cos(pos(2)), sin(pos(2)), 0.0);

        R.col(2) = zb;
        R.col(1) = zb.cross(xyaw).normalized();
        R.col(0) = R.col(1).cross(zb);
        p = pos;
        p(2) = rs2.z;

        return;
    }
    
    // invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma
    inline void UnevenMap::getTerrainVariables(const Eigen::Vector3d& pos, vector<double>& values)
    {
        double inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi;

        RXS2 rs2;
        getTerrain(pos, rs2);

        double c = rs2.getC();
        double inv_c = 1.0 / c;
        double cyaw = cos(pos(2));
        double syaw = sin(pos(2));
        Eigen::Vector2d xyaw(cyaw, syaw);
        Eigen::Vector2d yyaw(-syaw, cyaw);
        double t = xyaw.dot(rs2.zb);
        double s = -yyaw.dot(rs2.zb);
        double sqrt_1_t2 = sqrt(1.0 - t*t);
        double inv_sqrt_1_t2 = 1.0 / sqrt_1_t2;
        double inv_sqrt_1_t2_3 = inv_sqrt_1_t2 * inv_sqrt_1_t2 * inv_sqrt_1_t2;

        inv_cos_vphix = inv_sqrt_1_t2;
        sin_phix = -c * t * inv_sqrt_1_t2;
        inv_cos_vphiy = sqrt_1_t2 * inv_c;
        sin_phiy = s * inv_sqrt_1_t2;
        cos_xi = c;
        inv_cos_xi = inv_c; 

        values.clear();

        values.push_back(inv_cos_vphix);
        values.push_back(sin_phix);
        values.push_back(inv_cos_vphiy);
        values.push_back(sin_phiy);
        values.push_back(cos_xi);
        values.push_back(inv_cos_xi);
        values.push_back(rs2.sigma);
    }

    inline void UnevenMap::getTerrainWithGradI(const Eigen::Vector3d& pos, RXS2& value, Eigen::Matrix<double, 4, 3>& grad)
    {
        if (!isInMap(pos))
        {
            grad.setZero();
            value = RXS2();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * xy_resolution;
        pos_m(1) -= 0.5 * xy_resolution;
        pos_m(2) -= 0.5 * yaw_resolution;
        normSO2(pos_m(2));

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= xy_resolution_inv;
        diff(1) *= xy_resolution_inv;
        // SO(2) process
        diff(2) = atan2(sin(pos(2)-idx_pos(2)), cos(pos(2)-idx_pos(2))) * yaw_resolution_inv;

        RXS2 values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int yaw = 0; yaw < 2; yaw++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, yaw);
                    boundIndex(current_idx);
                    values[x][y][yaw] = map_buffer[toAddress(current_idx)];
                }
        
        // value & grad
        RXS2 v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        RXS2 v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        RXS2 v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        RXS2 v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        RXS2 v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        RXS2 v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        value = v0 * (1 - diff[2]) + v1 * diff[2];

        grad.block<3, 1>(0, 2) = (v1 - v0).toVector() * yaw_resolution_inv;
        grad.block<3, 1>(0, 1) = ((v10 - v00) * (1 - diff[2]) + (v11 - v01) * diff[2]).toVector() * xy_resolution_inv;
        grad.block<3, 1>(0, 0) = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]).toVector();
        grad.block<3, 1>(0, 0) += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]).toVector();
        grad.block<3, 1>(0, 0) += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]).toVector();
        grad.block<3, 1>(0, 0) += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]).toVector();
        grad.block<3, 1>(0, 0) *= xy_resolution_inv;
        grad.row(3) = -(grad.row(1)*value.zb.x() + grad.row(2)*value.zb.y()) / value.getC();

        return;
    }

    // getAllWithGrad: get values and grads of {invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma}
    inline void UnevenMap::getAllWithGrad(const Eigen::Vector3d& pos, vector<double>& values, vector<Eigen::Vector3d>& grads)
    {
        double inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi;
        Eigen::Vector3d grad_inv_cos_vphix, grad_sin_phix, grad_inv_cos_vphiy, grad_sin_phiy, grad_cos_xi, grad_inv_cos_xi;

        RXS2 rs2;
        Eigen::Matrix<double, 4, 3> rs2_grad;

        getTerrainWithGradI(pos, rs2, rs2_grad);
        double c = rs2.getC();
        double inv_c = 1.0 / c;
        double cyaw = cos(pos(2));
        double syaw = sin(pos(2));
        Eigen::Vector2d xyaw(cyaw, syaw);
        Eigen::Vector2d yyaw(-syaw, cyaw);
        double t = xyaw.dot(rs2.zb);
        double s = -yyaw.dot(rs2.zb);
        double sqrt_1_t2 = sqrt(1.0 - t*t);
        double inv_sqrt_1_t2 = 1.0 / sqrt_1_t2;
        double inv_sqrt_1_t2_3 = inv_sqrt_1_t2 * inv_sqrt_1_t2 * inv_sqrt_1_t2;
        Eigen::Vector3d dt = rs2_grad.block<2, 3>(1, 0).transpose()*xyaw;
        Eigen::Vector3d ds = -rs2_grad.block<2, 3>(1, 0).transpose()*yyaw;
        dt(2) -= s;
        ds(2) += t;

        inv_cos_vphix = inv_sqrt_1_t2;
        sin_phix = -c * t * inv_sqrt_1_t2;
        inv_cos_vphiy = sqrt_1_t2 * inv_c;
        sin_phiy = s * inv_sqrt_1_t2;
        cos_xi = c;
        inv_cos_xi = inv_c; 

        grad_inv_cos_vphix = t * inv_sqrt_1_t2_3 * dt;
        grad_sin_phix = -(t * inv_sqrt_1_t2 * rs2_grad.row(3).transpose() + inv_sqrt_1_t2_3 * c * dt);
        grad_inv_cos_vphiy = -inv_c * (t * inv_sqrt_1_t2 * dt + sqrt_1_t2 * inv_c * rs2_grad.row(3).transpose());
        grad_sin_phiy = inv_sqrt_1_t2 * ds + t * inv_sqrt_1_t2_3 * s * dt;
        grad_cos_xi = rs2_grad.row(3);
        grad_inv_cos_xi = -inv_cos_xi * inv_cos_xi * rs2_grad.row(3);
        
        values.clear();
        grads.clear();

        values.push_back(inv_cos_vphix);
        values.push_back(sin_phix);
        values.push_back(inv_cos_vphiy);
        values.push_back(sin_phiy);
        values.push_back(cos_xi);
        values.push_back(inv_cos_xi);
        values.push_back(rs2.sigma);

        grads.push_back(grad_inv_cos_vphix);
        grads.push_back(grad_sin_phix);
        grads.push_back(grad_inv_cos_vphiy);
        grads.push_back(grad_sin_phiy);
        grads.push_back(grad_cos_xi);
        grads.push_back(grad_inv_cos_xi);
        grads.push_back(rs2_grad.row(0));

        return;
    }

    inline double UnevenMap::getGravity(void)
    {
        return gravity;
    }

    inline double UnevenMap::getMass(void)
    {
        return mass;
    }

    inline double UnevenMap::getTerrainSig(const Eigen::Vector3d& pos)
    {
        RXS2 value;

        getTerrain(pos, value);

        return value.sigma;
    }

    inline void UnevenMap::boundIndex(Eigen::Vector3i& id)
    {
        id(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id(1) = max(min(id(1), max_idx(1)), min_idx(1));
        // SO(2) process
        while (id(2) > max_idx(2))
            id(2) -= voxel_num(2);
        while (id(2) < min_idx(2))
            id(2) += voxel_num(2);

        return;
    }

    inline void UnevenMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
    {
        id(0) = floor((pos(0) - map_origin(0)) * xy_resolution_inv);
        id(1) = floor((pos(1) - map_origin(1)) * xy_resolution_inv);
        id(2) = floor((pos(2) - map_origin(2)) * yaw_resolution_inv);
        return;
    }

    inline void UnevenMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
    {
        pos(0) = (id(0) + 0.5) * xy_resolution  + map_origin(0);
        pos(1) = (id(1) + 0.5) * xy_resolution  + map_origin(1);
        pos(2) = (id(2) + 0.5) * yaw_resolution + map_origin(2);
        return;
    }

    inline int UnevenMap::toAddress(const Eigen::Vector3i& id) 
    {
        return id(0) * voxel_num(1)*voxel_num(2) + id(1) * voxel_num(2) + id(2);
    }

    inline int UnevenMap::toAddress(const int& x, const int& y, const int& yaw) 
    {
        return x * voxel_num(1)*voxel_num(2) + y * voxel_num(2) + yaw;
    }
    
    inline bool UnevenMap::isInMap(const Eigen::Vector3d& pos) 
    {
        if (pos(0) < min_boundary(0) + 1e-4 || \
            pos(1) < min_boundary(1) + 1e-4 || \
            pos(2) < min_boundary(2) + 1e-4     ) 
        {
            return false;
        }

        if (pos(0) > max_boundary(0) - 1e-4 || \
            pos(1) > max_boundary(1) - 1e-4 || \
            pos(2) > max_boundary(2) - 1e-4     ) 
        {
            return false;
        }

        return true;
    }

    inline bool UnevenMap::isInMap(const Eigen::Vector3i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 || \
            idx(1) > voxel_num(1) - 1 || \
            idx(2) > voxel_num(2) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline int UnevenMap::isOccupancy(const Eigen::Vector3d& pos)
    {
        Eigen::Vector3i id;

        posToIndex(pos, id);
        
        return isOccupancy(id);
    }

    inline int UnevenMap::isOccupancy(const Eigen::Vector3i& id)
    {
        if (!isInMap(id))
            return -1;

        return int(occ_buffer[toAddress(id)]);
    }

    inline int UnevenMap::isOccupancyXY(const Eigen::Vector3d& pxy)
    {
        Eigen::Vector3i id;

        posToIndex(pxy, id);

        if (!isInMap(id))
            return -1;

        return int(occ_r2_buffer[id(0)*voxel_num(1) + id(1)]);
    }

    inline int UnevenMap::getXYNum()
    {
        return voxel_num(0)*voxel_num(1);
    }

    inline bool UnevenMap::mapReady()
    {
        return map_ready;
    }
}