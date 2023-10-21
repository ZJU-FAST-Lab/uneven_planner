#pragma once

#include <thread>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "uneven_map/uneven_map.h"
#include "front_end/kino_astar.h"
#include "utils/se2traj.hpp"
#include "utils/lbfgs.hpp"

namespace uneven_planner
{
    constexpr double delta_sigl = 0.01;
    constexpr double cur_scale = 10.0;
    constexpr double sig_scale = 1000.0;
    constexpr double scale_trick_jerk = 1000.0;

    class ALMTrajOpt
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        public:
            // params

            /// problem
            double rho_T;
            double rho_ter;
            double max_vel;
            double max_acc_lon;
            double max_acc_lat;
            double max_kap;
            double min_cxi;
            double max_sig;

            /// ALM
            bool   use_scaling;
            double rho;
            double beta;
            double gamma;
            double epsilon_con;
            double max_iter;

            /// L-BFGS
            double g_epsilon;
            double min_step;
            double inner_max_iter;
            double delta;
            int    mem_size;
            int    past;
            int    int_K;

            /// debug and test
            bool in_test;
            bool in_debug;
            bool in_opt = false;

            // data
            int             piece_xy;
            int             piece_yaw;
            int             dim_T;
            double          equal_num;
            double          non_equal_num;
            double          scale_fx;
            Eigen::VectorXd lambda;
            Eigen::VectorXd mu;
            Eigen::VectorXd hx;
            Eigen::VectorXd gx;
            Eigen::VectorXd scale_cx;
            Eigen::MatrixXd init_xy;
            Eigen::MatrixXd end_xy;
            Eigen::MatrixXd init_yaw;
            Eigen::MatrixXd end_yaw;
            MINCO_SE2       minco_se2;
            KinoAstar::Ptr  front_end;
            UnevenMap::Ptr  uneven_map;

            // ros
            ros::Publisher se2_pub;
            ros::Publisher se3_pub;
            ros::Publisher debug_pub;
            ros::Subscriber odom_sub;
            ros::Subscriber wps_sub;
            Eigen::Vector3d odom_pos;

        public:
            void init(ros::NodeHandle& nh);
            void rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg);
            void rcvWpsCallBack(const geometry_msgs::PoseStamped msg);
            int optimizeSE2Traj(const Eigen::MatrixXd &initStateXY, \
                                const Eigen::MatrixXd &endStateXY , \
                                const Eigen::MatrixXd &innerPtsXY , \
                                const Eigen::VectorXd &initYaw    , \
                                const Eigen::VectorXd &endYaw     , \
                                const Eigen::VectorXd &innerPtsYaw, \
                                const double & totalTime            );
            void initScaling(Eigen::VectorXd x0);
            void calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy, \
                                      Eigen::MatrixXd& gdCyaw, Eigen::VectorXd &gdTyaw);
            void pubDebugTraj(const SE2Trajectory& traj);
            void visSE2Traj(const SE2Trajectory& traj);
            void visSE3Traj(const SE2Trajectory& traj);

            inline void setFrontend(const KinoAstar::Ptr& front_end);
            inline void setEnvironment(const UnevenMap::Ptr& env);
            inline void updateDualVars();
            inline bool judgeConvergence();
            inline double getAugmentedCost(double h_or_g, double lambda_or_mu);
            inline double getAugmentedGrad(double h_or_g, double lambda_or_mu);
            inline SE2Trajectory getTraj();
            inline vector<double> getMaxVxAxAyCurAttSig(const SE2Trajectory& traj);

            // process with T and τ
            inline double expC2(const double& tau);
            inline double logC2(const double& T);
            inline double getTtoTauGrad(const double& tau);
            inline void calTfromTau(const double& tau, Eigen::VectorXd& T);
    };

    inline void ALMTrajOpt::setFrontend(const KinoAstar::Ptr& front_end_)
    {
        this->front_end = front_end_;
    }

    inline void ALMTrajOpt::setEnvironment(const UnevenMap::Ptr& env)
    {
        this->uneven_map = env;
    }

    inline void ALMTrajOpt::updateDualVars()
    {
        lambda += rho * hx;
        for(int i = 0; i < non_equal_num; i++)
            mu(i) = std::max(mu(i)+rho*gx(i), 0.0);
        rho = std::min((1 + gamma) * rho, beta);
    }

    inline bool ALMTrajOpt::judgeConvergence()
    {
        std::cout << "reshx: "<<hx.lpNorm<Eigen::Infinity>() << " resgx: "<<gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>() << std::endl;
        
        if (std::max(hx.lpNorm<Eigen::Infinity>(), \
                     gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>()) < epsilon_con)
        {
            return true;
        }

        return false;
    }

    // get 0.5ρ(h + λ/ρ)^2 or 0.5ρ(g + μ/ρ)^2
    inline double ALMTrajOpt::getAugmentedCost(double h_or_g, double lambda_or_mu)
    {
        return h_or_g * (lambda_or_mu + 0.5*rho*h_or_g);
    }
    
    // get ρh+λ or ρg+μ, the gradient of `getAugmentedCost`
    inline double ALMTrajOpt::getAugmentedGrad(double h_or_g, double lambda_or_mu)
    {
        return rho * h_or_g + lambda_or_mu;
    }

    inline SE2Trajectory ALMTrajOpt::getTraj()
    {
        return minco_se2.getTraj();
    }

    inline vector<double> ALMTrajOpt::getMaxVxAxAyCurAttSig(const SE2Trajectory& traj)
    {
        double max_ax = 0.0;
        double max_ay = 0.0;
        double max_vx = 0.0;
        double max_cur = 0.0;
        double max_att = -1.0;
        double max_sig = 0.0;
        double vx, ax, ay, wz, cur, att;
        double inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi, sigma;
        double gravity = uneven_map->getGravity();

        Eigen::Vector3d se2_pos;
        vector<double> terrain_var;
        for(double t = 0.0; t < traj.getTotalDuration(); t += 0.01)
        {
            se2_pos = traj.getNormSE2Pos(t);
            uneven_map->getTerrainVariables(se2_pos, terrain_var);

            inv_cos_vphix = terrain_var[0];
            sin_phix = terrain_var[1];
            inv_cos_vphiy = terrain_var[2];
            sin_phiy = terrain_var[3];
            cos_xi = terrain_var[4];
            inv_cos_xi = terrain_var[5];
            sigma = terrain_var[6];

            vx = traj.getVelNorm(t) * inv_cos_vphix;
            ax = traj.getLonAcc(t) * inv_cos_vphix + gravity*sin_phix;
            ay = traj.getLatAcc(t) * inv_cos_vphiy + gravity*sin_phiy;
            wz = traj.getAngleRate(t) * inv_cos_xi;
            cur = wz / sqrt(vx*vx + delta_sigl);
            att = -1.0 / inv_cos_xi;
            if(fabs(max_ax) < fabs(ax))
            {
                max_ax = ax;
            }
            if(fabs(max_ay) < fabs(ay))
            {
                max_ay = ay;
            }
            if(fabs(max_vx) < fabs(vx))
            {
                max_vx = vx;
            }
            if(fabs(max_cur) < fabs(cur))
            {
                max_cur = cur;
            }
            if(max_att < att)
            {
                max_att = att;
            }
            if(max_sig < sigma)
            {
                max_sig = sigma;
            }
        }
        return vector<double>{max_vx, max_ax, max_ay, max_cur, max_att, max_sig};
    }

    // T = e^τ
    inline double ALMTrajOpt::expC2(const double& tau)
    {
        return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
    }

    // τ = ln(T)
    inline double ALMTrajOpt::logC2(const double& T)
    {
        return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
    }

    // get dT/dτ
    inline double ALMTrajOpt::getTtoTauGrad(const double& tau)
    {
        if (tau > 0)
            return tau + 1.0;
        else 
        {
            double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
            return (1.0 - tau) / (denSqrt * denSqrt);
        } 
    }

    // know τ
    // then get T (uniform)
    inline void ALMTrajOpt::calTfromTau(const double& tau, Eigen::VectorXd& T)
    {
        T.setConstant(expC2(tau) / T.size());
        return;
    }
}
