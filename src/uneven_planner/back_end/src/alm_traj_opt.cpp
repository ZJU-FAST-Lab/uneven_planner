#include "back_end/alm_traj_opt.h"

namespace uneven_planner
{
    void ALMTrajOpt::init(ros::NodeHandle& nh)
    {
        nh.getParam("alm_traj_opt/rho_T", rho_T);
        nh.getParam("alm_traj_opt/rho_ter", rho_ter);
        nh.getParam("alm_traj_opt/max_vel", max_vel);
        nh.getParam("alm_traj_opt/max_acc_lon", max_acc_lon);
        nh.getParam("alm_traj_opt/max_acc_lat", max_acc_lat);
        nh.getParam("alm_traj_opt/max_kap", max_kap);
        nh.getParam("alm_traj_opt/min_cxi", min_cxi);
        nh.getParam("alm_traj_opt/max_sig", max_sig);
        nh.getParam("alm_traj_opt/use_scaling", use_scaling);
        nh.getParam("alm_traj_opt/rho", rho);
        nh.getParam("alm_traj_opt/beta", beta);
        nh.getParam("alm_traj_opt/gamma", gamma);
        nh.getParam("alm_traj_opt/epsilon_con", epsilon_con);
        nh.getParam("alm_traj_opt/max_iter", max_iter);
        nh.getParam("alm_traj_opt/g_epsilon", g_epsilon);
        nh.getParam("alm_traj_opt/min_step", min_step);
        nh.getParam("alm_traj_opt/inner_max_iter", inner_max_iter);
        nh.getParam("alm_traj_opt/delta", delta);
        nh.getParam("alm_traj_opt/mem_size", mem_size);
        nh.getParam("alm_traj_opt/past", past);
        nh.getParam("alm_traj_opt/int_K", int_K);
        nh.getParam("alm_traj_opt/in_test", in_test);
        nh.getParam("alm_traj_opt/in_debug", in_debug);

        se2_pub = nh.advertise<nav_msgs::Path>("/alm/se2_path", 1);
        se3_pub = nh.advertise<nav_msgs::Path>("/alm/se3_path", 1);
        if (in_test)
        {
            wps_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &ALMTrajOpt::rcvWpsCallBack, this);
            odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &ALMTrajOpt::rcvOdomCallBack, this);
        }
        if (in_debug)
        {
            debug_pub = nh.advertise<visualization_msgs::Marker>("/alm/debug_path", 1);
        }
            debug_pub = nh.advertise<visualization_msgs::Marker>("/alm/debug_path", 1);

        return;
    }

    void ALMTrajOpt::rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg)
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

    void ALMTrajOpt::rcvWpsCallBack(const geometry_msgs::PoseStamped msg)
    {
        if (in_opt)
            return;
        
        Eigen::Vector3d end_state(msg.pose.position.x, \
                                  msg.pose.position.y, \
                                  atan2(2.0*msg.pose.orientation.z*msg.pose.orientation.w, \
                                        2.0*pow(msg.pose.orientation.w, 2)-1.0)             );
        
        std::vector<Eigen::Vector3d> init_path = front_end->plan(odom_pos, end_state);
        if (init_path.empty())
            return;

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

        init_xy.col(1) << 0.05 * cos(init_yaw(0)), 0.05 * sin(init_yaw(0));
        end_xy.col(1) << 0.05 * cos(end_yaw(0)), 0.05 * sin(end_yaw(0));
        
        double temp_len_yaw = 0.0;
        double temp_len_pos = 0.0;
        double total_len = 0.0;
        double piece_len = 0.3;
        double piece_len_yaw = piece_len / 2.0;
        std::vector<Eigen::Vector2d> inner_xy_node;
        std::vector<double> inner_yaw_node;
        for (int k=0; k<init_path.size()-1; k++)
        {
            double temp_seg = (init_path[k+1] - init_path[k]).head(2).norm();
            temp_len_yaw += temp_seg;
            temp_len_pos += temp_seg;
            total_len += temp_seg;
            if (temp_len_yaw > piece_len_yaw)
            {
                double temp_yaw = init_path[k].z() + (1.0 - (temp_len_yaw-piece_len_yaw) / temp_seg) * (init_path[k+1] - init_path[k]).z();
                inner_yaw_node.push_back(temp_yaw);
                temp_len_yaw -= piece_len_yaw;
            }
            if (temp_len_pos > piece_len)
            {
                Eigen::Vector3d temp_node = init_path[k] + (1.0 - (temp_len_pos-piece_len) / temp_seg) * (init_path[k+1] - init_path[k]);
                inner_xy_node.push_back(temp_node.head(2));
                inner_yaw_node.push_back(temp_node.z());
                temp_len_pos -= piece_len;
            }
        }
        total_time = total_len / max_vel * 1.2;
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
    
        optimizeSE2Traj(init_xy, end_xy, inner_xy, \
                        init_yaw, end_yaw, inner_yaw, total_time);
        
        // visualization
        SE2Trajectory back_end_traj = getTraj();
        visSE2Traj(back_end_traj);
        visSE3Traj(back_end_traj);
        std::vector<double> max_terrain_value = getMaxVxAxAyCurAttSig(back_end_traj);
        std::cout << "equal error: "<< back_end_traj.getNonHolError() << std::endl;
        std::cout << "max vx rate: "<< max_terrain_value[0] << std::endl;
        std::cout << "max ax rate: "<< max_terrain_value[1] << std::endl;
        std::cout << "max ay rate: "<< max_terrain_value[2] << std::endl;
        std::cout << "max cur:     "<< max_terrain_value[3] << std::endl;
        std::cout << "min cosxi:   "<< -max_terrain_value[4] << std::endl;
        std::cout << "max sigma:   "<< max_terrain_value[5] << std::endl;
        
        return;
    }

    static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
    static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                         const double fx, const double step, int k, int ls);
    int ALMTrajOpt::optimizeSE2Traj(const Eigen::MatrixXd &initStateXY, \
                                    const Eigen::MatrixXd &endStateXY , \
                                    const Eigen::MatrixXd &innerPtsXY , \
                                    const Eigen::VectorXd &initYaw    , \
                                    const Eigen::VectorXd &endYaw     , \
                                    const Eigen::VectorXd &innerPtsYaw, \
                                    const double & totalTime            )
    {
        int ret_code = 0;
        
        in_opt = true;

        piece_xy = innerPtsXY.cols() + 1;
        piece_yaw = innerPtsYaw.size() + 1;
        minco_se2.reset(piece_xy, piece_yaw);
        init_xy = initStateXY;
        end_xy = endStateXY;
        init_yaw = initYaw.transpose();
        end_yaw = endYaw.transpose();

        int variable_num = 2*(piece_xy-1) + (piece_yaw-1) + 1;
        // non-holonomic
        equal_num = piece_xy * (int_K + 1);
        // longitude velocity, longitude acceleration, latitude acceleration, curvature, attitude, surface variation
        non_equal_num = piece_xy * (int_K + 1) * 6;
        hx.resize(equal_num);
        hx.setZero();
        lambda.resize(equal_num);
        lambda.setZero();
        gx.resize(non_equal_num);
        gx.setZero();
        mu.resize(non_equal_num);
        mu.setZero();
        scale_fx = 1.0;
        scale_cx.resize(equal_num+non_equal_num);
        scale_cx.setConstant(1.0);

        // init solution
        Eigen::VectorXd x;
        x.resize(variable_num);

        dim_T = 1;
        double& tau = x(0);
        Eigen::Map<Eigen::MatrixXd> Pxy(x.data()+dim_T, 2, piece_xy-1);
        Eigen::Map<Eigen::MatrixXd> Pyaw(x.data()+dim_T+2*(piece_xy-1), 1, piece_yaw-1);

        tau = logC2(totalTime);
        Pxy = innerPtsXY;
        Pyaw = innerPtsYaw.transpose();

        // lbfgs params
        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = mem_size;
        lbfgs_params.past = past;
        lbfgs_params.g_epsilon = g_epsilon;
        lbfgs_params.min_step = min_step;
        lbfgs_params.delta = delta;
        lbfgs_params.max_iterations = inner_max_iter;
        double inner_cost;

        // begin PHR Augmented Lagrangian Method
        ros::Time start_time = ros::Time::now();
        int iter = 0;
        if (use_scaling)
            initScaling(x);

        while (true)
        {
            int result = lbfgs::lbfgs_optimize(x, inner_cost, &innerCallback, nullptr, 
                                               &earlyExit, this, lbfgs_params);

            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGS_CANCELED ||
                result == lbfgs::LBFGS_STOP || 
                result == lbfgs::LBFGSERR_MAXIMUMITERATION)
            {
                ROS_INFO_STREAM("[Inner] optimization success! cost: " << inner_cost );
            }
            else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
            {
                ROS_WARN("[Inner] The line-search routine reaches the maximum number of evaluations.");
            }
            else
            {
                ret_code = 1;
                ROS_ERROR("[Inner] Solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
                break;
            }

            updateDualVars();

            if(judgeConvergence())
            {
                ROS_WARN_STREAM("[ALM] Convergence! iters: "<<iter);
                break;
            }

            if(++iter > max_iter)
            {
                ret_code = 2;
                ROS_WARN("[ALM] Reach max iteration");
                break;
            }
        }
        ROS_INFO_STREAM("[ALM] Time consuming: "<<(ros::Time::now()-start_time).toSec() * 1000.0 << " ms");
        ROS_INFO_STREAM("[ALM] Jerk cost: "<<minco_se2.getTrajJerkCost() << " traj time:" << minco_se2.getTraj().getTotalDuration());
        
        in_opt = false;
        
        return ret_code;
    }

    static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        ALMTrajOpt& obj = *(ALMTrajOpt*)ptrObj;

        // get x
        const double& tau = x(0);
        double& grad_tau = grad(0);
        Eigen::Map<const Eigen::MatrixXd> Pxy(x.data()+obj.dim_T, 2, obj.piece_xy-1);
        Eigen::Map<const Eigen::MatrixXd> Pyaw(x.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);
        Eigen::Map<Eigen::MatrixXd> gradPxy(grad.data() + obj.dim_T, 2, obj.piece_xy-1);
        Eigen::Map<Eigen::MatrixXd> gradPyaw(grad.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);
        
        // get T from τ, generate MINCO trajectory
        Eigen::VectorXd Txy, Tyaw;
        Txy.resize(obj.piece_xy);
        Tyaw.resize(obj.piece_yaw);
        obj.calTfromTau(tau, Txy);
        obj.calTfromTau(tau, Tyaw);
        obj.minco_se2.generate(obj.init_xy, obj.end_xy, Pxy, Txy, \
                               obj.init_yaw, obj.end_yaw, Pyaw, Tyaw);
        
        // get jerk cost with grad (C,T)
        double jerk_cost = 0.0;
        Eigen::MatrixXd gdCxy_jerk;
        Eigen::VectorXd gdTxy_jerk;
        Eigen::MatrixXd gdCyaw_jerk;
        Eigen::VectorXd gdTyaw_jerk;
        obj.minco_se2.calJerkGradCT(gdCxy_jerk, gdTxy_jerk, gdCyaw_jerk, gdTyaw_jerk);
        jerk_cost = obj.minco_se2.getTrajJerkCost() * obj.scale_fx;
        if (obj.use_scaling)
            jerk_cost *= scale_trick_jerk;

        // get constrain cost with grad (C,T)
        double constrain_cost = 0.0;
        Eigen::MatrixXd gdCxy_constrain;
        Eigen::VectorXd gdTxy_constrain;
        Eigen::MatrixXd gdCyaw_constrain;
        Eigen::VectorXd gdTyaw_constrain;
        obj.calConstrainCostGrad(constrain_cost, gdCxy_constrain, gdTxy_constrain, \
                                 gdCyaw_constrain, gdTyaw_constrain);

        // get grad (q, T) from (C, T)
        if (obj.use_scaling)
        {
            gdCxy_jerk *= scale_trick_jerk;
            gdTxy_jerk *= scale_trick_jerk;
            gdCyaw_jerk *= scale_trick_jerk;
            gdTyaw_jerk *= scale_trick_jerk;
        }
        Eigen::MatrixXd gdCxy = gdCxy_jerk * obj.scale_fx + gdCxy_constrain;
        Eigen::VectorXd gdTxy = gdTxy_jerk * obj.scale_fx + gdTxy_constrain;
        Eigen::MatrixXd gdCyaw = gdCyaw_jerk * obj.scale_fx + gdCyaw_constrain;
        Eigen::VectorXd gdTyaw = gdTyaw_jerk * obj.scale_fx + gdTyaw_constrain;
        Eigen::MatrixXd gradPxy_temp;
        Eigen::MatrixXd gradPyaw_temp;
        obj.minco_se2.calGradCTtoQT(gdCxy, gdTxy, gradPxy_temp, gdCyaw, gdTyaw, gradPyaw_temp);
        gradPxy = gradPxy_temp;
        gradPyaw = gradPyaw_temp;

        // get tau cost with grad
        double tau_cost = obj.rho_T * obj.expC2(tau) * obj.scale_fx;
        double grad_Tsum = obj.rho_T * obj.scale_fx + \
                           gdTxy.sum() / obj.piece_xy + \
                           gdTyaw.sum() / obj.piece_yaw;
        grad_tau = grad_Tsum * obj.getTtoTauGrad(tau);

        return jerk_cost + constrain_cost + tau_cost;
    }

    void ALMTrajOpt::initScaling(Eigen::VectorXd x0)
    {       
        // get x
        const double& tau = x0(0);
        Eigen::Map<const Eigen::MatrixXd> Pxy(x0.data()+dim_T, 2, piece_xy-1);
        Eigen::Map<const Eigen::MatrixXd> Pyaw(x0.data()+dim_T+2*(piece_xy-1), 1, piece_yaw-1);
        
        // get T from τ, generate MINCO trajectory
        Eigen::VectorXd Txy, Tyaw;
        Txy.resize(piece_xy);
        Tyaw.resize(piece_yaw);
        calTfromTau(tau, Txy);
        calTfromTau(tau, Tyaw);
        minco_se2.generate(init_xy, end_xy, Pxy, Txy, \
                           init_yaw, end_yaw, Pyaw, Tyaw);
        
        // get jerk grad (C,T)
        Eigen::MatrixXd gdCxy_fx;
        Eigen::VectorXd gdTxy_fx;
        Eigen::MatrixXd gdCyaw_fx;
        Eigen::VectorXd gdTyaw_fx;
        minco_se2.calJerkGradCT(gdCxy_fx, gdTxy_fx, gdCyaw_fx, gdTyaw_fx);
        
        std::vector<Eigen::MatrixXd> gdCxy;
        std::vector<Eigen::VectorXd> gdTxy;
        std::vector<Eigen::MatrixXd> gdCyaw;
        std::vector<Eigen::VectorXd> gdTyaw;

        for (int i=0; i<equal_num+non_equal_num; i++)
        {
            gdCxy.push_back(Eigen::MatrixXd::Zero(6*piece_xy, 2));
            gdTxy.push_back(Eigen::VectorXd::Zero(piece_xy));
            gdCyaw.push_back(Eigen::MatrixXd::Zero(6*piece_yaw, 1));
            gdTyaw.push_back(Eigen::VectorXd::Zero(piece_yaw));
        }

        Eigen::Vector2d pos, vel, acc, jer;
        Eigen::Vector2d xb, yb;
        Eigen::Vector3d se2_pos;
        vector<double> terrain_values;
        vector<Eigen::Vector3d> terrain_grads;
        double          yaw, dyaw, d2yaw;
        double          cyaw, syaw, v_norm;
        double          lon_acc, lat_acc, curv_snorm, vx, wz, ax, ay;
        double          inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi, sigma;
        double          gravity = uneven_map->getGravity();
        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_a = Eigen::Vector2d::Zero();
        Eigen::Vector3d grad_se2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_inv_cos_vphix, grad_sin_phix, grad_inv_cos_vphiy, grad_sin_phiy, grad_cos_xi, grad_inv_cos_xi, grad_sigma;
        double          grad_yaw = 0.0;
        double          grad_dyaw = 0.0;
        double          grad_d2yaw = 0.0;
        double          grad_vx2 = 0.0;
        double          grad_wz = 0.0;
        double          grad_ax = 0.0;
        double          grad_ay = 0.0;
        Eigen::Matrix<double, 6, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy;
        Eigen::Matrix<double, 6, 1> beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw;
        double s1, s2, s3, s4, s5;
        double s1_yaw, s2_yaw, s3_yaw, s4_yaw, s5_yaw;
        double step, alpha, omega;

        int constrain_idx = 0;
        int yaw_idx = 0;
        double base_time = 0.0;
        for (int i=0; i<piece_xy; i++)
        {
            const Eigen::Matrix<double, 6, 2> &c_xy = minco_se2.pos_minco.getCoeffs().block<6, 2>(i * 6, 0);
            step = minco_se2.pos_minco.T1(i) / int_K;
            s1 = 0.0;

            for (int j=0; j<=int_K; j++)
            {
                alpha = 1.0 / int_K * j;

                // set zero
                grad_p.setZero();
                grad_v.setZero();
                grad_a.setZero();
                grad_yaw = 0.0;
                grad_dyaw = 0.0;
                grad_d2yaw = 0.0;
                grad_vx2 = 0.0;
                grad_wz = 0.0;
                grad_ax = 0.0;
                grad_ay = 0.0;
                grad_se2.setZero();

                // analyse xy
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0_xy << 1.0, s1, s2, s3, s4, s5;
                beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                pos = c_xy.transpose() * beta0_xy;
                vel = c_xy.transpose() * beta1_xy;
                acc = c_xy.transpose() * beta2_xy;
                jer = c_xy.transpose() * beta3_xy;

                // analyse yaw
                double now_time = s1 + base_time;
                yaw_idx = int((now_time) / minco_se2.yaw_minco.T1(i));
                if (yaw_idx >= piece_yaw)
                    yaw_idx = piece_yaw - 1;
                const Eigen::Matrix<double, 6, 1> &c_yaw = minco_se2.yaw_minco.getCoeffs().block<6, 1>(yaw_idx * 6, 0);
                s1_yaw = now_time - yaw_idx * minco_se2.yaw_minco.T1(i);
                s2_yaw = s1_yaw * s1_yaw;
                s3_yaw = s2_yaw * s1_yaw;
                s4_yaw = s2_yaw * s2_yaw;
                s5_yaw = s4_yaw * s1_yaw;
                beta0_yaw << 1.0, s1_yaw, s2_yaw, s3_yaw, s4_yaw, s5_yaw;
                beta1_yaw << 0.0, 1.0, 2.0 * s1_yaw, 3.0 * s2_yaw, 4.0 * s3_yaw, 5.0 * s4_yaw;
                beta2_yaw << 0.0, 0.0, 2.0, 6.0 * s1_yaw, 12.0 * s2_yaw, 20.0 * s3_yaw;
                beta3_yaw << 0.0, 0.0, 0.0, 6.0, 24.0 * s1_yaw, 60.0 * s2_yaw;
                yaw = c_yaw.transpose() * beta0_yaw;
                dyaw = c_yaw.transpose() * beta1_yaw;
                d2yaw = c_yaw.transpose() * beta2_yaw;

                // analyse complex variable
                se2_pos = Eigen::Vector3d(pos(0), pos(1), yaw);
                UnevenMap::normSO2(se2_pos(2));
                syaw = sin(yaw);
                cyaw = cos(yaw);
                v_norm = vel.norm();
                xb = Eigen::Vector2d(cyaw, syaw);
                yb = Eigen::Vector2d(-syaw, cyaw);
                lon_acc = acc.dot(xb);
                lat_acc = acc.dot(yb);

                // analyse terrain
                uneven_map->getAllWithGrad(se2_pos, terrain_values, terrain_grads);
                inv_cos_vphix = terrain_values[0];
                sin_phix = terrain_values[1];
                inv_cos_vphiy = terrain_values[2];
                sin_phiy = terrain_values[3];
                cos_xi = terrain_values[4];
                inv_cos_xi = terrain_values[5];
                sigma = terrain_values[6];

                grad_inv_cos_vphix = terrain_grads[0];
                grad_sin_phix = terrain_grads[1];
                grad_inv_cos_vphiy = terrain_grads[2];
                grad_sin_phiy = terrain_grads[3];
                grad_cos_xi = terrain_grads[4];
                grad_inv_cos_xi = terrain_grads[5];
                grad_sigma = terrain_grads[6];

                vx = v_norm * inv_cos_vphix;
                wz = dyaw * inv_cos_xi;
                ax = lon_acc * inv_cos_vphix + gravity * sin_phix;
                ay = lat_acc * inv_cos_vphiy + gravity * sin_phiy;
                curv_snorm = wz * wz / (vx*vx + delta_sigl);

                // user-defined cost: surface variation
                if (j==0 || j==int_K)
                    omega = 0.5 * rho_ter * step;
                else
                    omega = rho_ter * step;
                double user_cost = omega * sigma * sigma;
                grad_se2 = omega * grad_sigma * sigma * 2.0;
                gdTxy_fx(i) += user_cost / int_K;
                gdCxy_fx.block<6, 2>(i * 6, 0) += beta0_xy * grad_se2.head(2).transpose();
                gdTxy_fx(i) += grad_se2.head(2).dot(vel) * alpha;
                gdCyaw_fx.block<6, 1>(yaw_idx * 6, 0) += (beta0_yaw * grad_se2(2));
                gdTyaw_fx(yaw_idx) += -(grad_se2(2) * dyaw) * yaw_idx;
                gdTxy_fx(i) += (grad_se2(2) * dyaw) * (alpha+i);

                // non-holonomic
                grad_v = Eigen::Vector2d(syaw, -cyaw);
                grad_yaw = vel.dot(xb);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta1_xy * grad_v.transpose();
                gdTxy[constrain_idx](i) += grad_v.dot(acc) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;
                
                // longitude velocity
                grad_vx2 = 1.0;
                grad_v = grad_vx2 * inv_cos_vphix * inv_cos_vphix * 2.0 * vel;
                grad_se2 = grad_vx2 * v_norm * v_norm * 2.0 * inv_cos_vphix * grad_inv_cos_vphix;
                grad_p = grad_se2.head(2);
                grad_yaw = grad_se2(2);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + \
                                                               beta1_xy * grad_v.transpose());
                gdTxy[constrain_idx](i) += (grad_p.dot(vel) + \
                                            grad_v.dot(acc)) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // longitude acceleration
                grad_ax = 2.0 * ax;
                grad_a = grad_ax * inv_cos_vphix * xb;
                grad_yaw = grad_ax * inv_cos_vphix * lat_acc;
                grad_se2 = grad_ax * (gravity * grad_sin_phix + grad_inv_cos_vphix * lon_acc);
                grad_p = grad_se2.head(2);
                grad_yaw += grad_se2(2);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + \
                                                               beta2_xy * grad_a.transpose());
                gdTxy[constrain_idx](i) += (grad_p.dot(vel) + \
                                            grad_a.dot(jer) ) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // latitude acceleration
                grad_ay = 2.0 * ay;
                grad_a = grad_ay * inv_cos_vphiy * yb;
                grad_yaw = -grad_ay * inv_cos_vphiy * lon_acc;
                grad_se2 = grad_ay * (gravity * grad_sin_phiy + grad_inv_cos_vphiy * lat_acc);
                grad_p = grad_se2.head(2);
                grad_yaw += grad_se2(2);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + \
                                                               beta2_xy * grad_a.transpose());
                gdTxy[constrain_idx](i) += (grad_p.dot(vel) + \
                                            grad_a.dot(jer) ) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // curvature
                double denominator = 1.0 / (vx*vx + delta_sigl);
                grad_wz = denominator * 2.0 * wz;
                grad_vx2 = -curv_snorm * denominator;
                grad_dyaw = grad_wz * inv_cos_xi;
                grad_se2 = grad_wz * dyaw * grad_inv_cos_xi;
                grad_v = grad_vx2 * inv_cos_vphix * inv_cos_vphix * 2.0 * vel;
                grad_se2 += grad_vx2 * v_norm * v_norm * 2.0 * inv_cos_vphix * grad_inv_cos_vphix;
                grad_p = grad_se2.head(2);
                grad_yaw = grad_se2(2);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + \
                                                               beta1_xy * grad_v.transpose());
                gdTxy[constrain_idx](i) += (grad_p.dot(vel) + \
                                            grad_v.dot(acc)) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += (beta0_yaw * grad_yaw + \
                                                                      beta1_yaw * grad_dyaw);
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw +
                                                    grad_dyaw * d2yaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw +
                                            grad_dyaw * d2yaw) * (alpha+i);
                constrain_idx++;

                // attitude
                grad_se2 = -grad_cos_xi;
                grad_p = grad_se2.head(2);
                grad_yaw = grad_se2(2);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose());
                gdTxy[constrain_idx](i) += grad_p.dot(vel) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // surface variation
                grad_se2 = grad_sigma;
                grad_p = grad_se2.head(2);
                grad_yaw = grad_se2(2);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose());
                gdTxy[constrain_idx](i) += grad_p.dot(vel) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;
                
                s1 += step;
            }
            base_time += minco_se2.pos_minco.T1(i);
        }
        
        Eigen::MatrixXd gdPxy_fx;
        Eigen::MatrixXd gdPyaw_fx;
        std::vector<Eigen::MatrixXd> gdPxy;
        std::vector<Eigen::MatrixXd> gdPyaw;
        std::vector<double>          gdTau;
        minco_se2.calGradCTtoQT(gdCxy_fx, gdTxy_fx, gdPxy_fx, gdCyaw_fx, gdTyaw_fx, gdPyaw_fx);
        double grad_Tsum_fx = rho_T + \
                              gdTxy_fx.sum() / piece_xy + \
                              gdTyaw_fx.sum() / piece_yaw;
        double gdTau_fx = grad_Tsum_fx * getTtoTauGrad(tau);
        for (int i=0; i<gdCxy.size(); i++)
        {
            Eigen::MatrixXd gdPxy_temp;
            Eigen::MatrixXd gdPyaw_temp;
            minco_se2.calGradCTtoQT(gdCxy[i], gdTxy[i], gdPxy_temp, gdCyaw[i], gdTyaw[i], gdPyaw_temp);
            double grad_Tsum = gdTxy[i].sum() / piece_xy + \
                               gdTyaw[i].sum() / piece_yaw;
            gdTau.push_back(grad_Tsum * getTtoTauGrad(tau));
            gdPxy.push_back(gdPxy_temp);
            gdPyaw.push_back(gdPyaw_temp);
        }

        gdPxy_fx.resize((piece_xy-1)*2, 1);
        gdPyaw_fx.resize(piece_yaw-1, 1);
        scale_fx = 1.0 / max(1.0, max(max(gdPxy_fx.lpNorm<Eigen::Infinity>(), \
                                      gdPyaw_fx.lpNorm<Eigen::Infinity>()), fabs(gdTau_fx)));
        std::cout<<"scale_fx="<<scale_fx<<std::endl;
        for (int i=0; i<equal_num+non_equal_num; i++)
        {
            gdPxy[i].resize((piece_xy-1)*2, 1);
            gdPyaw[i].resize(piece_yaw-1, 1);
            scale_cx(i) = 1.0 / max(1.0, max(max(gdPxy[i].lpNorm<Eigen::Infinity>(), \
                                      gdPyaw[i].lpNorm<Eigen::Infinity>()), fabs(gdTau[i])));
        }
    }

    void ALMTrajOpt::calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy, \
                                          Eigen::MatrixXd& gdCyaw, Eigen::VectorXd &gdTyaw)
    {
        cost = 0.0;
        gdCxy.resize(6*piece_xy, 2);
        gdCxy.setZero();
        gdTxy.resize(piece_xy);
        gdTxy.setZero();
        gdCyaw.resize(6*piece_yaw, 1);
        gdCyaw.setZero();
        gdTyaw.resize(piece_yaw);
        gdTyaw.setZero();

        Eigen::Vector2d pos, vel, acc, jer;
        Eigen::Vector2d xb, yb;
        Eigen::Vector3d se2_pos;
        vector<double> terrain_values;
        vector<Eigen::Vector3d> terrain_grads;
        double          yaw, dyaw, d2yaw;
        double          cyaw, syaw, v_norm;
        double          lon_acc, lat_acc, curv_snorm, vx, wz, ax, ay;
        double          inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi, sigma;
        double          gravity = uneven_map->getGravity();
        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_a = Eigen::Vector2d::Zero();
        Eigen::Vector3d grad_se2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_inv_cos_vphix, grad_sin_phix, grad_inv_cos_vphiy, grad_sin_phiy, grad_cos_xi, grad_inv_cos_xi, grad_sigma;
        double          grad_yaw = 0.0;
        double          grad_dyaw = 0.0;
        double          grad_d2yaw = 0.0;
        double          aug_grad = 0.0;
        double          grad_vx2 = 0.0;
        double          grad_wz = 0.0;
        double          grad_ax = 0.0;
        double          grad_ay = 0.0;
        Eigen::Matrix<double, 6, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy;
        Eigen::Matrix<double, 6, 1> beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw;
        double s1, s2, s3, s4, s5;
        double s1_yaw, s2_yaw, s3_yaw, s4_yaw, s5_yaw;
        double step, alpha, omega;

        int equal_idx = 0;
        int non_equal_idx = 0;
        int constrain_idx = 0;
        int yaw_idx = 0;
        double base_time = 0.0;
        for (int i=0; i<piece_xy; i++)
        {
            const Eigen::Matrix<double, 6, 2> &c_xy = minco_se2.pos_minco.getCoeffs().block<6, 2>(i * 6, 0);
            step = minco_se2.pos_minco.T1(i) / int_K;
            s1 = 0.0;

            for (int j=0; j<=int_K; j++)
            {
                alpha = 1.0 / int_K * j;

                // set zero
                grad_p.setZero();
                grad_v.setZero();
                grad_a.setZero();
                grad_yaw = 0.0;
                grad_dyaw = 0.0;
                grad_d2yaw = 0.0;
                grad_vx2 = 0.0;
                grad_wz = 0.0;
                grad_ax = 0.0;
                grad_ay = 0.0;
                grad_se2.setZero();

                // analyse xy
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0_xy << 1.0, s1, s2, s3, s4, s5;
                beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                pos = c_xy.transpose() * beta0_xy;
                vel = c_xy.transpose() * beta1_xy;
                acc = c_xy.transpose() * beta2_xy;
                jer = c_xy.transpose() * beta3_xy;

                // analyse yaw
                double now_time = s1 + base_time;
                yaw_idx = int((now_time) / minco_se2.yaw_minco.T1(i));
                if (yaw_idx >= piece_yaw)
                    yaw_idx = piece_yaw - 1;
                const Eigen::Matrix<double, 6, 1> &c_yaw = minco_se2.yaw_minco.getCoeffs().block<6, 1>(yaw_idx * 6, 0);
                s1_yaw = now_time - yaw_idx * minco_se2.yaw_minco.T1(i);
                s2_yaw = s1_yaw * s1_yaw;
                s3_yaw = s2_yaw * s1_yaw;
                s4_yaw = s2_yaw * s2_yaw;
                s5_yaw = s4_yaw * s1_yaw;
                beta0_yaw << 1.0, s1_yaw, s2_yaw, s3_yaw, s4_yaw, s5_yaw;
                beta1_yaw << 0.0, 1.0, 2.0 * s1_yaw, 3.0 * s2_yaw, 4.0 * s3_yaw, 5.0 * s4_yaw;
                beta2_yaw << 0.0, 0.0, 2.0, 6.0 * s1_yaw, 12.0 * s2_yaw, 20.0 * s3_yaw;
                beta3_yaw << 0.0, 0.0, 0.0, 6.0, 24.0 * s1_yaw, 60.0 * s2_yaw;
                yaw = c_yaw.transpose() * beta0_yaw;
                dyaw = c_yaw.transpose() * beta1_yaw;
                d2yaw = c_yaw.transpose() * beta2_yaw;

                // analyse complex variable
                se2_pos = Eigen::Vector3d(pos(0), pos(1), yaw);
                UnevenMap::normSO2(se2_pos(2));
                syaw = sin(yaw);
                cyaw = cos(yaw);
                v_norm = vel.norm();
                xb = Eigen::Vector2d(cyaw, syaw);
                yb = Eigen::Vector2d(-syaw, cyaw);
                lon_acc = acc.dot(xb);
                lat_acc = acc.dot(yb);

                // analyse terrain
                uneven_map->getAllWithGrad(se2_pos, terrain_values, terrain_grads);
                inv_cos_vphix = terrain_values[0];
                sin_phix = terrain_values[1];
                inv_cos_vphiy = terrain_values[2];
                sin_phiy = terrain_values[3];
                cos_xi = terrain_values[4];
                inv_cos_xi = terrain_values[5];
                sigma = terrain_values[6];

                // debug
                // inv_cos_vphix = 1.0;
                // sin_phix = 0.0;
                // inv_cos_vphiy = 1.0;
                // sin_phiy = 0.0;
                // cos_xi = 1.0;
                // inv_cos_xi = 1.0;
                // sigma = 0.0;
                // // terrain_grads[0].setZero();
                // // terrain_grads[1].setZero();
                // // terrain_grads[2].setZero();
                // // terrain_grads[3].setZero();
                // // terrain_grads[4].setZero();
                // // terrain_grads[5].setZero();
                // // terrain_grads[6].setZero();
                // for (size_t i=0; i<terrain_grads.size(); i++)
                //     terrain_grads[i].setZero();

                grad_inv_cos_vphix = terrain_grads[0];
                grad_sin_phix = terrain_grads[1];
                grad_inv_cos_vphiy = terrain_grads[2];
                grad_sin_phiy = terrain_grads[3];
                grad_cos_xi = terrain_grads[4];
                grad_inv_cos_xi = terrain_grads[5];
                grad_sigma = terrain_grads[6];

                vx = v_norm * inv_cos_vphix;
                wz = dyaw * inv_cos_xi;
                ax = lon_acc * inv_cos_vphix + gravity * sin_phix;
                ay = lat_acc * inv_cos_vphiy + gravity * sin_phiy;
                curv_snorm = wz * wz / (vx*vx + delta_sigl);

                // user-defined cost: surface variation
                if (j==0 || j==int_K)
                    omega = 0.5 * rho_ter * step * scale_fx;
                else
                    omega = rho_ter * step * scale_fx;
                double user_cost = omega * sigma * sigma;
                cost += user_cost;
                grad_se2 += omega * grad_sigma * sigma * 2.0;
                gdTxy(i) += user_cost / int_K;

                // non-holonomic
                double nonh_lambda = lambda[equal_idx];
                Eigen::Vector2d non_holonomic_yaw(syaw, -cyaw);
                hx[equal_idx] = vel.dot(non_holonomic_yaw) * scale_cx(constrain_idx);
                cost += getAugmentedCost(hx[equal_idx], nonh_lambda);
                double nonh_grad = getAugmentedGrad(hx[equal_idx], nonh_lambda) * scale_cx(constrain_idx);
                grad_v += nonh_grad * non_holonomic_yaw;
                grad_yaw += nonh_grad * vel.dot(xb);
                equal_idx++;
                constrain_idx++;
                
                // longitude velocity
                double v_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (vx*vx - max_vel*max_vel) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + v_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], v_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], v_mu) * scale_cx(constrain_idx);
                    grad_vx2 += aug_grad;
                }
                else
                {
                    cost += -0.5 * v_mu * v_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // longitude acceleration
                double lona_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (ax*ax - max_acc_lon*max_acc_lon) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + lona_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], lona_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], lona_mu) * scale_cx(constrain_idx);
                    grad_ax += aug_grad * 2.0 * ax;
                }
                else
                {
                    cost += -0.5 * lona_mu * lona_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // latitude acceleration
                double lata_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (ay*ay - max_acc_lat*max_acc_lat) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + lata_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], lata_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], lata_mu) * scale_cx(constrain_idx);
                    grad_ay += aug_grad * 2.0 * ay;
                }
                else
                {
                    cost += -0.5 * lata_mu * lata_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // curvature
                double curv_mu = mu[non_equal_idx];
                if (use_scaling)
                    gx[non_equal_idx] = (curv_snorm - max_kap*max_kap) * scale_cx(constrain_idx);
                else
                    gx[non_equal_idx] = (curv_snorm - max_kap*max_kap) * cur_scale;
                if (rho * gx[non_equal_idx] + curv_mu > 0)
                {
                    double denominator = 1.0 / (vx*vx + delta_sigl);
                    cost += getAugmentedCost(gx[non_equal_idx], curv_mu);
                    if (use_scaling)
                        aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * scale_cx(constrain_idx);
                    else
                        aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * cur_scale;
                    grad_wz += aug_grad * denominator * 2.0 * wz;
                    grad_vx2 -= aug_grad * curv_snorm * denominator;
                }
                else
                {
                    cost += -0.5 * curv_mu * curv_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // attitude
                double att_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (min_cxi - cos_xi) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + att_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], att_mu);
                    grad_se2 -= getAugmentedGrad(gx[non_equal_idx], att_mu) * grad_cos_xi * scale_cx(constrain_idx);
                }
                else
                {
                    cost += -0.5 * att_mu * att_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // surface variation
                double sig_mu = mu[non_equal_idx];
                if (use_scaling)
                    gx[non_equal_idx] = (sigma - max_sig) * scale_cx(constrain_idx);
                else
                    gx[non_equal_idx] = (sigma - max_sig) * sig_scale;
                if (rho * gx[non_equal_idx] + sig_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], sig_mu);
                    if (use_scaling)
                        grad_se2 += getAugmentedGrad(gx[non_equal_idx], sig_mu) * grad_sigma * scale_cx(constrain_idx);
                    else
                        grad_se2 += getAugmentedGrad(gx[non_equal_idx], sig_mu) * grad_sigma * sig_scale;
                }
                else
                {
                    cost += -0.5 * sig_mu * sig_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // process with vx, wz, ax
                grad_v += grad_vx2 * inv_cos_vphix * inv_cos_vphix * 2.0 * vel;
                grad_se2 += grad_vx2 * v_norm * v_norm * 2.0 * inv_cos_vphix * grad_inv_cos_vphix;
                
                grad_dyaw += grad_wz * inv_cos_xi;
                grad_se2 += grad_wz * dyaw * grad_inv_cos_xi;

                grad_a += grad_ax * inv_cos_vphix * xb;
                grad_yaw += grad_ax * inv_cos_vphix * lat_acc;
                grad_se2 += grad_ax * (gravity * grad_sin_phix + grad_inv_cos_vphix * lon_acc);

                grad_a += grad_ay * inv_cos_vphiy * yb;
                grad_yaw -= grad_ay * inv_cos_vphiy * lon_acc;
                grad_se2 += grad_ay * (gravity * grad_sin_phiy + grad_inv_cos_vphiy * lat_acc);

                grad_p += grad_se2.head(2);
                grad_yaw += grad_se2(2);

                // add all grad into C,T
                // note that xy = Cxy*β(j/K*T_xy), yaw = Cyaw*β(i*T_xy+j/K*T_xy-yaw_idx*T_yaw)
                // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
                gdCxy.block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + \
                                                beta1_xy * grad_v.transpose() + \
                                                beta2_xy * grad_a.transpose());
                // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
                gdTxy(i) += (grad_p.dot(vel) + \
                             grad_v.dot(acc) + \
                             grad_a.dot(jer) ) * alpha;
                // ∂yaw/∂Cyaw, ∂dyaw/∂Cyaw, ∂d2yaw/∂Cyaw
                gdCyaw.block<6, 1>(yaw_idx * 6, 0) += (beta0_yaw * grad_yaw + \
                                                       beta1_yaw * grad_dyaw + \
                                                       beta2_yaw * grad_d2yaw);
                // ∂yaw/∂Tyaw, ∂dyaw/∂Tyaw, ∂d2yaw/∂Tyaw
                gdTyaw(yaw_idx) += -(grad_yaw * dyaw +
                                     grad_dyaw * d2yaw) * yaw_idx;
                // ∂yaw/∂Txy, ∂dyaw/∂Txy, ∂d2yaw/∂Txy
                gdTxy(i) += (grad_yaw * dyaw +
                             grad_dyaw * d2yaw) * (alpha+i);
                
                s1 += step;
            }
            base_time += minco_se2.pos_minco.T1(i);
        }
    }

    static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                         const double fx, const double step, int k, int ls)
    {
        ALMTrajOpt& obj = *(ALMTrajOpt*)ptrObj;
        if (obj.in_debug)
        {
            const double& tau = x(0);
            Eigen::Map<const Eigen::MatrixXd> Pxy(x.data()+obj.dim_T, 2, obj.piece_xy-1);
            Eigen::Map<const Eigen::MatrixXd> Pyaw(x.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);
            
            // get T from τ, generate MINCO trajectory
            Eigen::VectorXd Txy, Tyaw;
            Txy.resize(obj.piece_xy);
            Tyaw.resize(obj.piece_yaw);
            obj.calTfromTau(tau, Txy);
            obj.calTfromTau(tau, Tyaw);
            obj.minco_se2.generate(obj.init_xy, obj.end_xy, Pxy, Txy, \
                                obj.init_yaw, obj.end_yaw, Pyaw, Tyaw);
            auto traj = obj.minco_se2.getTraj();
            obj.pubDebugTraj(traj);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return k > 1e3;
    }

    void ALMTrajOpt::pubDebugTraj(const SE2Trajectory& traj)
    {
        int id = 0;
        double scale = 0.03;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;
        id++;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;

        double dur = traj.pos_traj.getTotalDuration();
        for (double i = 0; i < dur - 1e-4; i+=0.02)
        {
            Eigen::Vector2d dur_p = traj.pos_traj.getValue(i);
            pt.x = dur_p(0);
            pt.y = dur_p(1);
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        Eigen::VectorXd ts = traj.pos_traj.getDurations();
        double time = 0.0;
        for (int i=0; i<ts.size(); i++)
        {
            time += ts(i);
            Eigen::Vector2d dur_p = traj.pos_traj.getValue(time);
            pt.x = dur_p(0);
            pt.y = dur_p(1);
            pt.z = 0.0;
            sphere.points.push_back(pt); 
        }
        debug_pub.publish(sphere);
        debug_pub.publish(line_strip);
    }

    void ALMTrajOpt::visSE2Traj(const SE2Trajectory& traj)
    {
        nav_msgs::Path back_end_path;
        back_end_path.header.frame_id = "world";
        back_end_path.header.stamp = ros::Time::now();
        
        geometry_msgs::PoseStamped p;
        for (double t=0.0; t<traj.getTotalDuration(); t+=0.03)
        {
            Eigen::Vector2d pos = traj.getPos(t);
            double yaw = traj.getAngle(t);
            p.pose.position.x = pos(0);
            p.pose.position.y = pos(1);
            p.pose.position.z = 0.0;
            p.pose.orientation.w = cos(yaw/2.0);
            p.pose.orientation.x = 0.0;
            p.pose.orientation.y = 0.0;
            p.pose.orientation.z = sin(yaw/2.0);
            back_end_path.poses.push_back(p);
        }
        Eigen::Vector2d pos = traj.getPos(traj.getTotalDuration());
        double yaw = traj.getAngle(traj.getTotalDuration());
        p.pose.position.x = pos(0);
        p.pose.position.y = pos(1);
        p.pose.position.z = 0.0;
        p.pose.orientation.w = cos(yaw/2.0);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = sin(yaw/2.0);
        back_end_path.poses.push_back(p);

        se2_pub.publish(back_end_path);
    }

    void ALMTrajOpt::visSE3Traj(const SE2Trajectory& traj)
    {
        nav_msgs::Path back_end_path;
        back_end_path.header.frame_id = "world";
        back_end_path.header.stamp = ros::Time::now();
        
        geometry_msgs::PoseStamped p;
        for (double t=0.0; t<traj.getTotalDuration(); t+=0.03)
        {
            Eigen::Vector3d pos = traj.getNormSE2Pos(t);
            Eigen::Vector3d pos_3d;
            Eigen::Matrix3d R;
            uneven_map->getTerrainPos(pos, R, pos_3d);
            p.pose.position.x = pos_3d(0);
            p.pose.position.y = pos_3d(1);
            p.pose.position.z = pos_3d(2);
            Eigen::Quaterniond q(R);
            p.pose.orientation.w = q.w();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            back_end_path.poses.push_back(p);
        }
        Eigen::Vector3d pos = traj.getNormSE2Pos(traj.getTotalDuration());
        Eigen::Vector3d pos_3d;
        Eigen::Matrix3d R;
        uneven_map->getTerrainPos(pos, R, pos_3d);
        p.pose.position.x = pos_3d(0);
        p.pose.position.y = pos_3d(1);
        p.pose.position.z = pos_3d(2);
        Eigen::Quaterniond q(R);
        p.pose.orientation.w = q.w();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        back_end_path.poses.push_back(p);

        se3_pub.publish(back_end_path);
    }
}