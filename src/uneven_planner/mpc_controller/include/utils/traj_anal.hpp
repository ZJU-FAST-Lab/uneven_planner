#pragma once

#include "mpc_controller/SE2Traj.h"
#include "utils/minco_traj.hpp"
#include <ros/ros.h>

#define BK_UNEVEN 0
#define BK_TOWARDS 1
#define BK_PUTN 2

template <typename T>
T lineTerp(T &x0, const double t0, T &x1, const double t1, const double t)
{
  if (std::abs(t1 - t0) <= 1.0e-6) 
  {
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  return x0 + (x1 - x0) * r;
}

class TrajPoint
{
public:
    double x = 0;
    double y = 0;
    double theta = 0;
    TrajPoint() {}
    TrajPoint(double x_, double y_, double theta_): x(x_), y(y_), theta(theta_) {}
    TrajPoint operator+(const TrajPoint& a)
    {
      return TrajPoint(a.x+x, a.y+y, a.theta+theta);
    }
    TrajPoint operator-(const TrajPoint& a)
    {
      return TrajPoint(x-a.x, y-a.y, theta-a.theta);
    }
    TrajPoint operator*(const double& a)
    {
      return TrajPoint(a*x, a*y, a*theta);
    }
    TrajPoint operator/(const double& a)
    {
      return TrajPoint(x/a, y/a, theta/a);
    }
};

class TrajAnalyzer{
private:
    mpc_utils::SE2Traj minco_traj;
    mpc_utils::MINCO_SE2 minco_anal;
    ros::Time start_time;
    double traj_duration;

public:
    bool at_goal = false;

    // for benchmark
    int traj_type = BK_UNEVEN;
    int traj_seg = 0;
    int traj_num = 0;
    std::vector<mpc_utils::SE2Traj> uneven_trajs;

    // for towards
    mpc_utils::Trajectory<3> towards_traj;
    std::vector<mpc_utils::Trajectory<3>> towards_trajs;
    
    // for putn
    double putn_dt = 0.1;
    std::vector<TrajPoint> putn_traj;
    std::vector<std::vector<TrajPoint>> putn_trajs;

    TrajAnalyzer() {}

    int CountLines(std::string filename)
    {
        std::ifstream ReadFile;
        int n=0;
        std::string tmp;
        ReadFile.open(filename.c_str(), std::ios::in);
        if(ReadFile.fail())
        {
          ROS_ERROR("read file error!");
          return 0;
        }
        else
        {
          while(getline(ReadFile, tmp))
          {
            n++;
          }
          ReadFile.close();
          return n;
        }
    }
 
    std::string ReadLine(std::string filename, int line)
    {
      int lines, i=0;
      std::string temp;
      std::fstream file;
      file.open(filename.c_str(), std::ios::in);
      lines = CountLines(filename);
  
      if(line<=0)
      {
        return "Error: line wrong, it should not be zero or negative.";
      }
      if(file.fail())
      {
        return "Error: file is not exist.";
      }
      if(line > lines)
      {
        return "Error: line is over max length.";
      }
      while(getline(file, temp) && i < line-1)
      {
        i++;
      }
      file.close();
      return temp;
    }

    void setTraj(mpc_controller::SE2TrajConstPtr msg)
    {
      start_time = msg->start_time;

      Eigen::MatrixXd posP(2, msg->pos_pts.size() - 2);
      Eigen::MatrixXd angleP(1, msg->angle_pts.size() - 2);
      Eigen::VectorXd posT(msg->posT_pts.size());
      Eigen::VectorXd angleT(msg->angleT_pts.size());
      Eigen::MatrixXd initS, tailS;

      for (int i = 1; i < (int)msg->pos_pts.size() - 1; i++)
      {
        posP(0, i - 1) = msg->pos_pts[i].x;
        posP(1, i - 1) = msg->pos_pts[i].y;
      }

      for (int i = 1; i < (int)msg->angle_pts.size() - 1; i++)
      {
        angleP(0, i - 1) = msg->angle_pts[i].x;
      }

      for (int i = 0; i < (int)msg->posT_pts.size(); i++)
      {
        posT(i) = msg->posT_pts[i];
      }

      for (int i = 0; i < (int)msg->angleT_pts.size(); i++)
      {
        angleT(i) = msg->angleT_pts[i];
      }

      initS.setZero(2, 3);
      tailS.setZero(2, 3);
      initS.col(0) = Eigen::Vector2d(msg->pos_pts[0].x, msg->pos_pts[0].y);
      initS.col(1) = Eigen::Vector2d(msg->init_v.x, msg->init_v.y);
      initS.col(2) = Eigen::Vector2d(msg->init_a.x, msg->init_a.y);
      tailS.col(0) = Eigen::Vector2d(msg->pos_pts.back().x, msg->pos_pts.back().y);
      tailS.col(1) = Eigen::Vector2d::Zero();
      tailS.col(2) = Eigen::Vector2d::Zero();
      minco_anal.pos_anal.reset(initS, msg->pos_pts.size() - 1);
      minco_anal.pos_anal.generate(posP, tailS, posT);
      minco_traj.pos_traj = minco_anal.pos_anal.getTraj();

      initS.setZero(1, 3);
      tailS.setZero(1, 3);
      initS(0, 0) = msg->angle_pts[0].x;
      initS(0, 1) = msg->init_v.z;
      initS(0, 2) = msg->init_a.z;
      tailS(0, 0) = msg->angle_pts.back().x;
      tailS(0, 1) = 0.0;
      tailS(0, 1) = 0.0;
      minco_anal.angle_anal.reset(initS, msg->angle_pts.size() - 1);
      minco_anal.angle_anal.generate(angleP, tailS, angleT);
      minco_traj.angle_traj = minco_anal.angle_anal.getTraj();

      traj_duration = minco_traj.pos_traj.getTotalDuration();
    }

    void setTraj(std::string file)
    {
      if (file.find("proposed")!=std::string::npos)
      {
        traj_type = BK_UNEVEN;
        uneven_trajs.clear();

        std::string data;
        std::vector<std::string> datas;
        std::string data_line;
        
        traj_num = CountLines(file);
        for (int i=1; i<=traj_num; i++)
        {
          data_line = ReadLine(file, i);
          std::stringstream line2(data_line);
          datas.clear();
          while(line2 >> data) 
          {
            datas.push_back(data);
          }

          int pos_piece_num  = std::stoi(datas[0]);
          int angle_piece_num  = std::stoi(datas[1]);
          int index = 2;
          mpc_utils::SE2Traj traj; 

          Eigen::MatrixXd posP(2, pos_piece_num - 1);
          Eigen::MatrixXd angleP(1, angle_piece_num - 1);
          Eigen::VectorXd posT(pos_piece_num);
          Eigen::VectorXd angleT(angle_piece_num);
          Eigen::MatrixXd initS, tailS;
          Eigen::MatrixXd ainitS, atailS;

          initS.setZero(2, 3);
          tailS.setZero(2, 3);
          ainitS.setZero(1, 3);
          atailS.setZero(1, 3);

          initS.col(0).x() = std::stod(datas[index++]);
          initS.col(0).y() = std::stod(datas[index++]);
          for (int i = 0; i < pos_piece_num - 1; i++)
          {
            posP(0, i) = std::stod(datas[index++]);
            posP(1, i) = std::stod(datas[index++]);
          }
          tailS.col(0).x() = std::stod(datas[index++]);
          tailS.col(0).y() = std::stod(datas[index++]);

          ainitS(0, 0) = std::stod(datas[index++]);
          for (int i = 0; i < angle_piece_num - 1; i++)
          {
            angleP(0, i) = std::stod(datas[index++]);
          }
          atailS(0, 0) = std::stod(datas[index++]);

          for (int i = 0; i < pos_piece_num; i++)
          {
            posT(i) = std::stod(datas[index++]);
          }
          for (int i = 0; i < angle_piece_num; i++)
          {
            angleT(i) =std::stod(datas[index++]);
          }

          minco_anal.pos_anal.reset(initS, pos_piece_num);
          minco_anal.pos_anal.generate(posP, tailS, posT);
          traj.pos_traj = minco_anal.pos_anal.getTraj();
          minco_anal.angle_anal.reset(ainitS, angle_piece_num);
          minco_anal.angle_anal.generate(angleP, atailS, angleT);
          traj.angle_traj = minco_anal.angle_anal.getTraj();

          uneven_trajs.push_back(traj);
        }

        ROS_WARN("have gotten proposed traj num: %d, trigger ready.", traj_num);
      }
      else if(file.find("Wangs")!=std::string::npos)
      {
        traj_type = BK_TOWARDS;
        towards_trajs.clear();

        std::string data;
        std::vector<std::string> datas;
        std::string data_line;
        std::vector<double> T;
        std::vector<Eigen::Matrix<double, 3, 6>> cMats;
        Eigen::Matrix<double, 3, 6> cMat;
        
        traj_num = CountLines(file);
        for (int i=1; i<=traj_num; i++)
        {
          data_line = ReadLine(file, i);
          std::stringstream line2(data_line);
          datas.clear();
          while(line2 >> data) 
          {
            datas.push_back(data);
          }

          int traj_pieces  = std::stoi(datas[0]);
          int index = 1;
          T.clear();
          cMats.clear();

          for(int i = 0; i < traj_pieces; i++) 
          {
            T.push_back(std::stod(datas[index++]));
            for( int a = 0 ; a < 3 ; a++ )
            {
              for(int b = 0 ; b < 6 ; b++)
              {
                cMat(a, b) = std::stod(datas[index++]);
              }
            }
            cMats.push_back(cMat);
          }

          mpc_utils::Trajectory<3> traj;
          traj.reserve(traj_pieces);

          for (int i = 0; i < traj_pieces; i++) 
          {
            traj.emplace_back(T[i], cMats[i] );
          }

          towards_trajs.push_back(traj);
        }

        ROS_WARN("have gotten Wangs traj num: %d, trigger ready.", traj_num);
      }
      else if(file.find("Jians")!=std::string::npos)
      {
        traj_type = BK_PUTN;

        putn_trajs.clear();

        std::string data;
        std::vector<std::string> datas;
        std::string data_line;
        
        traj_num = CountLines(file);
        for (int i=1; i<=traj_num; i++)
        {
          data_line = ReadLine(file, i);
          std::stringstream line2(data_line);
          datas.clear();
          while(line2 >> data) 
          {
            datas.push_back(data);
          }

          int wps_num = std::stoi(datas[0]);
          int index = 2;
          std::vector<TrajPoint> traj;

          putn_dt = std::stod(datas[1]);
          for(int i = 0; i < wps_num; i++) 
          {
            TrajPoint point;
            point.x = std::stod(datas[index++]);
            point.y = std::stod(datas[index++]);
            point.theta = std::stod(datas[index++]);
            traj.push_back(point);
          }

          putn_trajs.push_back(traj);
        }

        ROS_WARN("have gotten Jians traj num: %d, trigger ready.", traj_num);
      }
    }

    Eigen::Vector3d getNextInitPose()
    {
      Eigen::Vector3d initp(Eigen::Vector3d::Zero());

      if (traj_seg < traj_num)
      {
        if (traj_type == BK_UNEVEN)
        {
          initp.head(2) = uneven_trajs[traj_seg].pos_traj.getPos(0);
          initp.z() = uneven_trajs[traj_seg].angle_traj.getPos(0)[0];
        }
        else if (traj_type == BK_TOWARDS)
        {
          Eigen::Vector3d pos1 = towards_trajs[traj_seg].getPos(0);
          Eigen::Vector3d pos2 = towards_trajs[traj_seg].getPos(0.1);

          initp =  Eigen::Vector3d(pos1.x(), pos1.y(), \
                                   atan2(pos2.y()-pos1.y(), pos2.x()-pos1.x()));
        }
        else if (traj_type == BK_PUTN)
        {
          TrajPoint traj_pinit = putn_trajs[traj_seg][0];
          initp = Eigen::Vector3d(traj_pinit.x, traj_pinit.y, traj_pinit.theta);
        }
      }

      return initp;
    }

    void beginNextTraj()
    {
      if (traj_seg < traj_num)
      {
        if (traj_type == BK_UNEVEN)
        {
          minco_traj = uneven_trajs[traj_seg++];
          traj_duration = minco_traj.pos_traj.getTotalDuration();
        }
        else if (traj_type == BK_TOWARDS)
        {
          towards_traj = towards_trajs[traj_seg++];
          traj_duration = towards_traj.getTotalDuration();
        }
        else if (traj_type == BK_PUTN)
        {
          putn_traj = putn_trajs[traj_seg++];
          traj_duration = putn_dt * putn_traj.size();
        }

        start_time = ros::Time::now();
        at_goal = false;
      }
    }

    std::vector<TrajPoint> getTajWps(double dt)
    {
      std::vector<TrajPoint> P;
      TrajPoint tp;
      
      if (traj_type == BK_UNEVEN)
      {
        for (double t=0.0; t<=traj_duration; t+=dt)
        {
          Eigen::Vector2d po = minco_traj.pos_traj.getPos(t);
          tp.x = po[0];
          tp.y = po[1];
          tp.theta = minco_traj.angle_traj.getPos(t)[0];
          P.push_back(tp);
        }
      }
      else if (traj_type == BK_TOWARDS)
      {
        for (double t=dt+1e-3; t<=traj_duration; t+=dt)
        {
          Eigen::Vector3d po = towards_traj.getPos(t);
          Eigen::Vector3d pr = towards_traj.getPos(t-dt);

          tp.x = po[0];
          tp.y = po[1];
          tp.theta = std::atan2(po[1]-pr[1], po[0]-pr[0]);
          P.push_back(tp);
        }
      }
      else if (traj_type == BK_PUTN)
      {
        P = putn_traj;
      }
      
      return P;
    }

    void setTestTraj(double max_vel)
    {
      Eigen::MatrixXd eight(10, 3);
      eight << 2.34191,      -0.382897,  0.127306, 
              3.09871,      0.936706,   1.72168,
              1.99125,      2.68782,    2.53673,
              0.394621,     3.877,      2.25751,
              -0.0799935,   5.86051,    1.13753,
              1.90338,      6.56037,    -0.143977,
              3.17197,      5.21122,    -1.66021,
              2.12699,      3.42104,    -2.48622,
              0.48492,      2.23846,    -2.31336,
              -0.00904252,  0.365258,   -1.55525;
      Eigen::MatrixXd posP = eight.transpose();
      Eigen::VectorXd T(11);
      T << 3.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.0;
      T *= 1.5 / max_vel;
      Eigen::MatrixXd initS, tailS;

      initS.setZero(2, 3);
      tailS.setZero(2, 3);
      minco_anal.pos_anal.reset(initS, T.size());
      minco_anal.pos_anal.generate(posP.block<2, 10>(0, 0), tailS, T);
      minco_traj.pos_traj = minco_anal.pos_anal.getTraj();

      initS.setZero(1, 3);
      tailS.setZero(1, 3);
      initS(0, 0) = -0.257661;
      tailS(0, 0) = -1.54148 ;

      minco_anal.angle_anal.reset(initS, T.size());
      minco_anal.angle_anal.generate(posP.block<1, 10>(2, 0), tailS, T);
      minco_traj.angle_traj = minco_anal.angle_anal.getTraj();

      traj_duration = minco_traj.pos_traj.getTotalDuration();
      start_time = ros::Time::now();
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
      int j=0;

      if (t_cur > traj_duration + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }
      
      if (traj_type == BK_UNEVEN)
      {
        for (double t=t_cur+dt; j<T; j++, t+=dt)
        {
          double temp = t;
          if (temp <= traj_duration)
          {
            Eigen::Vector2d po = minco_traj.pos_traj.getPos(temp);
            tp.x = po[0];
            tp.y = po[1];
            tp.theta = minco_traj.angle_traj.getPos(temp)[0];
            P.push_back(tp);
          }
          else
          {
            Eigen::Vector2d po = minco_traj.pos_traj.getPos(traj_duration);
            tp.x = po[0];
            tp.y = po[1];
            tp.theta = minco_traj.angle_traj.getPos(traj_duration)[0];
            P.push_back(tp);
          }
        }
      }
      else if (traj_type == BK_TOWARDS)
      {
        for (double t=t_cur+dt; j<T; j++, t+=dt)
        {
          double temp = t;
          if (temp <= traj_duration)
          {
            Eigen::Vector3d po = towards_traj.getPos(temp);
            Eigen::Vector3d pr = towards_traj.getPos(temp-dt);

            tp.x = po[0];
            tp.y = po[1];
            tp.theta = std::atan2(po[1]-pr[1], po[0]-pr[0]);
            P.push_back(tp);
          }
          else
          {
            Eigen::Vector3d po = towards_traj.getPos(traj_duration);
            Eigen::Vector3d pr = towards_traj.getPos(traj_duration-dt);

            tp.x = po[0];
            tp.y = po[1];
            tp.theta = std::atan2(po[1]-pr[1], po[0]-pr[0]);
            P.push_back(tp);
          }
        }
      }
      else if (traj_type == BK_PUTN)
      {
        for (double t=t_cur+dt; j<T; j++, t+=dt)
        {
          int index = (int)floor(t / putn_dt);
          if (index > putn_traj.size() - 2)
          {
            tp = putn_traj.back();
            P.push_back(tp);
          }
          else
          {
            tp = lineTerp(putn_traj[index], 0.0, putn_traj[index+1], putn_dt, t-index*putn_dt);
            P.push_back(tp);
          }
        }
      }

      return P;
    }
   
    ~TrajAnalyzer() {}
};