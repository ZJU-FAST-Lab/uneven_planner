#ifndef SE2POLYTRAJ_HPP
#define SE2POLYTRAJ_HPP

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <Eigen/Eigen>

#include "utils/root_finder.hpp"
#include "utils/banded_system.hpp"

// MINCO with s = 3
namespace uneven_planner
{
    constexpr double PI = 3.1415926;
    constexpr double delta_v = 0.01;
    
    // row vector with value: c0, c1, c2, ...
    template<int D>
    using CoefficientMat = Eigen::Matrix<double, D, 6>;

    template<int D>
    using DotCoefficientMat = Eigen::Matrix<double, D, 5>;

    template<int D>
    using DDotCoefficientMat = Eigen::Matrix<double, D, 4>;
    
    template <int Dim> 
    class Piece
    {
    private:
        double duration;
        CoefficientMat<Dim> coeffMat;
        int order = 5;

    public:
        Piece() = default;

        Piece(double dur, const CoefficientMat<Dim> &cMat)
            : duration(dur), coeffMat(cMat) {}
        
        inline int getDim() const
        {
            return Dim;
        }
        
        inline int getOrder() const
        {
            return order;
        }

        inline double getDuration() const
        {
            return duration;
        }

        inline const CoefficientMat<Dim> &getCoeffMat() const
        {
            return coeffMat;
        }

        inline CoefficientMat<Dim> normalizedCoeffMat() const
        {
            CoefficientMat<Dim> nCoeffsMat;
            double t = 1.0;
            for (int i = order; i >= 0; i--)
            {
                nCoeffsMat.col(i) = coeffMat.col(i) * t;
                t *= duration;
            }
            return nCoeffsMat;
        }

        inline DotCoefficientMat<Dim> normalizedDotCoeffMat() const
        {
            DotCoefficientMat<Dim> nDotCoeffsMat;
            int n = 1;
            double t = duration;
            for (int i = order - 1; i >= 0; i--)
            {
                nDotCoeffsMat.col(i) = n * coeffMat.col(i) * t;
                t *= duration;
                n++;
            }
            return nDotCoeffsMat;
        }

        inline DDotCoefficientMat<Dim> normalizedDDotCoeffMat() const
        {
            DDotCoefficientMat<Dim> nDDotCoeffsMat;
            int n = 2;
            int m = 1;
            double t = duration * duration;
            for (int i = order - 2; i >= 0; i--)
            {
                nDDotCoeffsMat.col(i) = n * m * coeffMat.col(i) * t;
                n++;
                m++;
                t *= duration;
            }
            return nDDotCoeffsMat;
        }

        inline Eigen::Matrix<double, Dim, 1> getValue(const double &t) const
        {
            Eigen::Matrix<double, Dim, 1> value = Eigen::Matrix<double, Dim, 1>::Zero();
            double tn = 1.0;

            for (int i = order; i >= 0; i--)
            {
                value += tn * coeffMat.col(i);
                tn *= t;
            }

            return value;
        }

        inline Eigen::Matrix<double, Dim, 1> getDotValue(const double &t) const
        {
            Eigen::Matrix<double, Dim, 1> dvalue = Eigen::Matrix<double, Dim, 1>::Zero();
            double tn = 1.0;
            int n = 1;

            for (int i = order-1; i >= 0; i--)
            {
                dvalue += n * tn * coeffMat.col(i);
                tn *= t;
                n++;
            }

            return dvalue;
        }

        inline Eigen::Matrix<double, Dim, 1> getDDotValue(const double &t) const
        {
            Eigen::Matrix<double, Dim, 1> ddvalue = Eigen::Matrix<double, Dim, 1>::Zero();
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = order-2; i >= 0; i--)
            {
                ddvalue += m * n * tn * coeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            return ddvalue;
        }

        inline double getMaxDotValueNorm() const
        {
            DotCoefficientMat<Dim> nDsigmaCoeffMat = normalizedDotCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nDsigmaCoeffMat.row(0));
            for(int i = 1; i < Dim; i++)
            {
                coeff = coeff + RootFinder::polySqr(nDsigmaCoeffMat.row(i));
            }
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getDotValue(0.0).norm();
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                        FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxVelRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                    it != candidates.end();
                    it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getDotValue((*it) * duration).norm() * getDotValue((*it) * duration).norm();
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                    }
                }
                return sqrt(maxVelRateSqr);
            }
        }

        inline double getMaxDDotValueNorm() const
        {
            DDotCoefficientMat<Dim> nDDotCoeffsMat = normalizedDDotCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nDDotCoeffsMat.row(0));
            for(int i = 1; i < Dim; i++){
                coeff = coeff + RootFinder::polySqr(nDDotCoeffsMat.row(i));
            }
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getDDotValue(0.0).norm();
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                        FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                    it != candidates.end();
                    it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getDDotValue((*it) * duration).norm() * getDDotValue((*it) * duration).norm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }
    };

    template <int Dim>
    class PolyTrajectory
    {
    private:
        using Pieces = std::vector<Piece<Dim>>;
        Pieces pieces;

    public:
        PolyTrajectory() = default;

        PolyTrajectory(const std::vector<double> &durs,
                const std::vector<CoefficientMat<Dim>> &cMats)
        {
            int N = std::min(durs.size(), cMats.size());
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(durs[i], cMats[i]);
            }
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline const Piece<Dim> &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece<Dim> &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece<Dim> &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const double &dur,
                                const CoefficientMat<Dim> &cMat)
        {
            pieces.emplace_back(dur, cMat);
            return;
        }

        inline void append(const PolyTrajectory<Dim> &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                idx < N &&
                t > (dur = pieces[idx].getDuration());
                idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;
        }

        inline Eigen::Matrix<double, Dim, 1> getValue(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getValue(t);
        }

        inline Eigen::Matrix<double, Dim, 1> getDotValue(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getDotValue(t);
        }

        inline Eigen::Matrix<double, Dim, 1> getDDotValue(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getDDotValue(t);
        }

        inline double getMaxDotValueNorm() const
        {
            int N = getPieceNum();
            double maxDotValueNorm = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxDotValueNorm();
                maxDotValueNorm = maxDotValueNorm < tempNorm ? tempNorm : maxDotValueNorm;
            }
            return maxDotValueNorm;
        }

        inline double getMaxDDotValueNorm() const
        {
            int N = getPieceNum();
            double maxDDotValueNorm = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxDDotValueNorm();
                maxDDotValueNorm = maxDDotValueNorm < tempNorm ? tempNorm : maxDDotValueNorm;
            }
            return maxDDotValueNorm;
        }
    };

    class SE2Trajectory
    {
    public:
        SE2Trajectory() = default;
        PolyTrajectory<2> pos_traj;
        PolyTrajectory<1> yaw_traj;

        inline double getTotalDuration() const
        {
            return std::min(pos_traj.getTotalDuration(), yaw_traj.getTotalDuration());
        }

        inline Eigen::Vector2d getPos(double t) const
        {
            return pos_traj.getValue(t);
        }

        inline Eigen::Vector3d getSE2Pos(double t) const
        {
            Eigen::Vector3d se2_pos;
            se2_pos.head(2) = getPos(t);
            se2_pos(2) = getAngle(t);
            return se2_pos;
        }

        inline Eigen::Vector3d getNormSE2Pos(double t) const
        {
            Eigen::Vector3d se2_pos = getSE2Pos(t);

            while (se2_pos(2) < -M_PI)
                se2_pos(2) += 2*M_PI;
            while (se2_pos(2) > M_PI)
                se2_pos(2) -= 2*M_PI;
                
            return se2_pos;
        }

        inline Eigen::Vector2d getVel(double t) const
        {
            
            return pos_traj.getDotValue(t);
        }

        inline double getVelNorm(double t) const
        {
            return pos_traj.getDotValue(t).norm();
        }

        inline Eigen::Vector2d getAcc(double t) const
        {
            return pos_traj.getDDotValue(t);
        }

        inline double getAngle(double t) const
        {
            return yaw_traj.getValue(t).x();
        }

        inline double getAngleRate(double t) const
        {
            return yaw_traj.getDotValue(t).x();
        }

        inline double getLonAcc(double t) const
        {
            Eigen::Vector2d a = getAcc(t);
            double yaw = getAngle(t);
            return a.x() * cos(yaw) + a.y() * sin(yaw);
        }

        inline double getLatAcc(double t) const
        {
            Eigen::Vector2d a = getAcc(t);
            double yaw = getAngle(t);
            return -a.x() * sin(yaw) + a.y() * cos(yaw);
        }

        inline double getCur(double t) const
        {
            int eta = 1;
            if(getVelNorm(t) < 1.0e-4){
                return 0.0;
            }
            else
            {
                double yaw = getAngle(t);
                if(getVel(t).dot(Eigen::Vector2d(cos(yaw), sin(yaw))) < 0)
                    eta = -1;
            }
            return getAngleRate(t) / (eta * sqrt(getVel(t).squaredNorm() + delta_v));
        }

        inline double getMaxVelRate() const
        {
            return pos_traj.getMaxDotValueNorm();
        }

        inline double getMaxAccRate() const
        {
            return pos_traj.getMaxDDotValueNorm();
        }

        inline double getMaxLonAcc() const
        {
            double maxlonacc = 0.0;
            double tempNorm;
            for(double t = 0.0; t < getTotalDuration(); t += 0.01)
            {
                tempNorm = getLonAcc(t);
                if(fabs(maxlonacc) < fabs(tempNorm)){
                    maxlonacc = tempNorm;
                }
            }
            return maxlonacc;
        }

        inline double getMaxLatAcc() const
        {
            double maxlatacc = 0.0;
            double tempNorm;
            for(double t = 0.0; t < getTotalDuration(); t += 0.01)
            {
                tempNorm = getLatAcc(t);
                if(fabs(maxlatacc) < fabs(tempNorm)){
                    maxlatacc = tempNorm;
                }
            }
            return maxlatacc;
        }

        inline double getMaxCur() const
        {
            double maxcur = 0.0;
            double tempNorm;
            for(double t = 0.0; t < getTotalDuration(); t += 0.01){
                tempNorm = getCur(t);
                if(fabs(maxcur) < fabs(tempNorm)){
                    maxcur = tempNorm;
                }
            }
            return maxcur;
        }

        inline double getNonHolError() const
        {
            double error = 0.0;
            for(double t = 0.0; t < getTotalDuration(); t+= 0.01)
            {
                Eigen::Vector2d dotvalue = getVel(t);
                double yaw = getAngle(t);
                error += fabs(dotvalue.dot(Eigen::Vector2d(sin(yaw), -cos(yaw))));
            }
            return error;
        }
    };

    template <int Dim>  class MinJerkOpt
    {
    public:
        int N;
        Eigen::MatrixXd headPVA, tailPVA;
        BandedSystem A;
        Eigen::MatrixXd c;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;

    public:
        MinJerkOpt() = default;
        ~MinJerkOpt() { A.destroy(); }

        inline void reset(const int &pieceNum)
        {
            N = pieceNum;
            A.create(6 * N, 6, 6);
            c.resize(6 * N, Dim);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            return;
        }

        // from q,T to c,T
        inline void generate(const Eigen::MatrixXd &inPs,
                            const Eigen::VectorXd &ts,
                            const Eigen::MatrixXd &headState,
                            const Eigen::MatrixXd &tailState)
        {
            headPVA = headState;
            tailPVA = tailState;

            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);

            A.reset();
            c.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            c.row(0) = headPVA.col(0).transpose();
            c.row(1) = headPVA.col(1).transpose();
            c.row(2) = headPVA.col(2).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                c.row(6 * i + 5) = inPs.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            c.row(6 * N - 3) = tailPVA.col(0).transpose();
            c.row(6 * N - 2) = tailPVA.col(1).transpose();
            c.row(6 * N - 1) = tailPVA.col(2).transpose();

            A.factorizeLU();
            A.solve(c);

            return;
        }

        inline PolyTrajectory<Dim> getTraj() const
        {
            PolyTrajectory<Dim> polytraj;
            polytraj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                polytraj.emplace_back(T1(i),
                                  c.block<6, Dim>(6 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
            }
            return polytraj;
        }

        inline double getTrajJerkCost() const
        {
            double energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 36.0 * c.row(6 * i + 3).squaredNorm() * T1(i) +
                          144.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * T2(i) +
                          192.0 * c.row(6 * i + 4).squaredNorm() * T3(i) +
                          240.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * T3(i) +
                          720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * T4(i) +
                          720.0 * c.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return energy;
        }

        inline const Eigen::MatrixXd &getCoeffs(void) const
        {
            return c;
        }

        // know J=∫j²dt
        // then get ∂J/∂c, ∂J/∂T 
        inline void calJerkGradCT(Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT) 
        {
            gdC.resize(6 * N, Dim); 
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) = 240.0 * c.row(6 * i + 3) * T3(i) +
                                     720.0 * c.row(6 * i + 4) * T4(i) +
                                     1440.0 * c.row(6 * i + 5) * T5(i);
                gdC.row(6 * i + 4) = 144.0 * c.row(6 * i + 3) * T2(i) +
                                     384.0 * c.row(6 * i + 4) * T3(i) +
                                     720.0 * c.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) = 72.0 * c.row(6 * i + 3) * T1(i) +
                                     144.0 * c.row(6 * i + 4) * T2(i) +
                                     240.0 * c.row(6 * i + 5) * T3(i);
                gdC.block<3, Dim>(6 * i, 0).setZero();
            }

            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 36.0 * c.row(6 * i + 3).squaredNorm() +
                         288.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * T1(i) +
                         576.0 * c.row(6 * i + 4).squaredNorm() * T2(i) +
                         720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * T2(i) +
                         2880.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * T3(i) +
                         3600.0 * c.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return;
        }

        // know ∂K/∂C, ∂K/∂T, K(C(q,T),T) = W(q,T)
        // then get ∂W/∂q, ∂W/∂T
        inline void calGradCTtoQT(const Eigen::MatrixXd& gdC, Eigen::VectorXd& gdT, Eigen::MatrixXd& gdP)

        {
            gdP.resize(Dim, N - 1);
            Eigen::MatrixXd adjGrad = gdC;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gdP.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            Eigen::Matrix<double, 6, Dim> B1;
            Eigen::Matrix<double, 3, Dim> B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(c.row(i * 6 + 1) +
                              2.0 * T1(i) * c.row(i * 6 + 2) +
                              3.0 * T2(i) * c.row(i * 6 + 3) +
                              4.0 * T3(i) * c.row(i * 6 + 4) +
                              5.0 * T4(i) * c.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * c.row(i * 6 + 2) +
                              6.0 * T1(i) * c.row(i * 6 + 3) +
                              12.0 * T2(i) * c.row(i * 6 + 4) +
                              20.0 * T3(i) * c.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * c.row(i * 6 + 3) +
                              24.0 * T1(i) * c.row(i * 6 + 4) +
                              60.0 * T2(i) * c.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * c.row(i * 6 + 4) +
                              120.0 * T1(i) * c.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * c.row(i * 6 + 5);

                gdT(i) += B1.cwiseProduct(adjGrad.block<6, Dim>(6 * i + 3, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(c.row(6 * N - 5) +
                          2.0 * T1(N - 1) * c.row(6 * N - 4) +
                          3.0 * T2(N - 1) * c.row(6 * N - 3) +
                          4.0 * T3(N - 1) * c.row(6 * N - 2) +
                          5.0 * T4(N - 1) * c.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * c.row(6 * N - 4) +
                          6.0 * T1(N - 1) * c.row(6 * N - 3) +
                          12.0 * T2(N - 1) * c.row(6 * N - 2) +
                          20.0 * T3(N - 1) * c.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * c.row(6 * N - 3) +
                          24.0 * T1(N - 1) * c.row(6 * N - 2) +
                          60.0 * T2(N - 1) * c.row(6 * N - 1));

            gdT(N - 1) += B2.cwiseProduct(adjGrad.block<3, Dim>(6 * N - 3, 0)).sum();
            return;
        }
    };

    class MINCO_SE2
    {
    public:
        MinJerkOpt<2> pos_minco;
        MinJerkOpt<1> yaw_minco;
        
        inline void reset(const int& piece_xy, const int& piece_yaw)
        {
            pos_minco.reset(piece_xy);
            yaw_minco.reset(piece_yaw);
        }

        inline void generate(const Eigen::MatrixXd &initStateXY, \
                             const Eigen::MatrixXd &endStateXY , \
                             const Eigen::MatrixXd &innerPtsXY , \
                             const Eigen::VectorXd &timeXY     , \
                             const Eigen::MatrixXd &initYaw    , \
                             const Eigen::MatrixXd &endYaw     , \
                             const Eigen::MatrixXd &innerPtsYaw, \
                             const Eigen::VectorXd &timeYaw      )
        {
            pos_minco.generate(innerPtsXY, timeXY, initStateXY, endStateXY);
            yaw_minco.generate(innerPtsYaw, timeYaw, initYaw, endYaw);
        }

        inline SE2Trajectory getTraj() const
        {
            SE2Trajectory se2_traj;
            se2_traj.pos_traj = pos_minco.getTraj();
            se2_traj.yaw_traj = yaw_minco.getTraj();
            return se2_traj;
        }

        inline double getTrajJerkCost() const
        {
            return pos_minco.getTrajJerkCost() + yaw_minco.getTrajJerkCost();
        }

        inline void calJerkGradCT(Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy, \
                                  Eigen::MatrixXd& gdCyaw, Eigen::VectorXd &gdTyaw)
        {
            pos_minco.calJerkGradCT(gdCxy, gdTxy);
            yaw_minco.calJerkGradCT(gdCyaw, gdTyaw);
        }

        inline void calGradCTtoQT(const Eigen::MatrixXd& gdCxy, Eigen::VectorXd& gdTxy, Eigen::MatrixXd& gdPxy, \
                                  const Eigen::MatrixXd& gdCyaw, Eigen::VectorXd& gdTyaw, Eigen::MatrixXd& gdPyaw)
        {
            pos_minco.calGradCTtoQT(gdCxy, gdTxy, gdPxy);
            yaw_minco.calGradCTtoQT(gdCyaw, gdTyaw, gdPyaw);
        }
    };
}

#endif