#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

template<int D>
using CoefficientMat = Eigen::Matrix<double, D, 6>;
template<int D>
using VelCoefficientMat = Eigen::Matrix<double, D, 5>;
template<int D>
using AccCoefficientMat = Eigen::Matrix<double, D, 4>;

namespace mpc_utils
{
  template<int D>
  class Piece 
  {
    private:
      double duration;
      CoefficientMat<D> coeffMat;

    public:
      Piece() = default;

      Piece(double dur, const CoefficientMat<D> &cMat)
          : duration(dur), coeffMat(cMat) {}

      inline int getDim() const { return D; }

      inline int getOrder() const { return 5; }

      inline double getDuration() const { return duration; }

      inline const CoefficientMat<D> &getCoeffMat() const { return coeffMat; }

      inline Eigen::Matrix<double, D, 1> getPos(const double &t) const 
      {
        Eigen::Matrix<double, D, 1> pos;
        pos.setZero();
        double tn = 1.0;
        for (int i = 5; i >= 0; i--) {
          pos += tn * coeffMat.col(i);
          tn *= t;
        }
        return pos;
      }

      inline Eigen::Matrix<double, D, 1> getVel(const double &t) const 
      {
        Eigen::Matrix<double, D, 1> vel;
        vel.setZero();
        double tn = 1.0;
        int n = 1;
        for (int i = 4; i >= 0; i--) {
          vel += n * tn * coeffMat.col(i);
          tn *= t;
          n++;
        }
        return vel;
      }

      inline Eigen::Matrix<double, D, 1> getAcc(const double &t) const 
      {
        Eigen::Matrix<double, D, 1> acc;
        acc.setZero();
        double tn = 1.0;
        int m = 1;
        int n = 2;
        for (int i = 3; i >= 0; i--) {
          acc += m * n * tn * coeffMat.col(i);
          tn *= t;
          m++;
          n++;
        }
        return acc;
      }

      inline Eigen::Matrix<double, D, 1> getJer(const double &t) const 
      {
        Eigen::Matrix<double, D, 1> jer;
        jer.setZero();
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        for (int i = 2; i >= 0; i--) {
          jer += l * m * n * tn * coeffMat.col(i);
          tn *= t;
          l++;
          m++;
          n++;
        }
        return jer;
      }
  };

  template<int D>
  class Trajectory {
  private:
    typedef std::vector<Piece<D>> Pieces;
    Pieces pieces;

  public:
    Trajectory() = default;

    Trajectory(const std::vector<double> &durs,
              const std::vector<CoefficientMat<D>> &cMats) 
    {
      int N = std::min(durs.size(), cMats.size());
      pieces.reserve(N);
      for (int i = 0; i < N; i++) {
        pieces.emplace_back(durs[i], cMats[i]);
      }
    }

    inline int getPieceNum() const { return pieces.size(); }

    inline Eigen::VectorXd getDurations() const 
    {
      int N = getPieceNum();
      Eigen::VectorXd durations(N);
      for (int i = 0; i < N; i++) {
        durations(i) = pieces[i].getDuration();
      }
      return durations;
    }

    inline double getTotalDuration() const 
    {
      int N = getPieceNum();
      double totalDuration = 0.0;
      for (int i = 0; i < N; i++) {
        totalDuration += pieces[i].getDuration();
      }
      return totalDuration;
    }

    inline Eigen::MatrixXd getPositions() const 
    {
      int N = getPieceNum();
      Eigen::MatrixXd positions(D, N + 1);
      for (int i = 0; i < N; i++) {
        positions.col(i) = pieces[i].getCoeffMat().col(5);
      }
      positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
      return positions;
    }

    inline const Piece<D> &operator[](int i) const { return pieces[i]; }

    inline Piece<D> &operator[](int i) { return pieces[i]; }

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

    inline void emplace_back(const Piece<D> &piece) 
    {
      pieces.emplace_back(piece);
      return;
    }

    inline void emplace_back(const double &dur, const CoefficientMat<D> &cMat) 
    {
      pieces.emplace_back(dur, cMat);
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
          idx++) {
        t -= dur;
      }
      if (idx == N) {
        idx--;
        t += pieces[idx].getDuration();
      }
      return idx;
    }

    inline Eigen::Matrix<double, D, 1> getPos(double t) const 
    {
      int pieceIdx = locatePieceIdx(t);
      return pieces[pieceIdx].getPos(t);
    }

    inline Eigen::Matrix<double, D, 1> getVel(double t) const 
    {
      int pieceIdx = locatePieceIdx(t);
      return pieces[pieceIdx].getVel(t);
    }

    inline Eigen::Matrix<double, D, 1> getAcc(double t) const 
    {
      int pieceIdx = locatePieceIdx(t);
      return pieces[pieceIdx].getAcc(t);
    }

    inline Eigen::Matrix<double, D, 1> getJer(double t) const 
    {
      int pieceIdx = locatePieceIdx(t);
      return pieces[pieceIdx].getJer(t);
    }
  };

  // The banded system class is used for solving
  // banded linear system Ax=b efficiently.
  // A is an N*N band matrix with lower band width lowerBw
  // and upper band width upperBw.
  // Banded LU factorization has O(N) time complexity.
  class BandedSystem 
  {
    public:
      // The size of A, as well as the lower/upper
      // banded width p/q are needed
      inline void create(const int &n, const int &p, const int &q) 
      {
        // In case of re-creating before destroying
        destroy();
        N = n;
        lowerBw = p;
        upperBw = q;
        int actualSize = N * (lowerBw + upperBw + 1);
        ptrData = new double[actualSize];
        std::fill_n(ptrData, actualSize, 0.0);
        return;
      }

      inline void destroy() 
      {
        if (ptrData != nullptr) {
          delete[] ptrData;
          ptrData = nullptr;
        }
        return;
      }

    private:
      int N;
      int lowerBw;
      int upperBw;
      // Compulsory nullptr initialization here
      double *ptrData = nullptr;

    public:
      // Reset the matrix to zero
      inline void reset(void) 
      {
        std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
        return;
      }

      // The band matrix is stored as suggested in "Matrix Computation"
      inline const double &operator()(const int &i, const int &j) const 
      {
        return ptrData[(i - j + upperBw) * N + j];
      }

      inline double &operator()(const int &i, const int &j) 
      {
        return ptrData[(i - j + upperBw) * N + j];
      }

      // This function conducts banded LU factorization in place
      // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
      inline void factorizeLU() 
      {
        int iM, jM;
        double cVl;
        for (int k = 0; k <= N - 2; k++) {
          iM = std::min(k + lowerBw, N - 1);
          cVl = operator()(k, k);
          for (int i = k + 1; i <= iM; i++) {
            if (operator()(i, k) != 0.0) {
              operator()(i, k) /= cVl;
            }
          }
          jM = std::min(k + upperBw, N - 1);
          for (int j = k + 1; j <= jM; j++) {
            cVl = operator()(k, j);
            if (cVl != 0.0) {
              for (int i = k + 1; i <= iM; i++) {
                if (operator()(i, k) != 0.0) {
                  operator()(i, j) -= operator()(i, k) * cVl;
                }
              }
            }
          }
        }
        return;
      }

      // This function solves Ax=b, then stores x in b
      // The input b is required to be N*m, i.e.,
      // m vectors to be solved.
      inline void solve(Eigen::MatrixXd &b) const 
      {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
          iM = std::min(j + lowerBw, N - 1);
          for (int i = j + 1; i <= iM; i++) {
            if (operator()(i, j) != 0.0) {
              b.row(i) -= operator()(i, j) * b.row(j);
            }
          }
        }
        for (int j = N - 1; j >= 0; j--) {
          b.row(j) /= operator()(j, j);
          iM = std::max(0, j - upperBw);
          for (int i = iM; i <= j - 1; i++) {
            if (operator()(i, j) != 0.0) {
              b.row(i) -= operator()(i, j) * b.row(j);
            }
          }
        }
        return;
      }
  };

  template<int D>
  class MincoTraj 
  {
    public:
      int N;
      Eigen::MatrixXd headPVA;
      Eigen::VectorXd T1;
      BandedSystem A;
      Eigen::MatrixXd b;

      // Temp variables
      Eigen::VectorXd T2;
      Eigen::VectorXd T3;
      Eigen::VectorXd T4;
      Eigen::VectorXd T5;

      MincoTraj() = default;
      ~MincoTraj() { A.destroy(); }

      inline void reset(const Eigen::MatrixXd &headState,
                        const int &pieceNum) 
      {
        N = pieceNum;
        headPVA = headState;
        T1.resize(N);
        A.create(6 * N, 6, 6);
        b.resize(6 * N, D);
        return;
      }

      inline void generate(const Eigen::MatrixXd &inPs,
                          const Eigen::MatrixXd &tailPVA,
                          const Eigen::VectorXd &ts) 
      {
        T1 = ts;
        T2 = T1.cwiseProduct(T1);
        T3 = T2.cwiseProduct(T1);
        T4 = T2.cwiseProduct(T2);
        T5 = T4.cwiseProduct(T1);

        A.reset();
        b.setZero();

        A(0, 0) = 1.0;
        A(1, 1) = 1.0;
        A(2, 2) = 2.0;
        b.row(0) = headPVA.col(0).transpose();
        b.row(1) = headPVA.col(1).transpose();
        b.row(2) = headPVA.col(2).transpose();

        for (int i = 0; i < N - 1; i++) {
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

          b.row(6 * i + 5) = inPs.col(i).transpose();
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

        b.row(6 * N - 3) = tailPVA.col(0).transpose();
        b.row(6 * N - 2) = tailPVA.col(1).transpose();
        b.row(6 * N - 1) = tailPVA.col(2).transpose();

        A.factorizeLU();
        A.solve(b);

        return;
      }

      inline Trajectory<D> getTraj(void) const 
      {
        Trajectory<D> traj;
        traj.reserve(N);
        for (int i = 0; i < N; i++) {
          traj.emplace_back(T1(i), b.block<6, D>(6 * i, 0).transpose().rowwise().reverse());
        }
        return traj;
      }
    };

    typedef MincoTraj<1> MincoTraj1d;
    typedef MincoTraj<2> MincoTraj2d;
    typedef MincoTraj<3> MincoTraj3d;

    class SE2Traj
    {
      public:
        Trajectory<1> angle_traj;
        Trajectory<2> pos_traj;
    };

    class MINCO_SE2
    {
      public:
        MincoTraj1d angle_anal;
        MincoTraj2d pos_anal;
    };
}