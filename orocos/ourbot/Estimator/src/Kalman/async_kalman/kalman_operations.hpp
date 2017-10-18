/*
 *    This file is part of async_kalman.
 *
 *    async_kalman -- an synchronous kalman filter implementation
 *    Copyright (C) 2016 Joris Gillis,
 *                            K.U. Leuven. All rights reserved.
 *
 *    async_kalman is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    async_kalman is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with async_kalman; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <iostream>
#include <assert.h>

template <int Nr, int Nc>
using M = typename Eigen::Matrix<double, Nr, Nc>;

typedef Eigen::MatrixXd Md;

// Need an 8-byte integer since libslicot0 is compiled with  fdefault-integer-8
typedef long long int f_int;

// https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html

void slicot_mb05nd(int n, double delta, const double* A, int lda,
                   double* ex, int ldex, double* exint, int ldexin,
                   double tol, f_int* iwork, double* dwork, int ld_work, int& info);

template<int N>
void expm(const M<N, N>& A, double t, M<N, N>& Ae, M<N, N>& Aei,
    std::vector<f_int>& iwork, std::vector<double>& dwork) {
  int info;
  slicot_mb05nd(N, t, A.data(), N, Ae.data(), N, Aei.data(), N, 1e-7, &iwork[0], &dwork[0], 2*N*N,
    info);
}

/**
  x' = Ax + Bu + w

  With Cov(w)=Q

  Integrates forward in time to attain A_discrete, B_discrete, Q_discrete
*/
template <int N , int Nb>
class KalmanIntegrator {
public:
  KalmanIntegrator() : iwork_(2*N), dwork_(2*N*N*4) {
    F.setConstant(0);
  };

  void integrate(const M<N, N>& A, const M<N, Nb>& B, const M<N, N>&Q, double t,
                 M<N, N>& Ad, M<N, Nb>& Bd, M<N, N>& Qd) {
    expm(A, t, Ad, Aei, iwork_, dwork_);

    Bd = Aei*B;

    // trick Van Loan, 1978
    F.block(0,  0,  N, N) = -A;
    F.block(N, N, N, N) = A.transpose();
    F.block(0,  N, N, N) = Q;

    expm(F, t, Fd, Fei, iwork_, dwork_);
    Qd = Fd.block(N, N, N, N).transpose()*Fd.block(0, N, N, N);
  }

private:

  std::vector<f_int> iwork_;
  std::vector<double> dwork_;

  M<N, N> Aei; // placeholder for int expm(A) t
  M<2*N, 2*N> F;
  M<2*N, 2*N> Fd;
  M<2*N, 2*N> Fei;
};

/**
  Propagates state and covariance forward in time.

  More correctly, it propagate the Cholesky form of the covariance.

*/
template <int N, int Nu>
class KalmanPropagator {
public:
  KalmanPropagator() {}

  /**
    Given state x and sqrt-covariance S, propagates over a duration t
    to attain state xp and sqrt-covariance Sp.
    The continuous system dynamics (A, B) and process covariance (Q) is needed,
    and so is a value for the sample-and-hold input (u)
  */
  void propagate(const M<N, 1>& x, const M<N, N>& S, double t,
                 M<N, 1>& xp, M<N, N>& Sp,
                 const M<N, N>& A, const M<N, Nu>& B, const M<N, N>& Q, const M<Nu, 1>& u) {
    assert(t>=0);
    if (t==0) {
      xp << x;
      Sp << S;
      return;
    }
    // Discretize system
    ki.integrate(A, B, Q, t, Ad, Bd, Qd);

    // Propagate state
    xp = Ad*x+Bd*u;

    // Propagate covariance
    Qf.compute(Qd);
    SQ = Qf.matrixL();

    Mm << (Ad*S).transpose(), SQ.transpose();
    qr.compute(Mm);
    Sp = qr.matrixQR().block(0, 0, N, N).template triangularView<Eigen::Upper>().transpose();
  }

private:

  M<2*N, N> Mm;
  Eigen::HouseholderQR< M<2*N, N> > qr;

  KalmanIntegrator<N, Nu> ki;

  M<N, N> Ad;
  M<N, Nu> Bd;
  M<N, N> Qd;
  M<N, N> SQ;

  Eigen::LLT< M<N, N> > Qf;
};



template <int N, int Nu, int Ny>
class KalmanObserver {
public:
  KalmanObserver() {};

  /**
    y = C*x + D*u

    Updates the Kalman state x and sqrt-covariance S to attain state xp and sqrt-covariance Sp,
    using observation matrices (C,D), measurement covariance R and observation vector z.

    Also needs a value for the sample-and-hold input (u)

  */
  void observe(const M<N, 1>& x, const M<N, N>& S, M<N, 1>& xp, M<N, N>& Sp,
      const M<Ny, N>& C, const M<Ny, Nu>& D, const M<Ny, Ny>& R, const M<Ny, 1>& z,
      const M<Nu, 1>& u) {
    // Output
    zp = C*x+D*u;

    // Propagate covariance
    Rf.compute(R);
    SR = Rf.matrixL();

    Mm << (C*S).transpose(), SR.transpose();

    qr.compute(Mm);
    SZ = qr.matrixQR().block(0, 0, Ny, Ny).template triangularView<Eigen::Upper>().transpose();

    L = C*(S*S.transpose());

    SZ.template triangularView<Eigen::Lower>().solveInPlace(L);
    SZ.template triangularView<Eigen::Lower>().transpose().solveInPlace(L);

    xp = x + L.transpose()*(z - zp);
    LZ = L.transpose()*SZ;

    Sp = S;
    for (int i=0;i<Ny;++i) {
      Eigen::internal::llt_inplace<double, Eigen::Lower>::rankUpdate(Sp, LZ.col(i), -1);
    }
  }
  /** observe variant without inputs */
  void observe(const M<N, 1>& x, const M<N, N>& S, M<N, 1>& xp, M<N, N>& Sp,
      const M<Ny, N>& C, const M<Ny, Ny>& R, const M<Ny, 1>& z) {
    u.setConstant(0);
    Dm.setConstant(0);
    observe(x, S, xp, Sp, C, Dm, R, z, u);
  };

private:

  M<N + Ny, Ny> Mm;
  Eigen::HouseholderQR< M<N + Ny, Ny> > qr;


  M<Ny, Ny> SR;

  Eigen::LLT< M<Ny, Ny> > Rf;

  M<Ny, 1> zp;
  M<Ny, Ny> SZ;
  M<Ny, N> L;
  M<N, Ny> LZ;

  Eigen::LLT< M<N, N> > Sf;


  M<Ny, Nu> Dm;
  M<Nu, 1> u;
};
