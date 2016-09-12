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
#define KALMAN_ODOMETRY_IMPLEMENTATION
#include "kalman_odometry_impl.hpp"

template class OdometryFilter<3>;

void OdometryGenericObservation::transform_R_dR(double theta, M<2, 2>& A, M<2, 2>& B) {
  double st = sin(theta);
  double ct = cos(theta);
  A << ct, -st, st, ct;
  B << -st, -ct, ct, -st;
}

void OdometryObservation::observe(const M<6, 1>& x, const M<6, 6>& S, const M<0, 1>& u,
    M<6, 1>& xp, M<6, 6>& Sp) {
  M<2, 2> R, dR;

  transform_R_dR(x(OFF_THETA), R, dR);

  M<2, 1> v;
  v << x(OFF_X+1), x(OFF_Y+1);

  M<3, 6> H;
  H.setConstant(0);
  H(2, OFF_THETA+1) = 1;
  H.block(0, OFF_X+1, 2, 1) = R.transpose().block(0, 0, 2, 1);
  H.block(0, OFF_Y+1, 2, 1) = R.transpose().block(0, 1, 2, 1);
  H.block(0, OFF_THETA, 2, 1) = dR*v;

  M<2, 1> h = R.transpose()*v;
  M<3, 1> r;
  r << h-H.block(0, 0, 2, 6)*x, 0;

  ko.observe(x, S, xp, Sp, H, Sigma, V-r);
}

void OdometryObservation::set(double V_X, double V_Y, double omega,
    double sigma_X, double sigma_Y, double sigma_omega) {
  V << V_X, V_Y, omega;
  Sigma << sigma_X, 0, 0,   0, sigma_Y, 0,  0, 0, sigma_omega;
}
