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

void OdometryObservation::observe(const M<3, 1>& x, const M<3, 3>& S, const M<3, 1>& u,
    M<3, 1>& xp, M<3, 3>& Sp, M<3, 1>& up) {
  xp = x;
  Sp = Sp;

  M<2, 2> R, dR;

  transform_R_dR(x(OFF_THETA), R, dR);

  up << R*V, Omega;
}

void OdometryObservation::set(double V_X, double V_Y, double omega) {
  V << V_X, V_Y;
  Omega << omega;
}
