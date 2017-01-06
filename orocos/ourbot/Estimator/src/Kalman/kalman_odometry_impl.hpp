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
#include "kalman_odometry.hpp"

template<int Nm>
void markerObservation<Nm>::observe(const M<6, 1>& x, const M<6, 6>& S, const M<0, 1>& u,
    M<6, 1>& xp, M<6, 6>& Sp) {
  M<2, 2> R, dR;

  transform_R_dR(x(OFF_THETA), R, dR);
  M<2, 1> p;
  p << x(OFF_X), x(OFF_Y);
  M<2, Nm> pr = p*M<1, Nm>::Ones(1, Nm);

  M<2, Nm> m = pattern_meas_.transpose()  - (R*pattern_ref_.transpose() + pr);

  M<2*Nm, 6> H;
  H.setConstant(0);

  for (int i=0;i<Nm;++i) {
    H.block(2*i, OFF_THETA, 2, 1) = dR*pattern_ref_.block(i, 0, 1, 2).transpose();
    H(2*i,   OFF_X) = 1;
    H(2*i+1, OFF_Y) = 1;
  }

  // Eigen is Column major
  M <2*Nm, 1> z = Eigen::Map< M<2*Nm, 1> > (m.data(), 2*Nm) + H*x;
  M <2*Nm, 2*Nm> Rm = M<2*Nm, 2*Nm>::Identity(2*Nm, 2*Nm)*sigma_;

  ko.observe(x, S, xp, Sp, H, Rm, z);
}

template<int Nm>
void markerObservation<Nm>::set(const M<Nm, 2>& pattern_meas,
    const M<Nm, 2>& pattern_ref, double sigma) {
  pattern_meas_ = pattern_meas;
  pattern_ref_ = pattern_ref;
  sigma_ = sigma;
}

template<int Nm>
OdometryFilter<Nm>::OdometryFilter(double psd_x, double psd_y, double psd_theta, int buffer):
  KinematicKalmanFilter<OdometryObservations<Nm>, 2, 2, 2>({psd_x, psd_y, psd_theta}, buffer) {

}

template<int Nm>
void OdometryFilter<Nm>::unknown(double t, double sigma) {
  this->reset(t, M<6, 1>::Zero(6, 1), sigma*M<6, 6>::Identity(6, 6));
}

template<int Nm>
void OdometryFilter<Nm>::observe_odo(double t, double V_X, double V_Y, double omega,
    double sigma_X, double sigma_Y, double sigma_omega) {
  auto e = this->pop_event();
  e->active_observation = &e->m.odo;
  e->m.odo.set(V_X, V_Y, omega, sigma_X, sigma_Y, sigma_omega);
  this->add_event(t, e);
}

template<int Nm>
void OdometryFilter<Nm>::observe_markers(double t, const M<Nm, 2>& pattern_meas,
    const M<Nm, 2>& pattern_ref, double sigma) {
  auto e = this->pop_event();
  e->active_observation = &e->m.markers;
  e->m.markers.set(pattern_meas, pattern_ref, sigma);
  this->add_event(t, e);
}
