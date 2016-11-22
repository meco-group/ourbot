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
#include "kalman_kinematic.hpp"

class OdometryGenericObservation  {
public:
  enum {OFF_X = 0, OFF_Y = 1, OFF_THETA = 2};
  void transform_R_dR(double theta, M<2, 2>& A, M<2, 2>& B);
};


class OdometryObservation : public KalmanObservation<3, 3>, OdometryGenericObservation {
public:
  virtual void observe(const M<3, 1>& x, const M<3, 3>& S, const M<3, 1>& u,
    M<3, 1>& xp, M<3, 3>& Sp, M<3, 1>& up);
  void set(double V_X, double V_Y, double omega);

private:
  M<2, 1> V;
  M<1, 1> Omega;

  KalmanObserver<3, 3, 3> ko;
};

template<int Nm>
class markerObservation : public KalmanObservation<3, 3>, OdometryGenericObservation {
public:
  virtual void observe(const M<3, 1>& x, const M<3, 3>& S, const M<3, 1>& u,
      M<3, 1>& xp, M<3, 3>& Sp, M<3, 1>& up);
  void set(const M<Nm, 2>& pattern_meas, const M<Nm, 2>& pattern_ref, double sigma);

private:
  M<Nm, 2> pattern_meas_;
  M<Nm, 2> pattern_ref_;
  double sigma_;

  KalmanObserver<3, 3, 2*Nm> ko;
};


template<int N>
class OdometryObservations {
  public:
  OdometryObservation odo;
  markerObservation<N> markers;
};

/**
 * OdometryFilter is a Kalman filter tracking a rigid body in a plane.
 *
 * The states being tracked are x, y and theta [m],[m],[rad].
 * Kinematic models of order n=1 (constant position) are used for each coordinate.
 *
 */
template<int Nm>
class OdometryFilter : public KinematicKalmanFilter< OdometryObservations<Nm> , 1, 1, 1> {
  public:
  /**
   * Initialize the OdometryFilter, given noise levels for the x,y and theta models.
   * These noises are velocity-like squared [(m/s)^2]
   *
   *
   */
  OdometryFilter(double psd_x, double psd_y, double psd_theta, int buffer=100);

  /**
    Set the Kalman filter to the origin and with large covariance
  */
  void unknown(double t, double sigma=1e6);

  /**
    Observe odometry given in the local frame:
      V_X: velocity in x direction [m/s]
      V_Y: velocity in x direction [m/s]
      omega: angular velocity [rad/s]

  */
  void observe_odo(double t, double V_X, double V_Y, double omega);

  /**
    Observe markers on objects:
      pattern_meas: the meaured positions of markers [m], in the global reference frame
      pattern_ref: the nominal layout of markers [m], in the local reference frame

      sigma: uncertainty on pattern_meas [m^2]
  */
  void observe_markers(double t, const M<Nm, 2>& pattern_meas,
    const M<Nm, 2>& pattern_ref, double sigma);
};

extern template class OdometryFilter<3>;
