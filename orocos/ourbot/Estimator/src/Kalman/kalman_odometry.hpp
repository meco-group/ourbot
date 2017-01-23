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
  enum {OFF_X = 0, OFF_Y = 2, OFF_THETA = 4};
  void transform_R_dR(double theta, M<2, 2>& A, M<2, 2>& B);
};


class OdometryObservation : public KalmanObservation<6, 0>, OdometryGenericObservation {
public:
  virtual void observe(const M<6, 1>& x, const M<6, 6>& S, const M<0, 1>& u,
    M<6, 1>& xp, M<6, 6>& Sp);
  void set(double V_X, double V_Y, double omega,
    double sigma_X, double sigma_Y, double sigma_omega);

private:
  M<3, 1> V;
  M<3, 3> Sigma;

  KalmanObserver<6, 0, 3> ko;
};

template<int Nm>
class markerObservation : public KalmanObservation<6, 0>, OdometryGenericObservation {
public:
  virtual void observe(const M<6, 1>& x, const M<6, 6>& S, const M<0, 1>& u,
      M<6, 1>& xp, M<6, 6>& Sp);
  void set(const M<Nm, 2>& pattern_meas, const M<Nm, 2>& pattern_ref, double sigma);

private:
  M<Nm, 2> pattern_meas_;
  M<Nm, 2> pattern_ref_;
  double sigma_;

  KalmanObserver<6, 0, 2*Nm> ko;
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
 * Kinematic models of order n=2 (constant velocity) are used for each coordinate.
 * 
 */
template<int Nm>
class OdometryFilter : public KinematicKalmanFilter<OdometryObservations<Nm>, 2, 2, 2> {
  public:
  /** 
   * Initialize the OdometryFilter, given noise levels for the x,y and theta models.
   * These noises are acceleration-like squared [(m/s^2)^2]
   * 
   * E.g. if the rigid body is on a slippery hilly landscape, innovations in the kimenatic kalman filter
   * may be on the order of gravity. In that case you could set psd_x and psd_y to 9.81^2
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

      sigma_X: uncertainty on V_X [m^2/s^2]
      sigma_Y: uncertainty on V_Y [m^2/s^2]
      omega: uncertainty on omega [rad^2/s^2]

  */
  void observe_odo(double t, double V_X, double V_Y, double omega,
    double sigma_X, double sigma_Y, double sigma_omega);

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
