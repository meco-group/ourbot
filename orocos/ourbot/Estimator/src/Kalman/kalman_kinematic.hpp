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
#include "kalman_filter.hpp"

/** \brief Meta-programming helpers to compute the some of integer types
*/
template<typename T>
constexpr T pack_add(T v) {
  return v;
}

template<typename T, typename... Args>
constexpr T pack_add(T first, Args... args) {
  return first + pack_add(args...);
}

/**
*  KinematicKalmanFilter is a subclass of KalmanFilter that comes with a Kinematic model
*
*  Depending on the order, a kinematic model has a simple interpretation:
*     order 1: constant position; disturbances (psd) are speed-like squared (m/s)^2
*     order 2: constant vecolity; disturbances (psd) are acceleration-like squared (m/s^2)^2
*     order 3: constant acceleration; disturbances (psd) are jerk-like squared (m/s^3)^2
*     ...
*
*  This class allows for a variable number of template arguments
*  Example:
*  KinematicKalmanFilter< Measurements, 2, 1, 3>
*    Represents a system consisting of a 2-order coordinate, a 1-order coordinate,
*    and 3-order coordinate stacked together: [x_0;v_0;x_1;x_2;v_2;a_2].
*
*    Such system has block-diaginal covariance unless the measurements mix several coordinates.
*/

template <int N>
M<N, N> KinematicA(const std::vector<int>& orders) {

  M<N, N> A;
  A.setConstant(0);
  int offset_x = 0;
  for (size_t i=0;i<orders.size();++i) {
    int n = orders[i];
    A.block(offset_x, offset_x, n, n) << Md::Zero(n, 1), Md::Identity(n, n-1);
    offset_x+= orders[i];
  }
  return A;
}

template <int N>
M<N, 0> KinematicB(const std::vector<int>& orders) {
  return M<N, 0>();
}

template <int N>
M<N, N> KinematicQ(const std::vector<int>& orders, const std::vector<double>& psd) {
  M<N, N> Q;
  Q.setConstant(0);
  int offset_x = 0;
  for (size_t i=0;i<orders.size();++i) {
    int n = orders[i];
    Q(offset_x+n-1, offset_x+n-1) = psd[i];
    offset_x+= orders[i];
  }

  return Q;
}

template <class Measurements, int... order>
class KinematicKalmanFilter:  public KalmanFilter< pack_add(order...), 0, Measurements> {
public:

  KinematicKalmanFilter(const std::vector<double>& psd, int buffer=100) :
      KalmanFilter<pack_add(order...), 0, Measurements>(
          KinematicA< pack_add(order...) >({order...}),
          KinematicB< pack_add(order...) >({order...}),
          KinematicQ< pack_add(order...) >({order...}, psd), buffer) {

  }

};
