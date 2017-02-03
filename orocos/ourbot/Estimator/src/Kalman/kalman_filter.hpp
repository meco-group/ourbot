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
#include "kalman_operations.hpp"
#include <vector>
#include <map>
#include <iostream>
#include <assert.h>

/** \brief Master class for observations  */
template <int N, int Nu>
class KalmanObservation {
  public:
  virtual void observe(const M<N, 1>& x, const M<N, N>& S, const M<Nu, 1>& u,
    M<N, 1>& xp, M<N, N>& Sp) = 0;
};

/** \brief Observation consisting of constant C,D matrices */
template <int N, int Nu, int Ny>
class SimpleObservation : public KalmanObservation<N, Nu> {
public:
  virtual void observe(const M<N, 1>& x, const M<N, N>& S, const M<Nu, 1>& u,
      M<N, 1>& xp, M<N, N>& Sp) {
    ko.observe(x, S, xp, Sp, C_, D_, R_, z_, u);
  }
  void set(const M<Ny, N>&C, const M<Ny, Nu> &D, const M<Ny, Ny>&R, const M<Ny, 1>&z) {
    C_ = C;
    D_ = D;
    R_ = R;
    z_ = z;
  }
private:
  M<Ny, N> C_;
  M<Ny, Nu> D_;
  M<Ny, Ny> R_;
  M<Ny, 1> z_;

  KalmanObserver<N, Nu, Ny> ko;
};

/** \brief Represent any Event given to the Kalman filter */
template <int N, int Nu, class Measurements>
class KalmanEvent {
public:
  // Mutable part
  M<N,  N> S_cache;
  M<N,  1> x_cache;
  M<Nu, 1> u_cache;

  // Immutable
  Measurements m;

  KalmanObservation<N, Nu> * active_observation;

};

/** \brief Buffer containing all Kalman Events
 *
 *  Properties:
 *    - Ordered by time (early to late)
 *    - Non-unique keys (time) allowed
 *    - log(N) to remove or add items
 *    - log(N) to look up first elemenet bigger than some value
 */
template <int N, int Nu, class Measurements>
class EventBuffer {
public:
  typedef std::multimap< double, KalmanEvent<N, Nu, Measurements>* > EventMap;
  typedef typename EventMap::iterator EventMapIt;
  EventBuffer(int max_size) : max_size_(max_size), event_pool_(max_size) {}

  // Get available event
  KalmanEvent<N, Nu, Measurements>* pop_event() {
    if (pool_count < event_pool_.size()) {
      // If pool not exhausted; return an Event from the pool
      return &event_pool_[pool_count++];
    } else {
      // If pool exhausted, start recycling events from the buffer
      EventMapIt lastit = buffer_.begin();
      KalmanEvent<N, Nu, Measurements>* ret = lastit->second;
      buffer_.erase(lastit);
      return ret;
    }
  }

  EventMapIt add_event(double t, KalmanEvent<N, Nu, Measurements>* e) {
    return buffer_.insert(std::pair<double, KalmanEvent<N, Nu, Measurements>* >(t, e));
  }

  EventMapIt end() {
    return buffer_.end();
  }
  EventMapIt begin() {
    return buffer_.begin();
  }

  std::pair<EventMapIt, EventMapIt> equal_range(double t) {
    return buffer_.equal_range(t);
  }

  KalmanEvent<N, Nu, Measurements>* get_event(double t, double& te) {
    auto it = buffer_.upper_bound(t);
    if (it==buffer_.begin()) return 0;
    --it;
    te = it->first;
    return it->second;
  }

private:
  int max_size_;
  std::vector< KalmanEvent<N, Nu, Measurements> > event_pool_;
  EventMap buffer_;

  // How many pre-allocated events have been used up
  size_t pool_count = 0;
};

/** \brief Kalman Filter

*/
template <int N, int Nu, class Measurements>
class KalmanFilter {
public:
  using EventMapIt = typename std::multimap< double, KalmanEvent<N, Nu, Measurements>* >::iterator;
  KalmanFilter(const M<N, N>&A, const M<N, Nu>&B, const M<N, N>&Q, int buffer=100) :
      buffer_(buffer), A_(A), B_(B), Q_(Q) {
    work_todo = buffer_.end();
  }

  KalmanEvent<N, Nu, Measurements>* pop_event() {
    return buffer_.pop_event();
  }
  void add_event(double t, KalmanEvent<N, Nu, Measurements>* e) {
    // Add event to the buffer
    EventMapIt it_insert = buffer_.add_event(t, e);
    work_todo = it_min(work_todo, it_insert);
  }

  EventMapIt it_min(const EventMapIt& a, const EventMapIt& b) {
    if (a==buffer_.end()) return b;
    if (b==buffer_.end()) return a;
    if (a->first < b->first) return a;
    if (b->first < a->first) return b;

    // The following happens in multiset
    std::pair<EventMapIt, EventMapIt> ret = buffer_.equal_range(a->first);
    for (EventMapIt it=ret.first; it!=ret.second; ++it) {
      if (it==a || it==b) return it;
    }
    assert(false);
    return buffer_.end();
  }

  void update() {
    if (work_todo==buffer_.end()) return;
    // Update Kalman cache
    EventMapIt it_insert = work_todo;
    EventMapIt it_ref = it_insert;

    if (it_insert->second->active_observation) {
      if (it_ref == buffer_.begin()) {
        std::cerr << "Error: First Kalman Event should be a reset" << std::endl;
        exit(1);
      }
      --it_ref;
    }

    // Obtain previous starting value
    const M<N, 1>* x_ref = &it_ref->second->x_cache;
    const M<N, N>* S_ref = &it_ref->second->S_cache;
    M<Nu, 1> u_ref;
    double t_ref =  it_ref->first;

    for (auto it=it_insert;it!=buffer_.end();++it) {
      if (it->second->active_observation) {
        // Propagate from starting value to current
        kp.propagate(*x_ref, *S_ref, it->first-t_ref, it->second->x_cache, it->second->S_cache,
          A_, B_, Q_, u_ref);
        // Measurement update
        it->second->active_observation->observe(it->second->x_cache, it->second->S_cache, u_ref,
          it->second->x_cache, it->second->S_cache);
      }
      // Current value becomes new starting value
      x_ref = &it->second->x_cache;
      S_ref = &it->second->S_cache;
      t_ref =  it->first;
    }
    // Nothing to do
    work_todo = buffer_.end();
  }

  void reset(double t, const M<N, 1>&x, const M<N, N>&P) {
    SP.compute(P);
    auto e = pop_event();
    e->active_observation = 0;
    e->x_cache = x;
    e->S_cache = SP.matrixL();
    add_event(t, e);
  }
  void input(double t, const M<Nu, 1>&u) {

  }

  void predict(double t, M<N, 1>& x, M<N, N>& P) {
    update();
    double te;
    auto ref = buffer_.get_event(t, te);
    if (ref) {
      kp.propagate(ref->x_cache, ref->S_cache, t-te, x, P, A_, B_, Q_, ref->u_cache);
      P = P*P.transpose();
    } else {
      x.setConstant(NAN);
      P.setConstant(NAN);
    }
  }

private:
  EventBuffer<N, Nu, Measurements> buffer_;
  Eigen::LLT< M<N, N> > SP;
  KalmanPropagator<N, Nu> kp;


  M<N, N> A_;
  M<N, Nu> B_;
  M<N, N> Q_;

  EventMapIt work_todo;

};
