#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name){
}

bool MotionPlanning::config(){
  omg::Vehicle* vehicle = new omg::Holonomic();
  if (_ideal_prediction){
    vehicle->setIdealPrediction(true);
  }
  _n_st = vehicle->getNState();
  _n_in = vehicle->getNInput();
  _p2p = new omg::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length_full);
  _ref_pose.resize(_trajectory_length_full);
  _ref_velocity.resize(_trajectory_length_full);
  for(int k=0; k<_trajectory_length_full; k++){
    _ref_pose[k].resize(_n_st);
    _ref_velocity[k].resize(_n_in);
  }
  _obstacles.resize(_p2p->n_obs);
  _n_obs = _p2p->n_obs;
  std::cout << "Loaded motion planning problem with " << _n_obs << " obstacles." << std::endl;
  return true;
}

bool MotionPlanning::initialize(){
  _p2p->reset();
  _p2p->resetTime();
  return true;
}

bool MotionPlanning::trajectoryUpdate(){
  // get obstacles
  std::vector<omg::obstacle_t> obstacles(_n_obs);
  getObstacles(obstacles);
  // update motion planning algorithm
  bool check = _p2p->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, obstacles, _predict_shift);
  for (int k=0; k<_trajectory_length; k++){
    for (int j=0; j<_n_st; j++){
      _ref_pose_trajectory[j][k] = _ref_pose[k][j];
    }
    for (int j=0; j<_n_in; j++){
      _ref_velocity_trajectory[j][k] = _ref_velocity[k][j];
    }
  }
  // subsample for transmission
  for (int k=0; k<_trajectory_length_tx; k++){
    for (int j=0; j<2; j++){
      _ref_pose_trajectory_ss[j][k] = _ref_pose[k*_tx_subsample][j];
    }
  }
  return check;
}

void MotionPlanning::getObstacles(std::vector<omg::obstacle_t>& obstacles){
  for (int k=0; k<_n_obs; k++){
    obstacles[k].position = _obstacles[k].position;
    obstacles[k].velocity = _obstacles[k].velocity;
    obstacles[k].acceleration = _obstacles[k].acceleration;
    obstacles[k].checkpoints = _obstacles[k].checkpoints;
    obstacles[k].radii = _obstacles[k].radii;
    obstacles[k].avoid = _obstacles[k].avoid;
  }
}

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
