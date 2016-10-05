#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name), _state0(2), _stateT(2){
}

bool MotionPlanning::config(){
  omg::Vehicle* vehicle = new omg::Holonomic();
  if (_ideal_prediction){
    vehicle->setIdealPrediction(true);
  }
  _p2p = new omg::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length);
  _obstacles.resize(_p2p->n_obs);
  _ref_pose.resize(_trajectory_length);
  _ref_velocity.resize(_trajectory_length);
  for(int k=0; k<_trajectory_length; k++){
    _ref_pose[k].resize(2);
    _ref_velocity[k].resize(2);
  }
  _n_obs = _p2p->n_obs;
  return true;
}

bool MotionPlanning::initialize(){
  _p2p->reset();
  return true;
}

bool MotionPlanning::trajectoryUpdate(){
  std::cout << "est_pose: " << _est_pose[0] << "," << _est_pose[1] << "," << _est_pose[2] << std::endl;
  // get obstacles
  std::vector<omg::obstacle_t> obstacles(_n_obs);
  getObstacles(obstacles);
  // update motion planning algorithm
  bool check = _p2p->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, obstacles, _predict_shift);
  for (int k=0; k<_trajectory_length; k++){
    for (int j=0; j<2; j++){
      _ref_velocity_trajectory[j][k] = _ref_velocity[k][j];
      _ref_pose_trajectory[j][k] = _ref_pose[k][j];
    }
  }
  if (!check){
    _cnt++;
  }
  if (_cnt >= _cnt_max){
    return false;
  }
  return true;
}

void MotionPlanning::getObstacles(std::vector<omg::obstacle_t>& obstacles){
  for (int k=0; k<_n_obs; k++){
    obstacles[k].position = _obstacles[k].position;
    obstacles[k].velocity = _obstacles[k].velocity;
    obstacles[k].acceleration = _obstacles[k].acceleration;
    obstacles[k].checkpoints = _obstacles[k].checkpoints;
    obstacles[k].radii = _obstacles[k].radii;
    obstacles[k].avoid = _obstacles[k].avoid;
    // std::cout << "obstacle " << k << ":" << std::endl;
    // std::cout << "----------" << std::endl;
    // std::cout << "pos: " << obstacles[k].position << std::endl;
    // std::cout << "vel: " << obstacles[k].velocity << std::endl;
    // std::cout << "acc: " << obstacles[k].acceleration << std::endl;
    // std::cout << "chckp: " << obstacles[k].checkpoints << std::endl;
    // std::cout << "radii: " << obstacles[k].radii << std::endl;
    // std::cout << "avoid: " << obstacles[k].avoid << std::endl;
  }
}

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
