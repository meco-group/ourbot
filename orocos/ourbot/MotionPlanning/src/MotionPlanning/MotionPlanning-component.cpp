#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name), _state0(2), _stateT(2){
}

bool MotionPlanning::config(){
  omg::Holonomic* vehicle = new omg::Holonomic();
  _p2p = new omg::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length);
  _obstacles.resize(_p2p->n_obs);
  _ref_pose.resize(_trajectory_length);
  _ref_velocity.resize(_trajectory_length);
  for(int k=0; k<_trajectory_length; k++){
    _ref_pose[k].resize(2);
    _ref_velocity[k].resize(2);
  }
  return true;
}

bool MotionPlanning::initialize(){
  //Michiel load environment
  _p2p->reset();
  _obstacles[0].position[0] = 0.6;
  _obstacles[0].position[1] = -0.3;
  _obstacles[0].velocity[0] = 0.0;
  _obstacles[0].velocity[1] = 0.0;
  _obstacles[0].acceleration[0] = 0.0;
  _obstacles[0].acceleration[1] = 0.0;
  _obstacles[1].position[0] = -0.5;
  _obstacles[1].position[1] = 0.2;
  _obstacles[1].velocity[0] = 0.0;
  _obstacles[1].velocity[1] = 0.0;
  _obstacles[1].acceleration[0] = 0.0;
  _obstacles[1].acceleration[1] = 0.0;
  return true;
}

bool MotionPlanning::trajectoryUpdate(){

  // update motion planning algorithm
  std::cout << _est_pose << ": " << _target_pose << std::endl;
  bool check = _p2p->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, _obstacles, _predict_shift);

  //std::cout << _ref_velocity << std::endl;

  for (int k=0; k<_trajectory_length; k++){
    for (int j=0; j<2; j++){
      _ref_velocity_trajectory[j][k] = _ref_velocity[k][j];
      _ref_pose_trajectory[j][k] = _ref_pose[k][j];
    }
  }
  //if (!check){
  //  _cnt++;
  //}
  //if (_cnt >= _cnt_max){
  //  return false;
  //}
  return true;
}

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
