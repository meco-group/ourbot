#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name), _state0(2), _stateT(2){
}

bool MotionPlanning::config(){
  omg::Holonomic* vehicle = new omg::Holonomic();
  //vehicle->setIdealPrediction(true);// because we do not have an update from our state yet
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
  _p2p->reset();
  _obstacles[0].position[0] = 0.;
  _obstacles[0].position[1] = -0.58;
  _obstacles[0].velocity[0] = 0.0;
  _obstacles[0].velocity[1] = 0.0;
  _obstacles[0].acceleration[0] = 0.0;
  _obstacles[0].acceleration[1] = 0.0;
  _obstacles[1].position[0] = -0.76;
  _obstacles[1].position[1] = 0.7925;
  _obstacles[1].velocity[0] = 0.0;
  _obstacles[1].velocity[1] = 0.0;
  _obstacles[1].acceleration[0] = 0.0;
  _obstacles[1].acceleration[1] = 0.0;
  _obstacles[2].position[0] = 0.51;
  _obstacles[2].position[1] = 0.45;
  _obstacles[2].velocity[0] = 0.0;
  _obstacles[2].velocity[1] = 0.0;
  _obstacles[2].acceleration[0] = 0.0;
  _obstacles[2].acceleration[1] = 0.0;
  _obstacles[3].position[0] = 0.;
  _obstacles[3].position[1] = 0.5;
  _obstacles[3].velocity[0] = 0.0;
  _obstacles[3].velocity[1] = 0.0;
  _obstacles[3].acceleration[0] = 0.0;
  _obstacles[3].acceleration[1] = 0.0;
  return true;
}

bool MotionPlanning::trajectoryUpdate(){

  // update motion planning algorithm
  bool check = _p2p->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, _obstacles, _predict_shift);
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

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
