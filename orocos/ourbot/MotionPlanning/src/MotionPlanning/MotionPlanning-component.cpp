#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name),
_mp(_update_time, _sample_time, _horizon_time){
  _mp.setIdealUpdate(true); // because we do not have an update from our state yet
}

bool MotionPlanning::initialize(){
  _obstacles.resize(_mp.n_obs);
  _est_pose[0] = -1.5;
  _est_pose[1] = -1.5;
  _target_pose[0] = 2.;
  _target_pose[1] = 2.;
  return true;
}

bool MotionPlanning::trajectoryUpdate(){
  // update motion planning algorithm
  TimeService::ticks timestamp = TimeService::Instance()->getTicks();
  bool check = _mp.update(_est_pose, _target_pose, _target_velocity, _ref_pose_trajectory, _ref_velocity_trajectory, _obstacles);
  Seconds time_elapsed = TimeService::Instance()->secondsSince(timestamp);
  cout << "update took " << time_elapsed << "s" << endl;
  if (!check){
    _cnt++;
  }
  if (_cnt >= _cnt_max){
    return false;
  }
  return true;
}

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
