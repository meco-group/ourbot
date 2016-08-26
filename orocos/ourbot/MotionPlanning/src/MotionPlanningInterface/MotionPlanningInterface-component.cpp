#define DEBUG

#include "MotionPlanningInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


MotionPlanningInterface::MotionPlanningInterface(std::string const& name) : TaskContext(name, PreOperational),
    _predict_shift(0), _est_pose(3), _target_pose(3),
    _ref_pose_trajectory(3), _ref_velocity_trajectory(3){
  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose");
  ports()->addPort("target_pose_port", _target_pose_port).doc("Target pose");

  ports()->addEventPort("predict_shift_port", _predict_shift_port).doc("Trigger for motion planning");

  ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
  ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
  ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");

  ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("pathupd_sample_rate", _pathupd_sample_rate).doc("Frequency to update the path");
  addProperty("horizon_time", _horizon_time).doc("Horizon to compute motion trajectory");

  addOperation("setTargetPose", &MotionPlanningInterface::setTargetPose, this).doc("Set target pose");
}

void MotionPlanningInterface::setTargetPose(double target_x, double target_y, double target_t){
  _target_pose[0] = target_x;
  _target_pose[1] = target_y;
  _target_pose[2] = target_t;
  std::cout << "target set: " << _target_pose[0] <<","<<_target_pose[1]<<","<<_target_pose[2]<<std::endl;
}

bool MotionPlanningInterface::configureHook(){
  _update_time = 1./_pathupd_sample_rate;
  _sample_time = 1./_control_sample_rate;
  // Compute path length
  _update_length = int(_control_sample_rate/_pathupd_sample_rate);
  _trajectory_length = 3*_update_length;
  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _ref_pose_trajectory[i].resize(_trajectory_length);
    _ref_velocity_trajectory[i].resize(_trajectory_length);
  }
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_trajectory_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_pose_trajectory_port[i].setDataSample(example);
    _ref_velocity_trajectory_port[i].setDataSample(example);
  }
  if (!config()){
    log(Error) << "Error occured in configure() !" << endlog();
    return false;
  }
  return true;
}

bool MotionPlanningInterface::startHook(){
  if (!_est_pose_port.connected()){
    log(Warning) << "_est_pose_port not connected !" <<endlog();
  }
  if (!_target_pose_port.connected()){
    log(Warning) << "_target_pose_port not connected !" <<endlog();
  }
  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    return false;
  }
  return true;
}

void MotionPlanningInterface::updateHook(){
  #ifdef DEBUG
  std::cout << "started mp update" << std::endl;
  log(Info) << "started mp update" << endlog();
  #endif

  _predict_shift_port.read(_predict_shift);
  _timestamp = TimeService::Instance()->getTicks();
  // read data from estimator
  if (_est_pose_port.connected()){
    _est_pose_port.read(_est_pose);
  }
  if (_target_pose_port.connected()){
  _target_pose_port.read(_target_pose);
  }
  // update trajectory
  if(!trajectoryUpdate()){
    log(Error) << "Error occured in trajectoryUpdate() !" <<endlog();
    error();
  }
  // write trajectory
  for (int i=0; i<3; i++){
    _ref_pose_trajectory_port[i].write(_ref_pose_trajectory[i]);
    _ref_velocity_trajectory_port[i].write(_ref_velocity_trajectory[i]);
  }
  // check timing
  Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  if (time_elapsed > _update_time*0.9){
    log(Warning) << "MotionPlanning: Duration of calculation exceeded 90% of update_time" <<endlog();
  }
  #ifdef DEBUG
  std::cout << "ended mp update in "<<time_elapsed<< " s" << std::endl;
  log(Info) << "ended mp update in " << time_elapsed<< " s" << endlog();
  #endif
}

ORO_CREATE_COMPONENT_LIBRARY()
