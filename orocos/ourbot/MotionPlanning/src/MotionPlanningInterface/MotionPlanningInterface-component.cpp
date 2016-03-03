#include "MotionPlanningInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

MotionPlanningInterface::MotionPlanningInterface(std::string const& name) : TaskContext(name, PreOperational),
    _est_pose(3), _target_pose(3), _target_velocity(3),
    _ref_pose_trajectory(3), _ref_velocity_trajectory(3){
  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose");
  ports()->addPort("target_pose_port", _target_pose_port).doc("Target pose");
  ports()->addPort("target_velocity_port", _target_velocity_port).doc("Target pose");

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
  addOperation("setTargetVelocity", &MotionPlanningInterface::setTargetVelocity, this).doc("Set target velocity");
}

void MotionPlanningInterface::setTargetPose(std::vector<double> const& target_pose){
  _target_pose = target_pose;
}

void MotionPlanningInterface::setTargetVelocity(std::vector<double> const& target_velocity){
  _target_velocity = target_velocity;
}

bool MotionPlanningInterface::configureHook(){
  // Compute path length
  _trajectory_length = static_cast<int>(_control_sample_rate/_pathupd_sample_rate);
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

  _update_time = 1./_pathupd_sample_rate;
  _sample_time = 1./_control_sample_rate;

  return true;
}

bool MotionPlanningInterface::startHook(){
  if (!_est_pose_port.connected()){
    log(Warning) << "_est_pose_port not connected !" <<endlog();
  }
  if (!_target_pose_port.connected()){
    log(Warning) << "_target_pose_port not connected !" <<endlog();
  }
  if (!_target_velocity_port.connected()){
    log(Warning) << "_target_velocity_port not connected !" <<endlog();
  }
  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    return false;
  }
  _period = getPeriod();
  _timestamp = TimeService::Instance()->getTicks();
  _init = 0;
  return true;
}

void MotionPlanningInterface::updateHook(){
  TimeService::ticks prev_timestamp = _timestamp;
  _timestamp = TimeService::Instance()->getTicks();
  // read data from estimator
  if (_est_pose_port.connected()){
    _est_pose_port.read(_est_pose);
  }
  if (_target_pose_port.connected()){
    _target_pose_port.read(_target_pose);
  }
  if (_target_velocity_port.connected()){
    _target_velocity_port.read(_target_velocity);
  }
  // update trajectory
  if(!trajectoryUpdate()){
    log(Error) << "Error occured in trajectoryUpdate() !" <<endlog();
  }
  // write trajectory
  for (int i=0; i<3; i++){
    _ref_pose_trajectory_port[i].write(_ref_pose_trajectory[i]);
    _ref_velocity_trajectory_port[i].write(_ref_velocity_trajectory[i]);
  }
  // check timing
  if (_init>2){
    Seconds prev_time_elapsed = TimeService::Instance()->secondsSince( prev_timestamp );
    Seconds time_elapsed = TimeService::Instance()->secondsSince( _timestamp );
    if (time_elapsed > _period*0.9){
      log(Warning) << "PathGenerator: Duration of calculation exceeded 90% of sample period" <<endlog();
    }
    if ((time_elapsed-prev_time_elapsed-_period) > 0.1*_period){
      log(Warning) << "PathGenerator: Jitter exceeded 10% of sample period'" <<endlog();
    }
  }
  else{
    _init++;
  }
}

ORO_CREATE_COMPONENT_LIBRARY()
