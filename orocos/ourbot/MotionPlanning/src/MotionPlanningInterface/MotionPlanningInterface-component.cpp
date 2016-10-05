#define DEBUG

#include "MotionPlanningInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

MotionPlanningInterface::MotionPlanningInterface(std::string const& name) : TaskContext(name, PreOperational),
    _got_target(false), _enable(false), _predict_shift(0), _est_pose(3), _target_pose(3),
    _ref_pose_trajectory(3), _ref_velocity_trajectory(3), _n_obs(0){
  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose");
  ports()->addPort("target_pose_port", _target_pose_port).doc("Target pose");
  ports()->addPort("obstacle_port", _obstacle_port).doc("Detected obstacles");
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
  addProperty("ideal_prediction", _ideal_prediction).doc("Use prediction based on computed trajectories and not on state estimation");

  addOperation("setTargetPose", &MotionPlanningInterface::setTargetPose, this).doc("Set target pose");
  addOperation("gotTarget", &MotionPlanningInterface::gotTarget, this).doc("Do we have a target?");
  addOperation("enable", &MotionPlanningInterface::enable, this).doc("Enable Motion Planning");
}

void MotionPlanningInterface::setTargetPose(double target_x, double target_y, double target_t){
  _target_pose[0] = target_x;
  _target_pose[1] = target_y;
  _target_pose[2] = target_t;
  std::cout << "target set: (" << _target_pose[0] <<","<<_target_pose[1]<<","<<_target_pose[2]<<")"<<std::endl;
  _got_target = true;
}

bool MotionPlanningInterface::gotTarget(){
  if (_target_pose_port.connected()){
    if(_target_pose_port.read(_target_pose) == RTT::NewData){
      std::cout << "got target: (" << _target_pose[0] << "," << _target_pose[1] << "," << _target_pose[2] << ")" << std::endl;
      _got_target = true;
    }
  }
  return _got_target;
}

void MotionPlanningInterface::enable(){
  _enable = true;
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
  // init obstacle data
  _obstacle_data.resize(_n_obs*5);
  initObstacles();
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
  _got_target = false;
  _enable = false;
  return true;
}

void MotionPlanningInterface::updateHook(){
  if (!_enable){
    return;
  }
  #ifdef DEBUG
  std::cout << "started mp update" << std::endl;
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
  // read obstacle data
  if (_obstacle_port.connected()){
    _obstacle_port.read(_obstacle_data);
  }
  computeObstacles();
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
  std::cout << "ended mp update in " << time_elapsed << " s" << std::endl;
  #endif
}

void MotionPlanningInterface::initObstacles(){
  for (int k=0; k<_n_obs; k++){
    _obstacles[k].position.resize(2, 0.0);
    _obstacles[k].velocity.resize(2, 0.0);
    _obstacles[k].acceleration.resize(2, 0.0);
    _obstacles[k].checkpoints.resize(2*4, 0.0);
    _obstacles[k].radii.resize(4, 0.0);
  }
}

void MotionPlanningInterface::computeObstacles(){
  for (int k=0; k<_n_obs; k++){
    if (_obstacle_data[5*k] == -100){
      _obstacles[k].avoid = false;
    } else {
      _obstacles[k].avoid = true;
      _obstacles[k].position[0] = _obstacle_data[5*k+0];
      _obstacles[k].position[1] = _obstacle_data[5*k+1];
      if (_obstacle_data[5*k+3] == -100){
        // circle
        for (int i=0; i<4; i++){
          _obstacles[k].radii[i] = _obstacle_data[5*k+2];
          for (int j=0; j<2; j++){
            _obstacles[k].checkpoints[2*i+j] = _obstacle_data[5*k+j];
          }
        }
      } else {
        // rectangle
        double orientation = (M_PI/180.)*_obstacle_data[5*k+2];
        double width = _obstacle_data[5*k+3];
        double height = _obstacle_data[5*k+4];
        for (int i=0; i<4; i++){
          _obstacles[k].radii[i] = 0.001;
        }
        _obstacles[k].checkpoints[0] = 0.5*width*cos(orientation) - 0.5*height*sin(orientation);
        _obstacles[k].checkpoints[1] = 0.5*width*sin(orientation) + 0.5*height*cos(orientation);
        _obstacles[k].checkpoints[2] = 0.5*width*cos(orientation) + 0.5*height*sin(orientation);
        _obstacles[k].checkpoints[3] = 0.5*width*sin(orientation) - 0.5*height*cos(orientation);
        _obstacles[k].checkpoints[4] = -0.5*width*cos(orientation) + 0.5*height*sin(orientation);
        _obstacles[k].checkpoints[5] = -0.5*width*sin(orientation) - 0.5*height*cos(orientation);
        _obstacles[k].checkpoints[6] = -0.5*width*cos(orientation) - 0.5*height*sin(orientation);
        _obstacles[k].checkpoints[7] = -0.5*width*sin(orientation) + 0.5*height*cos(orientation);
      }
    }
  }
}


ORO_CREATE_COMPONENT_LIBRARY()
