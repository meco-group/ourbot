#define DEBUG

#include "MotionPlanningInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

MotionPlanningInterface::MotionPlanningInterface(std::string const& name) : TaskContext(name, PreOperational),
    _got_target(false), _enable(false), _mp_trigger_data(4), _failure_cnt(0), _first_iteration(true), _valid(false), _predict_shift(0), _est_pose(3),
    _target_pose(3), _ref_pose_trajectory(3), _ref_velocity_trajectory(3), _ref_pose_trajectory_ss(2), _n_obs(0){

  ports()->addPort("target_pose_port", _target_pose_port).doc("Target pose");
  ports()->addPort("obstacle_port", _obstacle_port).doc("Detected obstacles");
  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose wrt to initial frame");
  ports()->addEventPort("mp_trigger_port", _mp_trigger_port).doc("Trigger for motion planning: is composed of current estimated pose and start index of internal reference input vector");
  ports()->addPort("robobs_pose_port", _robobs_pose_port).doc("Pose information of a robot obstacle");
  ports()->addPort("robobs_velocity_port", _robobs_velocity_port).doc("Velocity information of a robot obstacle");

  ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
  ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
  ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");

  ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

  ports()->addPort("ref_pose_trajectory_ss_x_port", _ref_pose_trajectory_ss_port[0]).doc("subsampled x reference trajectory for whole horizon");
  ports()->addPort("ref_pose_trajectory_ss_y_port", _ref_pose_trajectory_ss_port[1]).doc("subsampled y reference trajectory for whole horizon");

  ports()->addPort("motion_time_port", _motion_time_port).doc("Motion time of computed motion trajectory");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("pathupd_sample_rate", _pathupd_sample_rate).doc("Frequency to update the path");
  addProperty("horizon_time", _horizon_time).doc("Horizon to compute motion trajectory");
  addProperty("max_computation_periods", _max_computation_periods).doc("Maximum allowed number of trajectory update periods to make trajectory computations");
  addProperty("ideal_prediction", _ideal_prediction).doc("Use prediction based on computed trajectories and not on state estimation");
  addProperty("maximum_failures", _maximum_failures).doc("Maximum allowed consecutive failures");
  addProperty("ref_tx_subsample", _tx_subsample).doc("Subsamples for transmitted reference trajectories");
  addProperty("target_detection", _target_detection).doc("Target reached detection?");

  addOperation("setTargetPose", &MotionPlanningInterface::setTargetPose, this).doc("Set target pose");
  addOperation("gotTarget", &MotionPlanningInterface::gotTarget, this).doc("Do we have a target?");
  addOperation("targetReached", &MotionPlanningInterface::targetReached, this).doc("Did we reach the current target?");
  addOperation("valid", &MotionPlanningInterface::valid, this).doc("Valid trajectories computed?");
  addOperation("enable", &MotionPlanningInterface::enable, this).doc("Enable Motion Planning");
  addOperation("disable", &MotionPlanningInterface::disable, this).doc("Disable Motion Planning");
  addOperation("setConfiguration", &MotionPlanningInterface::setConfiguration, this);
  addOperation("zeroOrientation", &MotionPlanningInterface::zeroOrientation, this).doc("Should orientation kept zero?");
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
      setTargetPose(_target_pose[0], _target_pose[1], _target_pose[2]);
    }
  }
  return _got_target;
}

void MotionPlanningInterface::enable(){
  if (!_enable){
    initialize();
    std::cout << "re-init" << std::endl;
  }
  _enable = true;
  _first_iteration = true;
  _omega = 0.;
}

void MotionPlanningInterface::disable(){
  _enable = false;
  _got_target = false;
  _valid = false;
  _omega = 0.;
  std::cout << "disabling...." << std::endl;
}

bool MotionPlanningInterface::valid(){
  return _valid;
}

bool MotionPlanningInterface::configureHook(){
  _update_time = 1./_pathupd_sample_rate;
  _sample_time = 1./_control_sample_rate;
  // compute path length
  _update_length = int(_control_sample_rate/_pathupd_sample_rate);
  _trajectory_length = (_max_computation_periods+1)*_update_length; // +1 to make prediction possible
  _trajectory_length_full = int((_horizon_time-_update_time)/_sample_time);
  _trajectory_length_tx = 0;
  for (int i=0; i<_trajectory_length_full; i+=_tx_subsample){
    _trajectory_length_tx++;
  }
  // reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _ref_pose_trajectory[i].resize(_trajectory_length);
    _ref_velocity_trajectory[i].resize(_trajectory_length);
    if (i < 2){
      _ref_pose_trajectory_ss[i].resize(_trajectory_length_tx);
    }
  }
  // show example data sample to ports to make data flow real-time
  std::vector<double> example(_trajectory_length, 0.0);
  std::vector<double> example2(_trajectory_length_tx, 0.0);
  for(int i=0; i<3; i++){
    _ref_pose_trajectory_port[i].setDataSample(example);
    _ref_velocity_trajectory_port[i].setDataSample(example);
    if (i < 2){
      _ref_pose_trajectory_ss_port[i].setDataSample(example2);
    }
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
  if (!_target_pose_port.connected()){
    log(Warning) << "_target_pose_port not connected !" <<endlog();
  }
  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    return false;
  }
  _got_target = false;
  _enable = false;
  _omega = 0.;
  return true;
}

bool MotionPlanningInterface::targetReached(){
  double target_dist = 0.;
  double input_norm = 0.;
  for (int i=0; i<3; i++){
    target_dist += pow(_target_pose[i] - _est_pose[i], 2);
    input_norm += pow(_ref_velocity_trajectory[i][_predict_shift], 2);
  }
  target_dist = sqrt(target_dist);
  input_norm = sqrt(input_norm);
  if (target_dist < 5e-2 && input_norm < 1e-2){
    return true;
  }
  return false;
}

double MotionPlanningInterface::getMotionTime(){
  return _horizon_time; //dummy implementation
}

void MotionPlanningInterface::updateHook(){
  if (!_enable){
    return;
  }
  #ifdef DEBUG
  std::cout << "started mp update" << std::endl;
  #endif
  _timestamp = TimeService::Instance()->getTicks();
  // read data from reference
  _mp_trigger_port.read(_mp_trigger_data);
  for (int k=0; k<3; k++){
    _est_pose[k] = _mp_trigger_data[k];
  }
  _predict_shift = static_cast<int>(_mp_trigger_data[3]);
  if (_target_pose_port.connected()){
    _target_pose_port.read(_target_pose);
  }
  // read obstacle data
  if (_obstacle_port.connected()){
    _obstacle_port.read(_obstacle_data);
  }
  computeObstacles();
  if (_target_detection){
    if (targetReached()){
      std::cout << "target reached!" << std::endl;
      disable();
      return;
    }
  }
  // update trajectory
  _valid = trajectoryUpdate();
  if (!_valid){
    _failure_cnt++;
    if (_failure_cnt >= _maximum_failures || _first_iteration){
      log(Error) << "MotionPlanning could not find trajectory." << endlog();
      disable();
    }
  } else {
    _failure_cnt = 0;
  }
  _first_iteration = false;
  // write trajectory
  for (int i=0; i<3; i++){
    _ref_pose_trajectory_port[i].write(_ref_pose_trajectory[i]);
    _ref_velocity_trajectory_port[i].write(_ref_velocity_trajectory[i]);
    if (i < 2){
      _ref_pose_trajectory_ss_port[i].write(_ref_pose_trajectory_ss[i]);
    }
  }
  // write motion time
  _motion_time_port.write(getMotionTime());
  // check timing
  Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  #ifdef DEBUG
  std::cout << "ended mp update in " << time_elapsed << " s" << std::endl;
  #endif
}

std::vector<double> MotionPlanningInterface::setConfiguration(){
  std::vector<double> est_pose(3, 0);
  _est_pose_port.read(est_pose);
  est_pose[2] = 0.0;
  return est_pose;
}

bool MotionPlanningInterface::zeroOrientation(){
  return true;
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
  std::vector<double> obst_pose(3);
  std::vector<double> obst_velocity(3);
  double orientation, width, height;
  for (int k=0; k<_n_obs; k++){
    if (_obstacle_data[5*k] == -100){
      _obstacles[k].avoid = false;
    } else {
      _obstacles[k].avoid = true;
      _obstacles[k].position[0] = _obstacle_data[5*k+0];
      _obstacles[k].position[1] = _obstacle_data[5*k+1];
      _obstacles[k].velocity[0] = 0.;
      _obstacles[k].velocity[1] = 0.;
      _obstacles[k].acceleration[0] = 0.;
      _obstacles[k].acceleration[1] = 0.;
      // set checkpoints
      for (int k=0; k<_n_obs; k++){
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
          orientation = (M_PI/180.)*_obstacle_data[5*k+2];
          width = _obstacle_data[5*k+3];
          height = _obstacle_data[5*k+4];
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
  if (_robobs_pose_port.read(obst_pose) == RTT::NewData){
    _obstacles[_n_obs-1].avoid = true;
    _obstacles[_n_obs-1].position[0] = obst_pose[0];
    _obstacles[_n_obs-1].position[1] = obst_pose[1];
    orientation = obst_pose[2];
    if (_robobs_velocity_port.read(obst_velocity) == RTT::NewData){
        _obstacles[_n_obs-1].velocity[0] = obst_velocity[0];
        _obstacles[_n_obs-1].velocity[1] = obst_velocity[1];
    }
    else {
        _obstacles[_n_obs-1].velocity[0] = 0.;
        _obstacles[_n_obs-1].velocity[1] = 0.;
    }
    _obstacles[_n_obs-1].acceleration[0] = 0.;
    _obstacles[_n_obs-1].acceleration[1] = 0.;
    width = 0.55;
    height = 0.4;
    for (int i=0; i<4; i++){
      _obstacles[_n_obs-1].radii[i] = 0.001;
    }
    _obstacles[_n_obs-1].checkpoints[0] = 0.5*width*cos(orientation) - 0.5*height*sin(orientation);
    _obstacles[_n_obs-1].checkpoints[1] = 0.5*width*sin(orientation) + 0.5*height*cos(orientation);
    _obstacles[_n_obs-1].checkpoints[2] = 0.5*width*cos(orientation) + 0.5*height*sin(orientation);
    _obstacles[_n_obs-1].checkpoints[3] = 0.5*width*sin(orientation) - 0.5*height*cos(orientation);
    _obstacles[_n_obs-1].checkpoints[4] = -0.5*width*cos(orientation) + 0.5*height*sin(orientation);
    _obstacles[_n_obs-1].checkpoints[5] = -0.5*width*sin(orientation) - 0.5*height*cos(orientation);
    _obstacles[_n_obs-1].checkpoints[6] = -0.5*width*cos(orientation) - 0.5*height*sin(orientation);
    _obstacles[_n_obs-1].checkpoints[7] = -0.5*width*sin(orientation) + 0.5*height*cos(orientation);
    // std::cout << "obst pos: " << _obstacles[_n_obs-1].position[0] << "," << _obstacles[_n_obs-1].position[0] << std::endl;
    // std::cout << "obst ori: " << orientation << std::endl;
    // std::cout << "obst vel: " << _obstacles[_n_obs-1].velocity[0] << "," << _obstacles[_n_obs-1].velocity[1] << std::endl;
  }
}

void MotionPlanningInterface::sample_spline_trajs(){
  return;
}

ORO_CREATE_COMPONENT_LIBRARY()

