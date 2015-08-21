#include "PathGeneratorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

PathGeneratorInterface::PathGeneratorInterface(std::string const& name) : TaskContext(name, PreOperational),
    _est_pose(3), _est_global_offset(3), _target_abspose(3), _target_relpose(3), _target_abs(false){
  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose wrt to initial frame");
  ports()->addPort("est_global_offset_port", _est_global_offset_port).doc("Estimated offset of initial frame wrt world frame");
  ports()->addPort("map_obstacles_port", _map_obstacles_port).doc("Estimated obstacle data wrt to world frame");
  ports()->addPort("target_abspose_port", _target_abspose_port).doc("Target wrt world frame");
  ports()->addPort("target_relpose_port", _target_relpose_port).doc("Target wrt initial frame");

  ports()->addPort("ref_pose_path_x_port", _ref_pose_path_port[0]).doc("x reference path");
  ports()->addPort("ref_pose_path_y_port", _ref_pose_path_port[1]).doc("y reference path");
  ports()->addPort("ref_pose_path_t_port", _ref_pose_path_port[2]).doc("theta reference path");

  ports()->addPort("ref_ffw_path_x_port", _ref_ffw_path_port[0]).doc("x ffw reference path");
  ports()->addPort("ref_ffw_path_y_port", _ref_ffw_path_port[1]).doc("y ffw reference path");
  ports()->addPort("ref_ffw_path_t_port", _ref_ffw_path_port[2]).doc("theta ffw reference path");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("pathupd_sample_rate", _pathupd_sample_rate).doc("Frequency to update the path");
  addProperty("obs_data_length", _obs_data_length).doc("Length of obstacle data");
  addProperty("kin_limitations", _kin_limitations).doc("Kinematic limitations (vmax, vmin, amax, amin)");

  addOperation("writeSample",&PathGeneratorInterface::writeSample, this).doc("Set data sample on output ports");
}

void PathGeneratorInterface::writeSample(){
  std::vector<double> example(_path_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_pose_path_port[i].write(example);
    _ref_ffw_path_port[i].write(example);
  }
}

bool PathGeneratorInterface::configureHook(){
  // Compute path length
  _path_length = static_cast<int>(_control_sample_rate/_pathupd_sample_rate);
  // Reserve required memory and initialize with zeros
  _map_obstacles.resize(_obs_data_length);
  for(int i=0;i<3;i++){
    _ref_pose_path[i].resize(_path_length);
    _ref_ffw_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_path_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_pose_path_port[i].setDataSample(example);
    _ref_ffw_path_port[i].setDataSample(example);
  }
  return true;
}

bool PathGeneratorInterface::startHook(){
  if (!_est_pose_port.connected()){
    log(Warning) << "_est_pose_port not connected !" <<endlog();
  }
  if (!_est_global_offset_port.connected()){
    log(Warning) << "_est_global_offset_port not connected !" <<endlog();
  }
  if (!_target_abspose_port.connected()){
    log(Warning) << "_target_abspose_port not connected !" <<endlog();
  }
  if (!_target_relpose_port.connected()){
    log(Warning) << "_target_relpose_port not connected !" <<endlog();
  }
  if (!_map_obstacles_port.connected()){
    log(Warning) << "_map_obstacles not connected !" <<endlog();
  }

  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    return false;
  }
  _period = getPeriod();
  _timestamp = TimeService::Instance()->getTicks();
  _init = 0;

  std::cout << "PathGenerator started !" <<std::endl;
  return true;
}

void PathGeneratorInterface::updateHook(){
  TimeService::ticks prev_timestamp = _timestamp;
  _timestamp = TimeService::Instance()->getTicks();
  // Read data from estimator
  std::vector<double> pose(3);
  if (_est_pose_port.connected() && (_est_pose_port.read(pose) == RTT::NewData)){
    _est_pose = pose;
  }
  if (_est_global_offset_port.connected() && (_est_global_offset_port.read(pose) == RTT::NewData)){
    _est_global_offset = pose;
  }
  std::vector<double> obs(_obs_data_length);
  if (_map_obstacles_port.connected() && (_map_obstacles_port.read(obs) == RTT::NewData)){
    _map_obstacles = obs;
  }
  if (_target_abspose_port.connected() && (_target_abspose_port.read(pose) == RTT::NewData)){
    _target_abspose = pose;
    _target_abs     = true;
  }
  if (_target_relpose_port.connected() && (_target_relpose_port.read(pose) == RTT::NewData)){
    _target_relpose = pose;
    _target_abs     = false;
  }

  // Update path
  pathUpdate();

  // Write path
  for (int i=0; i<3; i++){
    _ref_pose_path_port[i].write(_ref_pose_path[i]);
    _ref_ffw_path_port[i].write(_ref_ffw_path[i]);
  }

  // Check timing
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

void PathGeneratorInterface::stopHook() {
  std::cout << "Pathgenerator stopped !" <<std::endl;
}

int PathGeneratorInterface::getPathLength() { return _path_length; }
double PathGeneratorInterface::getControlSampleRate() { return _control_sample_rate; }
double PathGeneratorInterface::getPathUpdSampleRate() { return _pathupd_sample_rate; }
std::vector<double> PathGeneratorInterface::getKinLimitations() { return _kin_limitations; }
std::vector<double> PathGeneratorInterface::getEstPose() { return _est_pose; }
std::vector<double> PathGeneratorInterface::getTargetPose(){
  if (_target_abs){
    std::vector<double> target_relpose(3);
    for (int i=0; i<3; i++){
      target_relpose.at(i) = _target_abspose.at(i) - _est_global_offset.at(i);
    }
    return target_relpose;
  }
  else{
    return _target_relpose;
  }
}
std::vector<double> PathGeneratorInterface::getObstacleData(){
  // Should be further implemented... -> should be substracted with _est_global_offset
  return _map_obstacles;
}
void PathGeneratorInterface::setRefPosePath(std::vector<double> const& ref_pose_path_x, std::vector<double> const& ref_pose_path_y, std::vector<double> const& ref_pose_path_t){
 _ref_pose_path[0] = ref_pose_path_x;
 _ref_pose_path[1] = ref_pose_path_y;
 _ref_pose_path[2] = ref_pose_path_t;
}
void PathGeneratorInterface::setRefFfwPath(std::vector<double> const& ref_ffw_path_x, std::vector<double> const& ref_ffw_path_y, std::vector<double> const& ref_ffw_path_t){
 _ref_ffw_path[0] = ref_ffw_path_x;
 _ref_ffw_path[1] = ref_ffw_path_y;
 _ref_ffw_path[2] = ref_ffw_path_t;
}

ORO_CREATE_COMPONENT_LIBRARY()
