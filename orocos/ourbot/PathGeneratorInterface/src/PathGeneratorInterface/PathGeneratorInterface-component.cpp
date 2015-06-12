#include "PathGeneratorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

PathGeneratorInterface::PathGeneratorInterface(std::string const& name) : TaskContext(name, PreOperational),
    _est_pose(3), _est_global_offset(3), _target_abspose(3), _target_relpose(3), _target_abs(false){
  ports()->addPort("est_pose_inport", _est_pose_inport).doc("Estimated pose wrt to initial frame");
  ports()->addPort("est_global_offset_inport", _est_global_offset_inport).doc("Estimated offset of initial frame wrt world frame");
  ports()->addPort("map_obstacles_inport", _map_obstacles_inport).doc("Estimated obstacle data wrt to world frame");
  ports()->addPort("target_abspose_inport", _target_abspose_inport).doc("Target wrt world frame");
  ports()->addPort("target_relpose_inport", _target_relpose_inport).doc("Target wrt initial frame");

  ports()->addPort("ref_pose_path_x_outport", _ref_pose_path_outport[0]).doc("x reference path");
  ports()->addPort("ref_pose_path_y_outport", _ref_pose_path_outport[1]).doc("y reference path");
  ports()->addPort("ref_pose_path_t_outport", _ref_pose_path_outport[2]).doc("theta reference path");

  ports()->addPort("ref_ffw_path_x_outport", _ref_ffw_path_outport[0]).doc("x ffw reference path");
  ports()->addPort("ref_ffw_path_y_outport", _ref_ffw_path_outport[1]).doc("y ffw reference path");
  ports()->addPort("ref_ffw_path_t_outport", _ref_ffw_path_outport[2]).doc("theta ffw reference path");
}

bool PathGeneratorInterface::configureHook(){
  // Get configurator component and get parameters
  TaskContext* configurator = getPeer("configurator");
  if (configurator==NULL){
    log(Error) << "No peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getPathLength = configurator->getOperation("getPathLength");
  if (!getPathLength.ready()){
    log(Error) << "No operation getPathLength of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getControlSampleRate = configurator->getOperation("getControlSampleRate");
  if (!getControlSampleRate.ready()){
    log(Error) << "No operation getControlSampleRate of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getPathUpdSampleRate = configurator->getOperation("getPathUpdSampleRate");
  if (!getPathUpdSampleRate.ready()){
    log(Error) << "No operation getPathUpdSampleRate of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<std::vector<double>(void)> getKinLimitations = configurator->getOperation("getKinLimitations");
  if (!getKinLimitations.ready()){
    log(Error) << "No operation getKinLimitations of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getObsDataLength = configurator->getOperation("getObsDataLength");
  if (!getObsDataLength.ready()){
    log(Error) << "No operation getObsDataLength of peer configurator !" <<endlog();
    return false;
  }
  _path_length          = getPathLength();
  _control_sample_rate  = getControlSampleRate();
  _pathUpd_sample_rate  = getPathUpdSampleRate();
  _kin_limitations      = getKinLimitations();
  _obs_data_length      = getObsDataLength();

  // Reserve required memory and initialize with zeros
  _map_obstacles.resize(_obs_data_length);
  for(int i=0;i<3;i++){
    _ref_pose_path[i].resize(_path_length);
    _ref_ffw_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_path_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_pose_path_outport[i].setDataSample(example);
    _ref_ffw_path_outport[i].setDataSample(example);
  }

  return true;
}

bool PathGeneratorInterface::startHook(){
  if (!_est_pose_inport.connected()){
    log(Warning) << "_est_pose_inport not connected !" <<endlog();
  }
  if (!_est_global_offset_inport.connected()){
    log(Warning) << "_est_global_offset_inport not connected !" <<endlog();
  }
  if (!_target_abspose_inport.connected()){
    log(Warning) << "_target_abspose_inport not connected !" <<endlog();
  }
  if (!_target_relpose_inport.connected()){
    log(Warning) << "_target_relpose_inport not connected !" <<endlog();
  }
  if (!_map_obstacles_inport.connected()){
    log(Warning) << "_map_obstacles not connected !" <<endlog();
  }

  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    return false;
  }
  std::cout << "PathGenerator started !" <<std::endl;
  return true;
}

void PathGeneratorInterface::updateHook(){
  // Read data from estimator
  std::vector<double> pose(3);
  if (_est_pose_inport.connected() && (_est_pose_inport.read(pose) == RTT::NewData)){
    _est_pose = pose;
  }
  if (_est_global_offset_inport.connected() && (_est_global_offset_inport.read(pose) == RTT::NewData)){
    _est_global_offset = pose;
  }
  std::vector<double> obs(_obs_data_length);
  if (_map_obstacles_inport.connected() && (_map_obstacles_inport.read(obs) == RTT::NewData)){
    _map_obstacles = obs;
  }
  if (_target_abspose_inport.connected() && (_target_abspose_inport.read(pose) == RTT::NewData)){
    _target_abspose = pose;
    _target_abs     = true;
  }
  if (_target_relpose_inport.connected() && (_target_relpose_inport.read(pose) == RTT::NewData)){
    _target_relpose = pose;
    _target_abs     = false;
  }

  // Update path
  pathUpdate();

  // Write path
  for (int i=0; i<3; i++){
    _ref_pose_path_outport[i].write(_ref_pose_path[i]);
    _ref_ffw_path_outport[i].write(_ref_ffw_path[i]);
  }

  log(Info) << "Path updated !" <<endlog();
}

int PathGeneratorInterface::getPathLength() { return _path_length; }
int PathGeneratorInterface::getControlSampleRate() { return _control_sample_rate; }
int PathGeneratorInterface::getPathUpdSampleRate() { return _pathUpd_sample_rate; }
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
