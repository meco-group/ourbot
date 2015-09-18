#include "DistributedPathGeneratorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedPathGeneratorInterface::DistributedPathGeneratorInterface(std::string const& name) : PathGeneratorInterface(name),
    _est_poseL(3), _est_poseR(3){
  ports()->addPort("est_poseL_port",_est_poseL_port).doc("Estimated pose of neighbour via left going signal");
  ports()->addPort("est_poseR_port",_est_poseR_port).doc("Estimated pose of neighbour via right going signal");

  ports()->addPort("ref_inL_path_x_port",_ref_inL_path_port[0]).doc("x reference path of neighbour via left going signal");
  ports()->addPort("ref_inL_path_y_port",_ref_inL_path_port[1]).doc("y reference path of neighbour via left going signal");
  ports()->addPort("ref_inL_path_t_port",_ref_inL_path_port[2]).doc("theta reference path of neighbour via left going signal");

  ports()->addPort("ref_inR_path_x_port",_ref_inR_path_port[0]).doc("x reference path of neighbour via right going signal");
  ports()->addPort("ref_inR_path_y_port",_ref_inR_path_port[1]).doc("y reference path of neighbour via right going signal");
  ports()->addPort("ref_inR_path_t_port",_ref_inR_path_port[2]).doc("theta reference path of neighbour via right going signal");

  ports()->addPort("ref_relposeL_path_x_port", _ref_relposeL_path_port[0]).doc("x relpose reference path wrt neighbour via left going signal");
  ports()->addPort("ref_relposeL_path_y_port", _ref_relposeL_path_port[1]).doc("y relpose reference path wrt neighbour via left going signal");
  ports()->addPort("ref_relposeL_path_t_port", _ref_relposeL_path_port[2]).doc("theta relpose reference path wrt neighbour via left going signal");

  ports()->addPort("ref_relposeR_path_x_port", _ref_relposeR_path_port[0]).doc("x relpose reference path wrt neighbour via right going signal");
  ports()->addPort("ref_relposeR_path_y_port", _ref_relposeR_path_port[1]).doc("y relpose reference path wrt neighbour via right going signal");
  ports()->addPort("ref_relposeR_path_t_port", _ref_relposeR_path_port[2]).doc("theta relpose reference path wrt neighbour via right going signal");
}

bool DistributedPathGeneratorInterface::configureHook(){
  bool check = PathGeneratorInterface::configureHook();

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _ref_relposeL_path[i].resize(_path_length);
    _ref_relposeR_path[i].resize(_path_length);
    _ref_inL_path[i].resize(_path_length);
    _ref_inR_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_path_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_relposeL_path_port[i].setDataSample(example);
    _ref_relposeR_path_port[i].setDataSample(example);
  }

  return check;
}

bool DistributedPathGeneratorInterface::startHook(){
  // Check if input ports are connected
  if (!_est_poseL_port.connected()){
    log(Warning) << "_est_poseL_port not connected !" <<endlog();
  }
  if (!_est_poseR_port.connected()){
    log(Warning) << "_est_poseR_port not connected !" <<endlog();
  }
  for (int i=0; i<3; i++){
    if (!_ref_inL_path_port[i].connected()){
      log(Warning) << "_ref_inL_port "<< i << " not connected !" <<endlog();
    }
    if (!_ref_inR_path_port[i].connected()){
      log(Warning) << "_ref_inR_port "<< i << " not connected !" <<endlog();
    }
  }

  return PathGeneratorInterface::startHook();
}

void DistributedPathGeneratorInterface::updateHook(){
  // Read estimated pose
  std::vector<double> pose(3);
  if (_est_poseL_port.connected() && (_est_poseL_port.read(pose) == RTT::NewData)){
    _est_poseL = pose;
  }
  if (_est_poseR_port.connected() && (_est_poseR_port.read(pose) == RTT::NewData)){
    _est_poseL = pose;
  }

  // Read paths from neighbours
  std::vector<double> path(_path_length);
  for (int i=0; i<3; i++){
    if (_ref_inL_path_port[i].connected() && (_ref_inL_path_port[i].read(path) == RTT::NewData)){
      _ref_inL_path[i] = path;
    }
    if (_ref_inR_path_port[i].connected() && (_ref_inR_path_port[i].read(path) == RTT::NewData)){
      _ref_inR_path[i] = path;
    }
  }

  PathGeneratorInterface::updateHook();

  // Write path
  for (int i=0; i<3; i++){
    _ref_relposeL_path_port[i].write(_ref_relposeL_path[i]);
    _ref_relposeL_path_port[i].write(_ref_relposeL_path[i]);
  }
}

std::vector<double> DistributedPathGeneratorInterface::getEstPoseL(){ return _est_poseL; }
std::vector<double> DistributedPathGeneratorInterface::getEstPoseR(){ return _est_poseR; }

std::vector<double> DistributedPathGeneratorInterface::getRefInLPath_x(){ return _ref_inL_path[0]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInLPath_y(){ return _ref_inL_path[1]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInLPath_t(){ return _ref_inL_path[2]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInRPath_x(){ return _ref_inR_path[0]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInRPath_y(){ return _ref_inR_path[1]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInRPath_t(){ return _ref_inR_path[2]; }

void DistributedPathGeneratorInterface::setRefRelPoseLPath(std::vector<double> const& ref_relposeL_path_x, std::vector<double> const& ref_relposeL_path_y, std::vector<double> const& ref_relposeL_path_t){
  _ref_relposeL_path[0] = ref_relposeL_path_x;
  _ref_relposeL_path[1] = ref_relposeL_path_y;
  _ref_relposeL_path[2] = ref_relposeL_path_t;
}

void DistributedPathGeneratorInterface::setRefRelPoseRPath(std::vector<double> const& ref_relposeR_path_x, std::vector<double> const& ref_relposeR_path_y, std::vector<double> const& ref_relposeR_path_t){
  _ref_relposeR_path[0] = ref_relposeR_path_x;
  _ref_relposeR_path[1] = ref_relposeR_path_y;
  _ref_relposeR_path[2] = ref_relposeR_path_t;
}
