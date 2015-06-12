#include "DistributedPathGeneratorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedPathGeneratorInterface::DistributedPathGeneratorInterface(std::string const& name) : PathGeneratorInterface(name),
    _est_poseA(3), _est_poseB(3){
  ports()->addPort("est_poseA_inport",_est_poseA_inport).doc("Estimated pose of neighbour A");
  ports()->addPort("est_poseB_inport",_est_poseB_inport).doc("Estimated pose of neighbour B");

  ports()->addPort("ref_inA_path_x_inport",_ref_inA_path_inport[0]).doc("x reference path of neighbour A");
  ports()->addPort("ref_inA_path_y_inport",_ref_inA_path_inport[1]).doc("y reference path of neighbour A");
  ports()->addPort("ref_inA_path_t_inport",_ref_inA_path_inport[2]).doc("theta reference path of neighbour A");

  ports()->addPort("ref_inB_path_x_inport",_ref_inB_path_inport[0]).doc("x reference path of neighbour B");
  ports()->addPort("ref_inB_path_y_inport",_ref_inB_path_inport[1]).doc("y reference path of neighbour B");
  ports()->addPort("ref_inB_path_t_inport",_ref_inB_path_inport[2]).doc("theta reference path of neighbour B");

  ports()->addPort("ref_relposeA_path_x_outport", _ref_relposeA_path_outport[0]).doc("x relpose reference path wrt neighbour A");
  ports()->addPort("ref_relposeA_path_y_outport", _ref_relposeA_path_outport[1]).doc("y relpose reference path wrt neighbour A");
  ports()->addPort("ref_relposeA_path_t_outport", _ref_relposeA_path_outport[2]).doc("theta relpose reference path wrt neighbour A");

  ports()->addPort("ref_relposeB_path_x_outport", _ref_relposeB_path_outport[0]).doc("x relpose reference path wrt neighbour B");
  ports()->addPort("ref_relposeB_path_y_outport", _ref_relposeB_path_outport[1]).doc("y relpose reference path wrt neighbour B");
  ports()->addPort("ref_relposeB_path_t_outport", _ref_relposeB_path_outport[2]).doc("theta relpose reference path wrt neighbour B");
}

bool DistributedPathGeneratorInterface::configureHook(){
  bool check = PathGeneratorInterface::configureHook();

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _ref_relposeA_path[i].resize(_path_length);
    _ref_relposeB_path[i].resize(_path_length);
    _ref_inA_path[i].resize(_path_length);
    _ref_inB_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_path_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_relposeA_path_outport[i].setDataSample(example);
    _ref_relposeB_path_outport[i].setDataSample(example);
  }

  return check;
}

bool DistributedPathGeneratorInterface::startHook(){
  // Check if input ports are connected
  if (!_est_poseA_inport.connected()){
    log(Warning) << "_est_poseA_inport not connected !" <<endlog();
  }
  if (!_est_poseB_inport.connected()){
    log(Warning) << "_est_poseB_inport not connected !" <<endlog();
  }
  for (int i=0; i<3; i++){
    if (!_ref_inA_path_inport[i].connected()){
      log(Warning) << "_ref_inA_inport "<< i << " not connected !" <<endlog();
    }
    if (!_ref_inB_path_inport[i].connected()){
      log(Warning) << "_ref_inB_inport "<< i << " not connected !" <<endlog();
    }
  }

  return PathGeneratorInterface::startHook();
}

void DistributedPathGeneratorInterface::updateHook(){ // NOT GOING TO FAATL
  // Read estimated pose
  std::vector<double> pose(3);
  if (_est_poseA_inport.connected() && (_est_poseA_inport.read(pose) == RTT::NewData)){
    _est_poseA = pose;
  }
  if (_est_poseB_inport.connected() && (_est_poseB_inport.read(pose) == RTT::NewData)){
    _est_poseA = pose;
  }

  // Read paths from neighbours
  std::vector<double> path(_path_length);
  for (int i=0; i<3; i++){
    if (_ref_inA_path_inport[i].connected() && (_ref_inA_path_inport[i].read(path) == RTT::NewData)){
      _ref_inA_path[i] = path;
    }
    if (_ref_inB_path_inport[i].connected() && (_ref_inB_path_inport[i].read(path) == RTT::NewData)){
      _ref_inB_path[i] = path;
    }
  }

  PathGeneratorInterface::updateHook();

  // Write path
  for (int i=0; i<3; i++){
    _ref_relposeA_path_outport[i].write(_ref_relposeA_path[i]);
    _ref_relposeA_path_outport[i].write(_ref_relposeA_path[i]);
  }
}

std::vector<double> DistributedPathGeneratorInterface::getEstPoseA(){ return _est_poseA; }
std::vector<double> DistributedPathGeneratorInterface::getEstPoseB(){ return _est_poseB; }

std::vector<double> DistributedPathGeneratorInterface::getRefInAPath_x(){ return _ref_inA_path[0]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInAPath_y(){ return _ref_inA_path[1]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInAPath_t(){ return _ref_inA_path[2]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInBPath_x(){ return _ref_inB_path[0]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInBPath_y(){ return _ref_inB_path[1]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInBPath_t(){ return _ref_inB_path[2]; }

void DistributedPathGeneratorInterface::setRefRelPoseAPath(std::vector<double> const& ref_relposeA_path_x, std::vector<double> const& ref_relposeA_path_y, std::vector<double> const& ref_relposeA_path_t){
  _ref_relposeA_path[0] = ref_relposeA_path_x;
  _ref_relposeA_path[1] = ref_relposeA_path_y;
  _ref_relposeA_path[2] = ref_relposeA_path_t;
}

void DistributedPathGeneratorInterface::setRefRelPoseBPath(std::vector<double> const& ref_relposeB_path_x, std::vector<double> const& ref_relposeB_path_y, std::vector<double> const& ref_relposeB_path_t){
  _ref_relposeB_path[0] = ref_relposeB_path_x;
  _ref_relposeB_path[1] = ref_relposeB_path_y;
  _ref_relposeB_path[2] = ref_relposeB_path_t;
}
