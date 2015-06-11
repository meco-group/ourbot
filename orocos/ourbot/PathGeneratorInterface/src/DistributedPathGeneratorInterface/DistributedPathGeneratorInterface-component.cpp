#include "DistributedPathGeneratorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedPathGeneratorInterface::DistributedPathGeneratorInterface(std::string const& name) : PathGeneratorInterface(name),
    _est_poseA(3), _est_poseB(3){
  ports()->addPort("est_poseA_inport",_est_poseA_inport).doc("Estimated pose of neighbour A");
  ports()->addPort("est_poseB_inport",_est_poseB_inport).doc("Estimated pose of neighbour B");

  ports()->addPort("ref_inA_x_inport",_ref_inA_inport[0]).doc("x reference of neighbour A");
  ports()->addPort("ref_inA_y_inport",_ref_inA_inport[1]).doc("y reference of neighbour A");
  ports()->addPort("ref_inA_t_inport",_ref_inA_inport[2]).doc("theta reference of neighbour A");

  ports()->addPort("ref_inB_x_inport",_ref_inB_inport[0]).doc("x reference of neighbour B");
  ports()->addPort("ref_inB_y_inport",_ref_inB_inport[1]).doc("y reference of neighbour B");
  ports()->addPort("ref_inB_t_inport",_ref_inB_inport[2]).doc("theta reference of neighbour B");

  ports()->addPort("ref_relposeA_x_outport", _ref_relposeA_outport[0]).doc("x relpose reference wrt neighbour A");
  ports()->addPort("ref_relposeA_y_outport", _ref_relposeA_outport[1]).doc("y relpose reference wrt neighbour A");
  ports()->addPort("ref_relposeA_t_outport", _ref_relposeA_outport[2]).doc("theta relpose reference wrt neighbour A");

  ports()->addPort("ref_relposeB_x_outport", _ref_relposeB_outport[0]).doc("x relpose reference wrt neighbour B");
  ports()->addPort("ref_relposeB_y_outport", _ref_relposeB_outport[1]).doc("y relpose reference wrt neighbour B");
  ports()->addPort("ref_relposeB_t_outport", _ref_relposeB_outport[2]).doc("theta relpose reference wrt neighbour B");
}

bool DistributedPathGeneratorInterface::configureHook(){
  bool check = PathGeneratorInterface::configureHook();

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _ref_relposeA[i].resize(_path_length);
    _ref_relposeB[i].resize(_path_length);
    _ref_inA[i].resize(_path_length);
    _ref_inB[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_path_length, 0.0);
  for(int i=0; i<3; i++){
    _ref_relposeA_outport[i].setDataSample(example);
    _ref_relposeB_outport[i].setDataSample(example);
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
    if (!_ref_inA_inport[i].connected()){
      log(Warning) << "_ref_inA_inport "<< i << " not connected !" <<endlog();
    }
    if (!_ref_inB_inport[i].connected()){
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
    if (_ref_inA_inport[i].connected() && (_ref_inA_inport[i].read(path) == RTT::NewData)){
      _ref_inA[i] = path;
    }
    if (_ref_inB_inport[i].connected() && (_ref_inB_inport[i].read(path) == RTT::NewData)){
      _ref_inB[i] = path;
    }
  }

  PathGeneratorInterface::updateHook();

  // Write path
  for (int i=0; i<3; i++){
    _ref_relposeA_outport[i].write(_ref_relposeA[i]);
    _ref_relposeA_outport[i].write(_ref_relposeA[i]);
  }
}

std::vector<double> DistributedPathGeneratorInterface::getEstPoseA(){ return _est_poseA; }
std::vector<double> DistributedPathGeneratorInterface::getEstPoseB(){ return _est_poseB; }

std::vector<double> DistributedPathGeneratorInterface::getRefInA_x(){ return _ref_inA[0]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInA_y(){ return _ref_inA[1]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInA_t(){ return _ref_inA[2]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInB_x(){ return _ref_inB[0]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInB_y(){ return _ref_inB[1]; }
std::vector<double> DistributedPathGeneratorInterface::getRefInB_t(){ return _ref_inB[2]; }

void DistributedPathGeneratorInterface::setRefRelPoseA(std::vector<double> const& ref_relposeA_x, std::vector<double> const& ref_relposeA_y, std::vector<double> const& ref_relposeA_t){
  _ref_relposeA[0] = ref_relposeA_x;
  _ref_relposeA[1] = ref_relposeA_y;
  _ref_relposeA[2] = ref_relposeA_t;
}

void DistributedPathGeneratorInterface::setRefRelPoseB(std::vector<double> const& ref_relposeB_x, std::vector<double> const& ref_relposeB_y, std::vector<double> const& ref_relposeB_t){
  _ref_relposeB[0] = ref_relposeB_x;
  _ref_relposeB[1] = ref_relposeB_y;
  _ref_relposeB[2] = ref_relposeB_t;
}
