#include "DistributedControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedControllerInterface::DistributedControllerInterface(std::string const& name, int com_size) : ControllerInterface(name),
    _ref_relposeA(3), _ref_relposeB(3), _est_poseA(3), _est_poseB(3){
  ports()->addPort("est_poseA_inport",_est_poseA_inport).doc("Estimated pose of neighbour A");
  ports()->addPort("est_poseB_inport",_est_poseB_inport).doc("Estimated pose of neighbour B");
  ports()->addPort("ref_relposeA_inport", _ref_relposeA_inport).doc("Relative pose reference sample wrt neighbour A");
  ports()->addPort("ref_relposeB_inport", _ref_relposeB_inport).doc("Relative pose reference sample wrt neighbour B");
  ports()->addPort("com_inA_inport", _com_inA_inport).doc("Input signal received from neighbour A's controller");
  ports()->addPort("com_inB_inport", _com_inB_inport).doc("Input signal received from neighbour B's controller");

  ports()->addPort("com_outA_outport", _com_outA_outport).doc("Output signal sent to neighbour A's controller");
  ports()->addPort("com_outB_outport", _com_outB_outport).doc("Output signal sent to neighbour B's controller");

  _com_size = com_size;
}

bool DistributedControllerInterface::configureHook(){
  // Reserve required memory and initialize with zeros
  _com_inA.resize(_com_size);
  _com_inB.resize(_com_size);
  _com_outA.resize(_com_size);
  _com_outB.resize(_com_size);

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_com_size, 0.0);
  _com_outA_outport.setDataSample(example);
  _com_outB_outport.setDataSample(example);

  return ControllerInterface::configureHook();
}

bool DistributedControllerInterface::startHook(){
  // Check if input ports are connected
  bool check = true;

  if (!_est_poseA_inport.connected()){
    log(Error) << "est_poseA_inport not connected !" <<endlog();
    check = false;
  }
  if (!_ref_relposeA_inport.connected()){
    log(Error) << "ref_relposeA_inport not connected !" <<endlog();
    check = false;
  }
  if (!_est_poseB_inport.connected()){
    log(Warning) << "est_poseB_inport not connected !" <<endlog();
  }
  if (!_ref_relposeB_inport.connected()){
    log(Warning) << "ref_relposeB_inport not connected !" <<endlog();
  }
  if (!_com_inA_inport.connected()){
    log(Warning) << "com_inA_inport not connected !" <<endlog();
  }
  if (!_com_inB_inport.connected()){
    log(Warning) << "com_inB_inport not connected !" <<endlog();
  }
  if (!check){
    return false;
  }
  return ControllerInterface::startHook();
}

void DistributedControllerInterface::updateHook(){
  // Read estimated poses from estimator
  if (_est_poseA_inport.read(_est_poseA) == RTT::NoData) {log(Error) << "No data on _est_poseA_inport !" <<endlog(); fatal();}
  if (_est_poseB_inport.connected()){
    if (_est_poseB_inport.read(_est_poseB) == RTT::NoData) {log(Error) << "No data on _est_poseB_inport !" <<endlog(); fatal();}
  }

  // Read reference rel poses
  if (_ref_relposeA_inport.read(_ref_relposeA) == RTT::NoData) {log(Error) << "No data on _ref_relposeA_inport !" <<endlog(); fatal();}
  if (_ref_relposeB_inport.connected()){
    if (_ref_relposeB_inport.read(_ref_relposeB) == RTT::NoData) {log(Error) << "No data on _ref_relposeB_inport !" <<endlog(); fatal();}
  }

  // Read signals from neighbouring controllers
  if (_com_inA_inport.connected()){
    if (_com_inA_inport.read(_com_inA) == RTT::NoData) {log(Error) << "No data on _com_inA_inport !" << endlog(); fatal();}
  }
  if (_com_inB_inport.connected()){
    if (_com_inB_inport.read(_com_inB) == RTT::NoData) {log(Error) << "No data on _com_inB_inport !" << endlog(); fatal();}
  }

  ControllerInterface::updateHook();

  // Write signals to neighbouring controllers
  _com_outA_outport.write(_com_outA);
  _com_outB_outport.write(_com_outB);
}

std::vector<double> DistributedControllerInterface::getRefRelPoseA(){ return _ref_relposeA; }
std::vector<double> DistributedControllerInterface::getRefRelPoseB(){ return _ref_relposeB; }
std::vector<double> DistributedControllerInterface::getEstPoseA(){ return _est_poseA; }
std::vector<double> DistributedControllerInterface::getEstPoseB(){ return _est_poseB; }
std::vector<double> DistributedControllerInterface::getComInA(){ return _com_inA; }
std::vector<double> DistributedControllerInterface::getComInB(){ return _com_inB; }
void DistributedControllerInterface::setComOutA(std::vector<double> const& com_outA){ _com_outA = com_outA; }
void DistributedControllerInterface::setComOutB(std::vector<double> const& com_outB){ _com_outB = com_outB; }
