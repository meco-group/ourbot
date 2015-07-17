#include "DistributedControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedControllerInterface::DistributedControllerInterface(std::string const& name, int com_size) : ControllerInterface(name),
    _ref_relposeL(3), _ref_relposeR(3), _est_poseL(3), _est_poseR(3){
  ports()->addPort("est_poseL_port",_est_poseL_port).doc("Estimated pose of neighbour via left going signal");
  ports()->addPort("est_poseR_port",_est_poseR_port).doc("Estimated pose of neighbour via right going signal");
  ports()->addPort("ref_relposeL_port", _ref_relposeL_port).doc("Relative pose reference sample wrt neighbour via left going signal");
  ports()->addPort("ref_relposeR_port", _ref_relposeR_port).doc("Relative pose reference sample wrt neighbour via right going signal");
  ports()->addPort("com_inL_port", _com_inL_port).doc("Input signal received from controller of neighbour via left going signal");
  ports()->addPort("com_inR_port", _com_inR_port).doc("Input signal received from controller of neighbour via right going signal");

  ports()->addPort("com_outL_port", _com_outL_port).doc("Output signal sent to controller of neighbour via left going signal");
  ports()->addPort("com_outR_port", _com_outR_port).doc("Output signal sent to controller of neighbour via right going signal");

  _com_size = com_size;
}

bool DistributedControllerInterface::configureHook(){
  // Reserve required memory and initialize with zeros
  _com_inL.resize(_com_size);
  _com_inR.resize(_com_size);
  _com_outL.resize(_com_size);
  _com_outR.resize(_com_size);

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(_com_size, 0.0);
  _com_outL_port.setDataSample(example);
  _com_outR_port.setDataSample(example);

  return ControllerInterface::configureHook();
}

bool DistributedControllerInterface::startHook(){
  // Check if input ports are connected
  bool check = true;

  if (!_est_poseL_port.connected()){
    log(Error) << "est_poseL_port not connected !" <<endlog();
    check = false;
  }
  if (!_ref_relposeL_port.connected()){
    log(Error) << "ref_relposeL_port not connected !" <<endlog();
    check = false;
  }
  if (!_est_poseR_port.connected()){
    log(Warning) << "est_poseR_port not connected !" <<endlog();
  }
  if (!_ref_relposeR_port.connected()){
    log(Warning) << "ref_relposeR_port not connected !" <<endlog();
  }
  if (!_com_inL_port.connected()){
    log(Warning) << "com_inL_port not connected !" <<endlog();
  }
  if (!_com_inR_port.connected()){
    log(Warning) << "com_inR_port not connected !" <<endlog();
  }
  if (!check){
    return false;
  }
  return ControllerInterface::startHook();
}

void DistributedControllerInterface::updateHook(){
  // Read estimated poses from estimator
  if (_est_poseL_port.read(_est_poseL) == RTT::NoData) {log(Error) << "No data on _est_poseL_port !" <<endlog(); error();}
  if (_est_poseR_port.connected()){
    if (_est_poseR_port.read(_est_poseR) == RTT::NoData) {log(Error) << "No data on _est_poseR_port !" <<endlog(); error();}
  }

  // Read reference rel poses
  if (_ref_relposeL_port.read(_ref_relposeL) == RTT::NoData) {log(Error) << "No data on _ref_relposeL_port !" <<endlog(); error();}
  if (_ref_relposeR_port.connected()){
    if (_ref_relposeR_port.read(_ref_relposeR) == RTT::NoData) {log(Error) << "No data on _ref_relposeR_port !" <<endlog(); error();}
  }

  // Read signals from neighbouring controllers
  if (_com_inL_port.connected()){
    if (_com_inL_port.read(_com_inL) == RTT::NoData) {log(Error) << "No data on _com_inL_port !" << endlog(); error();}
  }
  if (_com_inR_port.connected()){
    if (_com_inR_port.read(_com_inR) == RTT::NoData) {log(Error) << "No data on _com_inR_port !" << endlog(); error();}
  }

  ControllerInterface::updateHook();

  // Write signals to neighbouring controllers
  _com_outL_port.write(_com_outL);
  _com_outR_port.write(_com_outR);
}

std::vector<double> DistributedControllerInterface::getRefRelPoseL(){ return _ref_relposeL; }
std::vector<double> DistributedControllerInterface::getRefRelPoseR(){ return _ref_relposeR; }
std::vector<double> DistributedControllerInterface::getEstPoseL(){ return _est_poseL; }
std::vector<double> DistributedControllerInterface::getEstPoseR(){ return _est_poseR; }
std::vector<double> DistributedControllerInterface::getComInL(){ return _com_inL; }
std::vector<double> DistributedControllerInterface::getComInR(){ return _com_inR; }
void DistributedControllerInterface::setComOutL(std::vector<double> const& com_outL){ _com_outL = com_outL; }
void DistributedControllerInterface::setComOutR(std::vector<double> const& com_outR){ _com_outR = com_outR; }
