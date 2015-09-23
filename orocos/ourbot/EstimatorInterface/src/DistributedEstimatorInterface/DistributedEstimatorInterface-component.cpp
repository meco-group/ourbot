#include "DistributedEstimatorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedEstimatorInterface::DistributedEstimatorInterface(std::string const& name, int com_size) : EstimatorInterface(name),
    _est_poseL(3), _est_poseR(3){

  ports()->addPort("com_inL_port", _com_inL_port).doc("Input signal received from estimator of neighbour via left going signal");
  ports()->addPort("com_inR_port", _com_inR_port).doc("Input signal received from estimator of neighbour via right going signal");

  ports()->addPort("est_poseL_port",_est_poseL_port).doc("Estimated pose of neighbour via left going signal");
  ports()->addPort("est_poseR_port",_est_poseR_port).doc("Estimated pose of neighbour via right going signal");

  addOperation("writeSample",&DistributedEstimatorInterface::writeSample, this).doc("Set data sample on output ports");

  _com_size = com_size;
}

void DistributedEstimatorInterface::writeSample(){
  std::vector<double> example(3, 0.0);
  _est_poseL_port.write(example);
  _est_poseR_port.write(example);
}

bool DistributedEstimatorInterface::configureHook(){
  // Reserve required memory and initialize with zeros
  _com_inL.resize(_com_size);
  _com_inR.resize(_com_size);

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _est_poseL_port.setDataSample(example);
  _est_poseR_port.setDataSample(example);
  return EstimatorInterface::configureHook();
}

bool DistributedEstimatorInterface::startHook(){
  // Check if input ports are connected
  if (!_com_inL_port.connected()){
    log(Warning) << "com_inL_port not connected !" <<endlog();
  }
  if (!_com_inR_port.connected()){
    log(Warning) << "com_inR_port not connected !" <<endlog();
  }
  return EstimatorInterface::startHook();
}

void DistributedEstimatorInterface::updateHook(){
  // Read signals from neighbouring controllers
  if (_com_inL_port.connected()){
    if (_com_inL_port.read(_com_inL) == RTT::NoData) {log(Error) << "No data on _com_inL_port !" << endlog(); error();}
  }
  if (_com_inR_port.connected()){
    if (_com_inR_port.read(_com_inR) == RTT::NoData) {log(Error) << "No data on _com_inR_port !" << endlog(); error();}
  }

  EstimatorInterface::updateHook();

  // Write estimated poses of neighbours
  _est_poseL_port.write(_est_poseL);
  _est_poseR_port.write(_est_poseR);
}

std::vector<double> DistributedEstimatorInterface::getComInL(){ return _com_inL; }
std::vector<double> DistributedEstimatorInterface::getComInR(){ return _com_inR; }
void DistributedEstimatorInterface::setEstPoseL(std::vector<double> const& est_poseL){ _est_poseL = est_poseL; }
void DistributedEstimatorInterface::setEstPoseR(std::vector<double> const& est_poseR){ _est_poseR = est_poseR; }
