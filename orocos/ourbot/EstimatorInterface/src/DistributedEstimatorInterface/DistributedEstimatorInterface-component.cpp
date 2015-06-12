#include "DistributedEstimatorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedEstimatorInterface::DistributedEstimatorInterface(std::string const& name, int com_size) : EstimatorInterface(name),
    _est_poseA(3), _est_poseB(3){

  ports()->addPort("com_inA_inport", _com_inA_inport).doc("Input signal received from neighbour A's estimator");
  ports()->addPort("com_inB_inport", _com_inB_inport).doc("Input signal received from neighbour B's estimator");

  ports()->addPort("est_poseA_outport",_est_poseA_outport).doc("Estimated pose of neighbour A");
  ports()->addPort("est_poseB_outport",_est_poseB_outport).doc("Estimated pose of neighbour B");

  _com_size = com_size;
}

bool DistributedEstimatorInterface::configureHook(){
  // Reserve required memory and initialize with zeros
  _com_inA.resize(_com_size);
  _com_inB.resize(_com_size);

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _est_poseA_outport.setDataSample(example);
  _est_poseB_outport.setDataSample(example);

  return EstimatorInterface::configureHook();
}

bool DistributedEstimatorInterface::startHook(){
  // Check if input ports are connected
  if (!_com_inA_inport.connected()){
    log(Warning) << "com_inA_inport not connected !" <<endlog();
  }
  if (!_com_inB_inport.connected()){
    log(Warning) << "com_inB_inport not connected !" <<endlog();
  }
  return EstimatorInterface::startHook();
}

void DistributedEstimatorInterface::updateHook(){
  // Read signals from neighbouring controllers
  if (_com_inA_inport.connected()){
    if (_com_inA_inport.read(_com_inA) == RTT::NoData) {log(Error) << "No data on _com_inA_inport !" << endlog(); fatal();}
  }
  if (_com_inB_inport.connected()){
    if (_com_inB_inport.read(_com_inB) == RTT::NoData) {log(Error) << "No data on _com_inB_inport !" << endlog(); fatal();}
  }

  EstimatorInterface::updateHook();

  // Write estimated poses of neighbours
  _est_poseA_outport.write(_est_poseA);
  _est_poseB_outport.write(_est_poseB);
}

std::vector<double> DistributedEstimatorInterface::getComInA(){ return _com_inA; }
std::vector<double> DistributedEstimatorInterface::getComInB(){ return _com_inB; }
void DistributedEstimatorInterface::setEstPoseA(std::vector<double> const& est_poseA){ _est_poseA = est_poseA; }
void DistributedEstimatorInterface::setEstPoseB(std::vector<double> const& est_poseB){ _est_poseB = est_poseB; }
