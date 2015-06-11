#include "DistributedControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedControllerInterface::DistributedControllerInterface(std::string const& name) : ControllerInterface(name),
    _ref_relposeA(3), _ref_relposeB(3), _est_poseA(3), _est_poseB(3){
  ports()->addPort("est_poseA_inport",_est_poseA_inport).doc("Estimated pose of neighbour A");
  ports()->addPort("est_poseB_inport",_est_poseB_inport).doc("Estimated pose of neighbour B");

  ports()->addPort("ref_relposeA_inport", _ref_relposeA_inport).doc("Relative pose reference sample wrt neighbour A");
  ports()->addPort("ref_relposeB_inport", _ref_relposeB_inport).doc("Relative pose reference sample wrt neighbour B");

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

  ControllerInterface::updateHook();
}

std::vector<double> DistributedControllerInterface::getRefRelPoseA(){ return _ref_relposeA; }
std::vector<double> DistributedControllerInterface::getRefRelPoseB(){ return _ref_relposeB; }
std::vector<double> DistributedControllerInterface::getEstPoseA(){ return _est_poseA; }
std::vector<double> DistributedControllerInterface::getEstPoseB(){ return _est_poseB; }
