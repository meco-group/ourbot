#include "ControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <vector>

ControllerInterface::ControllerInterface(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose(3), _ref_ffw(3), _est_pose(3), _set_velocity(3){

  ports()->addPort("est_pose_inport",_est_pose_inport).doc("Estimated pose");
  ports()->addPort("ref_pose_inport", _ref_pose_inport).doc("Pose reference sample");
  ports()->addPort("ref_ffw_inport", _ref_ffw_inport).doc("Feedforward reference sample");

  ports()->addPort("set_velocity_outport",_set_velocity_outport).doc("Setpoint (vx,vy,w) for Teensy");
}

bool ControllerInterface::configureHook(){
  TaskContext* configurator = getPeer("configurator");
  if (configurator==NULL){
    log(Error) << "No peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getControlSampleRate = configurator->getOperation("getControlSampleRate");
  if (!getControlSampleRate.ready()){
    log(Error) << "No operation getControlSampleRate of peer configurator !" <<endlog();
    return false;
  }
  _sample_rate = getControlSampleRate();

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _set_velocity_outport.setDataSample(example);

  return true;
}

bool ControllerInterface::startHook(){
  // Check if input ports are connected
  bool check = true;
  if (!_est_pose_inport.connected()){
    log(Error) << "est_pose_inport not connected !" <<endlog();
    check = false;
  }
  if (!_ref_pose_inport.connected()){
    log(Error) << "ref_pose_inport not connected !" <<endlog();
    check = false;
  }
  if (!_ref_ffw_inport.connected()){
    log(Warning) << "ref_ffw_inport not connected !" <<endlog();
  }
  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    check = false;
  }
  if (!check){
    return false;
  }
  std::cout << "Controller started !" <<std::endl;
  return true;
}

void ControllerInterface::updateHook(){
  // Read estimated state from estimator
  if (_est_pose_inport.read(_est_pose) == RTT::NoData) {log(Error) << "No data on _est_pose_inport !" <<endlog(); fatal();}

  // Read reference
  if (_ref_pose_inport.read(_ref_pose) == RTT::NoData) {log(Error) << "No data on _ref_pose_inport !" <<endlog(); fatal();}
  if (_ref_ffw_inport.connected()){
    if (_ref_ffw_inport.read(_ref_ffw) == RTT::NoData) {log(Error) << "No data on _ref_ffw_inport !" <<endlog(); fatal();}
  }

  // Apply control law
  controlUpdate();

  // Write velocity setpoints
  _set_velocity_outport.write(_set_velocity);

  log(Info) << "Controller updated !" <<endlog();
}

void ControllerInterface::stopHook() {
  // Set velocity setpoint to zero
  std::vector<double> zerovelocity(3,0.0);
  setSetVelocity(zerovelocity);
  _set_velocity_outport.write(_set_velocity);
  std::cout << "Controller stopped !" <<std::endl;
}

double ControllerInterface::getSampleRate(){ return _sample_rate; }
std::vector<double> ControllerInterface::getEstPose(){ return _est_pose; }
std::vector<double> ControllerInterface::getRefPose(){ return _ref_pose; }
std::vector<double> ControllerInterface::getRefFfw(){ return _ref_ffw; }
std::vector<double> ControllerInterface::getSetVelocity(){ return _set_velocity; }
void ControllerInterface::setSetVelocity(std::vector<double> const& set_velocity){_set_velocity = set_velocity; }
