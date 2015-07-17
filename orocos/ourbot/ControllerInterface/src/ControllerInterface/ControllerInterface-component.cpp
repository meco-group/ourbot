#include "ControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <vector>

ControllerInterface::ControllerInterface(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose(3), _ref_ffw(3), _est_pose(3), _cmd_velocity(3){

  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose");
  ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
  ports()->addPort("ref_ffw_port", _ref_ffw_port).doc("Feedforward reference sample");

  ports()->addPort("cmd_velocity_port",_cmd_velocity_port).doc("Velocity command for actuator");

  addProperty("control_sample_rate",_control_sample_rate).doc("Frequency to update the control loop");
}

bool ControllerInterface::configureHook(){
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _cmd_velocity_port.setDataSample(example);

  return true;
}

bool ControllerInterface::startHook(){
  // Check if input ports are connected
  bool check = true;
  if (!_est_pose_port.connected()){
    log(Error) << "est_pose_port not connected !" <<endlog();
    check = false;
  }
  if (!_ref_pose_port.connected()){
    log(Error) << "ref_pose_port not connected !" <<endlog();
    check = false;
  }
  if (!_ref_ffw_port.connected()){
    log(Warning) << "ref_ffw_port not connected !" <<endlog();
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

// void ControllerInterface::updateHook(){

// }

void ControllerInterface::updateHook(){
  // Read estimated state from estimator
  if (_est_pose_port.read(_est_pose) == RTT::NoData) {log(Error) << "No data on _est_pose_port !" <<endlog(); error();}

  // Read reference
  if (_ref_pose_port.read(_ref_pose) == RTT::NoData) {log(Error) << "No data on _ref_pose_port !" <<endlog(); error();}
  if (_ref_ffw_port.connected()){
    if (_ref_ffw_port.read(_ref_ffw) == RTT::NoData) {log(Error) << "No data on _ref_ffw_port !" <<endlog(); error();}
  }

  // Apply control law
  controlUpdate();

  // Write velocity setpoints
  _cmd_velocity_port.write(_cmd_velocity);
}

void ControllerInterface::stopHook() {
  // Set velocity setpoint to zero
  std::vector<double> zerovelocity(3,0.0);
  setCmdVelocity(zerovelocity);
  _cmd_velocity_port.write(_cmd_velocity);
  std::cout << "Controller stopped !" <<std::endl;
}

double ControllerInterface::getControlSampleRate(){ return _control_sample_rate; }
std::vector<double> ControllerInterface::getEstPose(){ return _est_pose; }
std::vector<double> ControllerInterface::getRefPose(){ return _ref_pose; }
std::vector<double> ControllerInterface::getRefFfw(){ return _ref_ffw; }
std::vector<double> ControllerInterface::getCmdVelocity(){ return _cmd_velocity; }
void ControllerInterface::setCmdVelocity(std::vector<double> const& cmd_velocity){_cmd_velocity = cmd_velocity; }

ORO_CREATE_COMPONENT_LIBRARY()
