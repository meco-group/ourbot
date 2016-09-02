#include "ControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <vector>

ControllerInterface::ControllerInterface(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose(3), _ref_ffw(3), _est_pose(3), _cmd_velocity(3){

  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose");
  ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
  ports()->addPort("ref_velocity_port", _ref_velocity_port).doc("Velocity reference sample");

  ports()->addPort("cmd_velocity_port",_cmd_velocity_port).doc("Velocity command for actuator");

  addProperty("control_sample_rate",_control_sample_rate).doc("Frequency to update the control loop");

  addOperation("writeSample",&ControllerInterface::writeSample, this).doc("Set data sample on output ports");
}

void ControllerInterface::writeSample(){
  std::vector<double> example(3, 0.0);
  _cmd_velocity_port.write(example);
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
  if (!_ref_velocity_port.connected()){
    log(Warning) << "ref_velocity_port not connected !" <<endlog();
  }
  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    check = false;
  }
  if (!check){
    return false;
  }
  return true;
}

// void ControllerInterface::updateHook(){

// }

void ControllerInterface::updateHook(){
  // Read estimated state from estimator
  if (_est_pose_port.read(_est_pose) == RTT::NoData) {
    // log(Warning) << "No data on _est_pose_port ! No control action." <<endlog();
    return;
  }
  // Read reference
  if (_ref_pose_port.read(_ref_pose) == RTT::NoData) {
    // log(Warning) << "No data on _ref_pose_port ! No control action." <<endlog();
    return;
  }
  if (_ref_velocity_port.connected()){
    if (_ref_velocity_port.read(_ref_ffw) == RTT::NoData) {
      // log(Warning) << "No data on _ref_velocity_port ! No control action." <<endlog();
      return;
    }
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
}

double ControllerInterface::getControlSampleRate(){ return _control_sample_rate; }
std::vector<double> ControllerInterface::getEstPose(){ return _est_pose; }
std::vector<double> ControllerInterface::getRefPose(){ return _ref_pose; }
std::vector<double> ControllerInterface::getRefFfw(){ return _ref_ffw; }
std::vector<double> ControllerInterface::getCmdVelocity(){ return _cmd_velocity; }
void ControllerInterface::setCmdVelocity(std::vector<double> const& cmd_velocity){_cmd_velocity = cmd_velocity; }

ORO_CREATE_COMPONENT_LIBRARY()
