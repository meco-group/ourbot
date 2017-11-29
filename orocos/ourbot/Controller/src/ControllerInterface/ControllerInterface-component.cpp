#include "ControllerInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <vector>

ControllerInterface::ControllerInterface(std::string const& name) :
  TaskContext(name, PreOperational), _ref_pose(3), _ref_velocity(3),
  _est_pose(3), _cmd_velocity(3) {

  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose");
  ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
  ports()->addPort("ref_velocity_port", _ref_velocity_port).doc("Velocity reference sample");
  ports()->addPort("cmd_velocity_port", _cmd_velocity_port).doc("Velocity command for actuator");
}

bool ControllerInterface::configureHook() {
  // show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _cmd_velocity_port.setDataSample(example);
  return true;
}

bool ControllerInterface::startHook() {
  // check if input ports are connected
  if (!_est_pose_port.connected()) {
    log(Warning) << "est_pose_port not connected!" <<endlog();
  }
  if (!_ref_pose_port.connected()) {
    log(Warning) << "ref_pose_port not connected!" <<endlog();
  }
  if (!_ref_velocity_port.connected()) {
    log(Warning) << "ref_velocity_port not connected!" <<endlog();
  }
  if (!initialize()) {
    log(Error) << "Error occured in initialize()!" <<endlog();
    return false;
  }
  return true;
}

void ControllerInterface::updateHook() {
  // read out input ports
  _est_pose_port.read(_est_pose);
  _ref_pose_port.read(_ref_pose);
  _ref_velocity_port.read(_ref_velocity);
  // apply control law
  controlHook();
  // write velocity setpoints
  _cmd_velocity_port.write(_cmd_velocity);
}

void ControllerInterface::stopHook() {
  // set velocity setpoint to zero
  std::vector<double> zero_velocity(3, 0.0);
  setCmdVelocity(zero_velocity);
  _cmd_velocity_port.write(_cmd_velocity);
}

std::vector<double> ControllerInterface::getEstPose() {
  return _est_pose;
}

std::vector<double> ControllerInterface::getRefPose() {
  return _ref_pose;
}

std::vector<double> ControllerInterface::getRefVelocity() {
  return _ref_velocity;
}

std::vector<double> ControllerInterface::getCmdVelocity() {
  return _cmd_velocity;
}

void ControllerInterface::setCmdVelocity(const std::vector<double>& cmd_velocity) {
  _cmd_velocity = cmd_velocity;
}

ORO_CREATE_COMPONENT_LIBRARY()
