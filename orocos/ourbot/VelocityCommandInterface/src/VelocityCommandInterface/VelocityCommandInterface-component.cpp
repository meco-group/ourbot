#include "VelocityCommandInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

VelocityCommandInterface::VelocityCommandInterface(std::string const& name) : TaskContext(name),
  _cmd_velocity(3){
  ports()->addPort("cmd_velocity_port",_cmd_velocity_port).doc("Velocity command for actuator");
  addOperation("setVelocity", &VelocityCommandInterface::setVelocity, this).doc("Set velocity cmd manually");
  addOperation("writeSample",&VelocityCommandInterface::writeSample, this).doc("Set data sample on output ports");
}

void VelocityCommandInterface::writeSample(){
  std::vector<double> example(3, 0.0);
  _cmd_velocity_port.write(example);
}

bool VelocityCommandInterface::configureHook(){
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _cmd_velocity_port.setDataSample(example);
  return true;
}

bool VelocityCommandInterface::startHook(){
  std::cout << "Velocity Commander started !" <<std::endl;
  return true;
}

void VelocityCommandInterface::updateHook(){
  _cmd_velocity_port.write(_cmd_velocity);
}

void VelocityCommandInterface::stopHook() {
  _cmd_velocity[0] = 0.0;
  _cmd_velocity[1] = 0.0;
  _cmd_velocity[2] = 0.0;
  _cmd_velocity_port.write(_cmd_velocity);
  std::cout << "Velocity Commander stopped !" <<std::endl;
}

void VelocityCommandInterface::cleanupHook() {
  _cmd_velocity[0] = 0.0;
  _cmd_velocity[1] = 0.0;
  _cmd_velocity[2] = 0.0;
  _cmd_velocity_port.write(_cmd_velocity);
}

void VelocityCommandInterface::setCmdVelocity(std::vector<double> const& cmd_velocity){_cmd_velocity = cmd_velocity; }

void VelocityCommandInterface::setVelocity(double vx, double vy, double w){
  _cmd_velocity[0] = vx;
  _cmd_velocity[1] = vy;
  _cmd_velocity[2] = w;
}

ORO_CREATE_COMPONENT_LIBRARY()
