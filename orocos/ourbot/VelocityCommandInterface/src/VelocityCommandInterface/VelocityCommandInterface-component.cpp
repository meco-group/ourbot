#include "VelocityCommandInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

VelocityCommandInterface::VelocityCommandInterface(std::string const& name) : TaskContext(name),
  _cmd_velocity(3){
  ports()->addPort("cmd_velocity_port",_cmd_velocity_port).doc("Velocity command for actuator");
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
  log(Info) << "Velocity Commander updated !" <<endlog();
}

void VelocityCommandInterface::stopHook() {
  std::cout << "Velocity Commander stopped !" <<std::endl;
}

void VelocityCommandInterface::setCmdVelocity(std::vector<double> const& cmd_velocity){_cmd_velocity = cmd_velocity; }

ORO_CREATE_COMPONENT_LIBRARY()
