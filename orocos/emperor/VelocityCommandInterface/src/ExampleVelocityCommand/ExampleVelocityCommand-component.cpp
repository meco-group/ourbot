#include "ExampleVelocityCommand-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ExampleVelocityCommand::ExampleVelocityCommand(std::string const& name) : VelocityCommandInterface(name){

}

void ExampleVelocityCommand::updateHook(){
  std::vector<double> cmd_velocity(3);
  cmd_velocity.at(0) = 0.0;
  cmd_velocity.at(1) = 1.0;
  cmd_velocity.at(2) = 2.0;
  setCmdVelocity(cmd_velocity);
  VelocityCommandInterface::updateHook();
}

ORO_LIST_COMPONENT_TYPE(ExampleVelocityCommand);
