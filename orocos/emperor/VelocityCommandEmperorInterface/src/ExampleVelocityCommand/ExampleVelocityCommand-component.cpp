#include "ExampleVelocityCommand-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ExampleVelocityCommand::ExampleVelocityCommand(std::string const& name) : VelocityCommandEmperorInterface(name){

}

void ExampleVelocityCommand::updateHook(){
  VelocityCommandEmperorInterface::updateHook();
}

ORO_LIST_COMPONENT_TYPE(ExampleVelocityCommand);
