#include "GamePadCommand-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

GamePadCommand::GamePadCommand(std::string const& name) : VelocityCommandEmperorInterface(name), _cmd_velocity(3){
  this->ports()->addEventPort("gamepad_laxis_port", _gamepad_laxis_port).doc("X and Y value for left axis");
  this->ports()->addEventPort("gamepad_raxis_port", _gamepad_raxis_port).doc("X and Y value for right axis");
}

void GamePadCommand::updateHook(){
  std::vector<double> sample(2);
  if (_gamepad_raxis_port.read(sample) == RTT::NewData)
  {
    _cmd_velocity[0] = -sample[1]*_max_velocity/100.;
    _cmd_velocity[1] = sample[0]*_max_velocity/100.;
  }
  if (_gamepad_laxis_port.read(sample) == RTT::NewData)
  {
    _cmd_velocity[2] = sample[0]*_max_omega/100.;
  }
  setCmdVelocity(_cmd_velocity);
  VelocityCommandEmperorInterface::updateHook();
}

ORO_LIST_COMPONENT_TYPE(GamePadCommand);
