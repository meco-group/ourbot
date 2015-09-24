#ifndef OROCOS_GAMEPADCOMMAND_COMPONENT_HPP
#define OROCOS_GAMEPADCOMMAND_COMPONENT_HPP

#include "../VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-component.hpp"

class GamePadCommand : public VelocityCommandEmperorInterface{
  private:
    RTT::InputPort<std::vector<double> > _gamepad_laxis_port;
    RTT::InputPort<std::vector<double> > _gamepad_raxis_port;
    std::vector<double> _cmd_velocity;
    double transformData(double);
  public:
    GamePadCommand(std::string const& name);
    virtual void updateHook();
};

#endif
