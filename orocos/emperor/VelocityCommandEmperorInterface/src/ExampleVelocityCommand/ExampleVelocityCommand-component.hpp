#ifndef OROCOS_EXAMPLEVELOCITYCOMMAND_COMPONENT_HPP
#define OROCOS_EXAMPLEVELOCITYCOMMAND_COMPONENT_HPP

#include "../VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-component.hpp"

class ExampleVelocityCommand : public VelocityCommandEmperorInterface{

  public:
    ExampleVelocityCommand(std::string const& name);
    virtual void updateHook();
};

#endif
