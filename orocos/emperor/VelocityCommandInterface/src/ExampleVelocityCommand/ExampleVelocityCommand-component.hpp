#ifndef OROCOS_EXAMPLEVELOCITYCOMMAND_COMPONENT_HPP
#define OROCOS_EXAMPLEVELOCITYCOMMAND_COMPONENT_HPP

#include "../VelocityCommandInterface/VelocityCommandInterface-component.hpp"

class ExampleVelocityCommand : public VelocityCommandInterface{

  public:
    ExampleVelocityCommand(std::string const& name);
    virtual void updateHook();
};

#endif
