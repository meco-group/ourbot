#ifndef OROCOS_EXAMPLEVELOCITYCOMMAND_COMPONENT_HPP
#define OROCOS_EXAMPLEVELOCITYCOMMAND_COMPONENT_HPP

#include "../VelocityCommandInterface/VelocityCommandInterface-component.hpp"

class ExampleVelocityCommand : public VelocityCommandInterface{
  private:
    OutputPort<double > _test_port_out;
    InputPort<double > _test_port_in;
    OutputPort<std::vector<double> > _test_port_vec_out;
    InputPort<std::vector<double> > _test_port_vec_in;
    std::vector<double> _data_send;
    std::vector<double> _data_receive;
    double _cnt;

  public:
    ExampleVelocityCommand(std::string const& name);
    virtual void updateHook();
};

#endif
