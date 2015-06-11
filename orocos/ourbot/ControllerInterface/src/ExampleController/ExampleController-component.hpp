#ifndef OROCOS_EXAMPLECONTROLLER_COMPONENT_HPP
#define OROCOS_EXAMPLECONTROLLER_COMPONENT_HPP

#include "../ControllerInterface/ControllerInterface-component.hpp"

class ExampleController : public ControllerInterface{
  private:
    double _kp[3];
    double _ki[3];
    std::vector<double> _error;

  public:
    ExampleController(std::string const& name);
    bool controlUpdate();
    bool initialize();
};

#endif
