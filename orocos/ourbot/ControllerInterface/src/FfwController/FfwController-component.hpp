#ifndef OROCOS_FFWCONTROLLER_COMPONENT_HPP
#define OROCOS_FFWCONTROLLER_COMPONENT_HPP

#include "../ControllerInterface/ControllerInterface-component.hpp"

class FfwController : public ControllerInterface{
  public:
    FfwController(std::string const& name);
    bool controlUpdate();
    bool initialize();
};

#endif
