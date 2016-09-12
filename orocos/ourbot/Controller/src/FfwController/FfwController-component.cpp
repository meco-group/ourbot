#include "FfwController-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

FfwController::FfwController(std::string const& name) : ControllerInterface(name){

}

bool FfwController::initialize(){
  return true;
}

bool FfwController::controlUpdate(){
  std::vector<double> ref_ffw           = getRefFfw();
  setCmdVelocity(ref_ffw);
  return true;
}

ORO_LIST_COMPONENT_TYPE(FfwController);
