#include "VelocityCommandInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

VelocityCommandInterface::VelocityCommandInterface(std::string const& name) : TaskContext(name){
  std::cout << "VelocityCommandInterface constructed !" <<std::endl;
}

bool VelocityCommandInterface::configureHook(){
  std::cout << "VelocityCommandInterface configured !" <<std::endl;
  return true;
}

bool VelocityCommandInterface::startHook(){
  std::cout << "VelocityCommandInterface started !" <<std::endl;
  return true;
}

void VelocityCommandInterface::updateHook(){
  std::cout << "VelocityCommandInterface executes updateHook !" <<std::endl;
}

void VelocityCommandInterface::stopHook() {
  std::cout << "VelocityCommandInterface executes stopping !" <<std::endl;
}

void VelocityCommandInterface::cleanupHook() {
  std::cout << "VelocityCommandInterface cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(VelocityCommandInterface)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(VelocityCommandInterface)
