#include "SPIMaster-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

SPIMaster::SPIMaster(std::string const& name) : TaskContext(name){
  std::cout << "SPIMaster constructed !" <<std::endl;
}

bool SPIMaster::configureHook(){
  std::cout << "SPIMaster configured !" <<std::endl;
  return true;
}

bool SPIMaster::startHook(){
  std::cout << "SPIMaster started !" <<std::endl;
  return true;
}

void SPIMaster::updateHook(){
  std::cout << "SPIMaster executes updateHook !" <<std::endl;
}

void SPIMaster::stopHook() {
  std::cout << "SPIMaster executes stopping !" <<std::endl;
}

void SPIMaster::cleanupHook() {
  std::cout << "SPIMaster cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(SPIMaster)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(SPIMaster)
