#include "PathGeneratorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

PathGeneratorInterface::PathGeneratorInterface(std::string const& name) : TaskContext(name){
  std::cout << "PathGeneratorInterface constructed !" <<std::endl;
}

bool PathGeneratorInterface::configureHook(){
  std::cout << "PathGeneratorInterface configured !" <<std::endl;
  return true;
}

bool PathGeneratorInterface::startHook(){
  std::cout << "PathGeneratorInterface started !" <<std::endl;
  return true;
}

void PathGeneratorInterface::updateHook(){
  std::cout << "PathGeneratorInterface executes updateHook !" <<std::endl;
}

void PathGeneratorInterface::stopHook() {
  std::cout << "PathGeneratorInterface executes stopping !" <<std::endl;
}

void PathGeneratorInterface::cleanupHook() {
  std::cout << "PathGeneratorInterface cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(PathGeneratorInterface)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(PathGeneratorInterface)
