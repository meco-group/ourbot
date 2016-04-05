#include "Reference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Reference::Reference(std::string const& name) : TaskContext(name){
  std::cout << "Reference constructed !" <<std::endl;
}

bool Reference::configureHook(){
  std::cout << "Reference configured !" <<std::endl;
  return true;
}

bool Reference::startHook(){
  std::cout << "Reference started !" <<std::endl;
  return true;
}

void Reference::updateHook(){
  std::cout << "Reference executes updateHook !" <<std::endl;
}

void Reference::stopHook() {
  std::cout << "Reference executes stopping !" <<std::endl;
}

void Reference::cleanupHook() {
  std::cout << "Reference cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Reference)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Reference)
