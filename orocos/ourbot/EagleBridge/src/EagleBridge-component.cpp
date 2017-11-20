#include "EagleBridge-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

EagleBridge::EagleBridge(std::string const& name) : TaskContext(name){
  std::cout << "EagleBridge constructed !" <<std::endl;
}

bool EagleBridge::configureHook(){
  std::cout << "EagleBridge configured !" <<std::endl;
  return true;
}

bool EagleBridge::startHook(){
  std::cout << "EagleBridge started !" <<std::endl;
  return true;
}

void EagleBridge::updateHook(){
  std::cout << "EagleBridge executes updateHook !" <<std::endl;
}

void EagleBridge::stopHook() {
  std::cout << "EagleBridge executes stopping !" <<std::endl;
}

void EagleBridge::cleanupHook() {
  std::cout << "EagleBridge cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(EagleBridge)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(EagleBridge)
