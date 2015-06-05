#include "Test-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Test::Test(std::string const& name) : TaskContext(name){
  std::cout << "Test constructed !" <<std::endl;
}

bool Test::configureHook(){
  std::cout << "Test configured !" <<std::endl;
  return true;
}

bool Test::startHook(){
  std::cout << "Test started !" <<std::endl;
  return true;
}

void Test::updateHook(){
  std::cout << "Test executes updateHook !" <<std::endl;
}

void Test::stopHook() {
  std::cout << "Test executes stopping !" <<std::endl;
}

void Test::cleanupHook() {
  std::cout << "Test cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Test)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Test)
