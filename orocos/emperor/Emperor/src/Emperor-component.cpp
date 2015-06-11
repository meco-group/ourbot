#include "Emperor-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Emperor::Emperor(std::string const& name) : TaskContext(name){
  this->ports()->addPort("fsm_event_outport",_fsm_event_outport);

  this->ports()->addEventPort("current_state_evport",_current_state_evport);
  this->addOperation("setStateRun", &Emperor::setStateRun, this);
  this->addOperation("setStateIdle", &Emperor::setStateIdle, this);
  this->addOperation("setStateIdentify", &Emperor::setStateIdentify, this);

  std::cout << "Emperor constructed !" <<std::endl;
}

bool Emperor::configureHook(){

  std::cout << "Emperor configured !" <<std::endl;
  return true;
}

bool Emperor::startHook(){
  std::cout << "Emperor started !" <<std::endl;
  return true;
}

void Emperor::updateHook(){
  std::string result;
  _current_state_evport.read(result);
  std::cout << "Ourbot state: " << result <<std::endl;
}

void Emperor::stopHook() {
  std::cout << "Emperor executes stopping !" <<std::endl;
}

void Emperor::cleanupHook() {
  std::cout << "Emperor cleaning up !" <<std::endl;
}

void Emperor::setStateRun() {
  _fsm_event_outport.write("e_run");
}

void Emperor::setStateIdle() {
  _fsm_event_outport.write("e_idle");
}

void Emperor::setStateIdentify() {
  _fsm_event_outport.write("e_identify");
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Emperor)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Emperor)
