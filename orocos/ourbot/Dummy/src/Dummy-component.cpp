#include "Dummy-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Dummy::Dummy(std::string const& name) : TaskContext(name), _dummy_vec_in(3), _dummy_vec_out(3){
  ports()->addPort("dummy_in_port", _dummy_in_port).doc("Dummy input port");
  ports()->addPort("dummy_out_port", _dummy_out_port).doc("Dummy output port");
  ports()->addPort("dummy_vec_in_port", _dummy_vec_in_port).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_out_port", _dummy_vec_out_port).doc("Dummy output vector port");
  std::cout << "Dummy constructed !" <<std::endl;
}

bool Dummy::configureHook(){
  std::cout << "Dummy configured !" <<std::endl;
  return true;
}

bool Dummy::startHook(){
  _dummy_in = 0;
  _dummy_out = 0;
  _dummy_vec_in.resize(3);
  _dummy_vec_out.resize(3);
  std::cout << "Dummy started !" <<std::endl;
  return true;
}

void Dummy::updateHook(){
  if (_dummy_in_port.read(_dummy_in) == RTT::NewData){
    std::cout << "dummy_in: " << _dummy_in << std::endl;
  }
  if (_dummy_vec_in_port.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  // std::cout << "dummy_vec_out: (" << _dummy_vec_out[0] << "," << _dummy_vec_out[1] << "," << _dummy_vec_out[2] << ")" <<std::endl;
  _dummy_out_port.write(_dummy_out);
  _dummy_vec_out_port.write(_dummy_vec_out);

  _dummy_out = _dummy_out + 1.0;
  _dummy_vec_out[0] = _dummy_out;
}

void Dummy::stopHook() {
  std::cout << "Dummy executes stopping !" <<std::endl;
}

void Dummy::cleanupHook() {
  std::cout << "Dummy cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Dummy)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Dummy)
