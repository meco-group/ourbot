#include "Dummy-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Dummy::Dummy(std::string const& name) : TaskContext(name), _dummy_vec_in(3), _dummy_vec_out(3){
  ports()->addPort("dummy_in_port", _dummy_in_port).doc("Dummy input port");
  ports()->addPort("dummy_out_port", _dummy_out_port).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port1", _dummy_vec_out_port1).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port2", _dummy_vec_out_port2).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port3", _dummy_vec_out_port3).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port4", _dummy_vec_out_port4).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port5", _dummy_vec_out_port5).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port6", _dummy_vec_out_port6).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port7", _dummy_vec_out_port7).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port8", _dummy_vec_out_port8).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port9", _dummy_vec_out_port9).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port10", _dummy_vec_out_port10).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port11", _dummy_vec_out_port11).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port12", _dummy_vec_out_port12).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port13", _dummy_vec_out_port13).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port14", _dummy_vec_out_port14).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port15", _dummy_vec_out_port15).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port16", _dummy_vec_out_port16).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port17", _dummy_vec_out_port17).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port18", _dummy_vec_out_port18).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port19", _dummy_vec_out_port19).doc("Dummy output vector port");
  ports()->addPort("dummy_vec_out_port20", _dummy_vec_out_port20).doc("Dummy output vector port");

  ports()->addPort("dummy_vec_in_port1", _dummy_vec_in_port1).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port2", _dummy_vec_in_port2).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port3", _dummy_vec_in_port3).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port4", _dummy_vec_in_port4).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port5", _dummy_vec_in_port5).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port6", _dummy_vec_in_port6).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port7", _dummy_vec_in_port7).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port8", _dummy_vec_in_port8).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port9", _dummy_vec_in_port9).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port10", _dummy_vec_in_port10).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port11", _dummy_vec_in_port11).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port12", _dummy_vec_in_port12).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port13", _dummy_vec_in_port13).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port14", _dummy_vec_in_port14).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port15", _dummy_vec_in_port15).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port16", _dummy_vec_in_port16).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port17", _dummy_vec_in_port17).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port18", _dummy_vec_in_port18).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port19", _dummy_vec_in_port19).doc("Dummy input vector port");
  ports()->addPort("dummy_vec_in_port20", _dummy_vec_in_port20).doc("Dummy input vector port");
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
  if (_dummy_vec_in_port1.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in1: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port2.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in2: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port3.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in3: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port4.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in4: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port5.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in5: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port6.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in6: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port7.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in7: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port8.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in8: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port9.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in9: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port10.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in10: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port11.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in11: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port12.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in12: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port13.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in13: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port14.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in14: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port15.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in15: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port16.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in16: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port17.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in17: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port18.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in18: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port19.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in19: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  if (_dummy_vec_in_port20.read(_dummy_vec_in) == RTT::NewData){
    std::cout << "dummy_vec_in20: (" << _dummy_vec_in[0] << "," << _dummy_vec_in[1] << "," << _dummy_vec_in[2] << ")" << std::endl;
  }
  _dummy_out_port.write(_dummy_out);
  _dummy_vec_out_port1.write(_dummy_vec_out);
  _dummy_vec_out_port2.write(_dummy_vec_out);
  _dummy_vec_out_port3.write(_dummy_vec_out);
  _dummy_vec_out_port4.write(_dummy_vec_out);
  _dummy_vec_out_port5.write(_dummy_vec_out);
  _dummy_vec_out_port6.write(_dummy_vec_out);
  _dummy_vec_out_port7.write(_dummy_vec_out);
  _dummy_vec_out_port8.write(_dummy_vec_out);
  _dummy_vec_out_port9.write(_dummy_vec_out);
  _dummy_vec_out_port10.write(_dummy_vec_out);
  _dummy_vec_out_port11.write(_dummy_vec_out);
  _dummy_vec_out_port12.write(_dummy_vec_out);
  _dummy_vec_out_port13.write(_dummy_vec_out);
  _dummy_vec_out_port14.write(_dummy_vec_out);
  _dummy_vec_out_port15.write(_dummy_vec_out);
  _dummy_vec_out_port16.write(_dummy_vec_out);
  _dummy_vec_out_port17.write(_dummy_vec_out);
  _dummy_vec_out_port18.write(_dummy_vec_out);
  _dummy_vec_out_port19.write(_dummy_vec_out);
  _dummy_vec_out_port20.write(_dummy_vec_out);

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
