#include "TestCorba-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

TestCorba::TestCorba(std::string const& name) : TaskContext(name, PreOperational), _cnt(0.){
  ports()->addPort("channel1_port", _channel1_port).doc("Channel 1");
  addProperty("datasize",_datasize);
  std::cout << "TestCorba constructed !" <<std::endl;
}

bool TestCorba::configureHook(){
  std::vector<double> sample(_datasize);
  _writevalue =  std::vector<double>(_datasize);
  _channel1_port.setDataSample(sample);
  std::cout << "TestCorba configured !" <<std::endl;
  return true;
}

bool TestCorba::startHook(){

  std::cout << "TestCorba started !" <<std::endl;
  return true;
}

void TestCorba::updateHook(){
  _writevalue.at(0) = _cnt;
  _cnt = _cnt + 1.;
  _channel1_port.write(_writevalue);
  std::cout << "TestCorba executes updateHook !" <<std::endl;
}

void TestCorba::stopHook() {
  std::cout << "TestCorba executes stopping !" <<std::endl;
}

void TestCorba::cleanupHook() {
  std::cout << "TestCorba cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(TestCorba)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(TestCorba)
