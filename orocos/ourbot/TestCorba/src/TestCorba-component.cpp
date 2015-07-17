#include "TestCorba-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

TestCorba::TestCorba(std::string const& name) : TaskContext(name, PreOperational){
  ports()->addEventPort("channel1_port", _channel1_port).doc("Channel 1");
  addProperty("datasize",_datasize);
  addProperty("period",_period);
  std::cout << "TestCorba constructed !" <<std::endl;
}

bool TestCorba::configureHook(){
  _readdata = std::vector<double>(_datasize);
  std::cout << "TestCorba configured !" <<std::endl;
  return true;
}

bool TestCorba::startHook(){
  _timestamp = TimeService::Instance()->getTicks();
  _init = 0;
  std::cout << "TestCorba started !" <<std::endl;
  return true;
}

void TestCorba::updateHook(){
  TimeService::ticks prev_timestamp = _timestamp;
  _timestamp = TimeService::Instance()->getTicks();
  _channel1_port.read(_readdata);
  std::cout << _readdata.at(0) << std::endl;
  if (_init>2){
    Seconds prev_time_elapsed = TimeService::Instance()->secondsSince( prev_timestamp );
    Seconds time_elapsed = TimeService::Instance()->secondsSince( _timestamp );
    if ((time_elapsed-prev_time_elapsed-_period) > 0.1*_period){
      log(Warning) << "Jitter exceeded 10% of sample period'" <<endlog();
    }
    // std::cout<<"Duration = " << time_elapsed-prev_time_elapsed << std::endl;
  }
  else{
    _init++;
  }
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
