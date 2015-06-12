#include "Configurator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;

Configurator::Configurator(std::string const& name) : TaskContext(name){
  addOperation("getPathLength",&Configurator::getPathLength,this);
  addOperation("getControlSampleRate",&Configurator::getControlSampleRate,this);
  addOperation("getPathUpdSampleRate",&Configurator::getPathUpdSampleRate,this);
  addOperation("getKinLimitations",&Configurator::getKinLimitations,this);
  addOperation("getNrofIR",&Configurator::getNrofIR,this);
  addOperation("getLidarDataLength",&Configurator::getLidarDataLength,this);
  addOperation("getObsDataLength",&Configurator::getObsDataLength,this);

  std::cout << "Configurator constructed !" <<std::endl;
}

bool Configurator::configureHook(){
  std::cout << "Configurator configured !" <<std::endl;
  return true;
}

bool Configurator::startHook(){
  std::cout << "Configurator started !" <<std::endl;
  return true;
}

void Configurator::updateHook(){
  std::cout << "Configurator executes updateHook !" <<std::endl;
}

void Configurator::stopHook() {
  std::cout << "Configurator executes stopping !" <<std::endl;
}

void Configurator::cleanupHook() {
  std::cout << "Configurator cleaning up !" <<std::endl;
}

int Configurator::getPathLength(){
  return 10;
}
int Configurator::getControlSampleRate(){
  return 100;
}
int Configurator::getPathUpdSampleRate(){
  return 1;
}
std::vector<double> Configurator::getKinLimitations(){
  std::vector<double> kin_lim(4);
  kin_lim.at(0) = 5.;
  kin_lim.at(1) = -5.;
  kin_lim.at(2) = 1.;
  kin_lim.at(3) = -1.;
  return kin_lim;
}
int Configurator::getNrofIR(){
  return 8;
}
int Configurator::getLidarDataLength(){
  return 100;
}
int Configurator::getObsDataLength(){
  return 100;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Configurator)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Configurator)
