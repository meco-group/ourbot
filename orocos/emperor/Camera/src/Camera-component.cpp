#include "Camera-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Camera::Camera(std::string const& name) : TaskContext(name){
  std::cout << "Camera constructed !" <<std::endl;
}

bool Camera::configureHook(){
  std::cout << "Camera configured !" <<std::endl;
  return true;
}

bool Camera::startHook(){
  std::cout << "Camera started !" <<std::endl;
  return true;
}

void Camera::updateHook(){
  std::cout << "Camera executes updateHook !" <<std::endl;
}

void Camera::stopHook() {
  std::cout << "Camera executes stopping !" <<std::endl;
}

void Camera::cleanupHook() {
  std::cout << "Camera cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Camera)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Camera)
