#include "GamePad-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

GamePad::GamePad(std::string const& name) : TaskContext(name){
  this->ports()->addPort("gamepad_laxis_port", _gamepad_laxis_port).doc("X and Y value for left axis");
  this->ports()->addPort("gamepad_raxis_port", _gamepad_raxis_port).doc("X and Y value for right axis");
  this->ports()->addPort("gamepad_A_port", _gamepad_A_port).doc("Bool for A button");
  this->ports()->addPort("gamepad_B_port", _gamepad_B_port).doc("Bool for B button");
  this->ports()->addPort("gamepad_X_port", _gamepad_X_port).doc("Bool for X button");
  this->ports()->addPort("gamepad_Y_port", _gamepad_Y_port).doc("Bool for Y button");
  this->ports()->addPort("gamepad_back_port", _gamepad_back_port).doc("Bool for back button");
  this->ports()->addPort("gamepad_start_port", _gamepad_start_port).doc("Bool for start button");
  this->ports()->addPort("gamepad_logitech_port", _gamepad_logitech_port).doc("Bool for logitech button");
  this->ports()->addPort("gamepad_laxisbutton_port", _gamepad_laxisbutton_port).doc("Bool for button of left axis");
  this->ports()->addPort("gamepad_raxisbutton_port", _gamepad_raxisbutton_port).doc("Bool for button of right axis");
  this->ports()->addPort("gamepad_up_port", _gamepad_up_port).doc("Bool for up button");
  this->ports()->addPort("gamepad_down_port", _gamepad_down_port).doc("Bool for down button");
  this->ports()->addPort("gamepad_left_port", _gamepad_left_port).doc("Bool for left button");
  this->ports()->addPort("gamepad_right_port", _gamepad_right_port).doc("Bool for right button");
  this->ports()->addPort("gamepad_lb_port", _gamepad_lb_port).doc("Bool for lb button");
  this->ports()->addPort("gamepad_rb_port", _gamepad_rb_port).doc("Bool for rb button");
  this->ports()->addPort("gamepad_lt_port", _gamepad_lt_port).doc("Double for lt button");
  this->ports()->addPort("gamepad_rt_port", _gamepad_rt_port).doc("Double for rt button");

  addProperty("usb_port_name",_usb_port_name).doc("The usb port name.");
  addOperation("setUSBPortName", &USBInterface::setUSBPortName, this).doc("Set the _usb_port_name property.");
}

bool GamePad::configureHook(){
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(2, 0.0);
  _gamepad_laxis_port.setDataSample(example);
  _gamepad_raxis_port.setDataSample(example);

  #ifndef GAMEPAD_TESTFLAG
    return true;
  #else
    _usb_port_name = "/dev/input/js0";
    return setPeriod(0.01);
  #endif //GAMEPAD_TESTFLAG
}

bool GamePad::startHook(){
  if(connectGamePad()){
    return true;
  }
  else{
    RTT::log(RTT::Warning) << "Cannot connect to GamePad" << RTT::endlog();
    return false;
  }
}

bool GamePad::connectGamePad(){
  _usb_fd = open(_usb_port_name, O_RDONLY );
  return _usb_fd>0;
}

void GamePad::setUSBPortName()
void GamePad::updateHook(){
  std::cout << "GamePad executes updateHook !" <<std::endl;
}

void GamePad::stopHook() {
  std::cout << "GamePad executes stopping !" <<std::endl;
}

void GamePad::cleanupHook() {
  std::cout << "GamePad cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(GamePad)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(GamePad)
