#include "GamePad-component.hpp"
#include <iostream>
#include <math.h>

GamePad::GamePad(std::string const& name):USBInterface(name), _laxis(2), _raxis(2)
{
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
}

bool GamePad::configureHook()
{
  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(2, 0.0);
  _gamepad_laxis_port.setDataSample(example);
  _gamepad_raxis_port.setDataSample(example);

#ifndef GAMEPAD_TESTFLAG
  return true;
#else
  std::cout << "changed usb_port_name to "<< _usb_port_name << std::endl;
  _usb_port_name = "/dev/input/js0";
  return setPeriod(0.01);
#endif //GAMEPAD_TESTFLAG
}

bool GamePad::startHook()
{
  setReadWriteOptions(O_RDONLY);
  return USBInterface::startHook();
}

void GamePad::updateHook()
{
  int numbytes = readBytes((uint8_t*)&_event, sizeof(_event));

  if(numbytes>0){
    GAMEPAD_DEBUG_PRINT("Bytes received: " << numbytes)

    if (_event.type == GAMEPAD_EVENT_BUTTON) {
      decodeButtons();
    }
    if (_event.type == GAMEPAD_EVENT_AXIS) {
      decodeAxes();
    }
  }
}

void GamePad::decodeButtons()
{
  switch(_event.number)
  {
    case BUTTON_A:
      _gamepad_A_port.write(_event.value);
      break;
    case BUTTON_B:
      _gamepad_B_port.write(_event.value);
      break;
    case BUTTON_X:
      _gamepad_X_port.write(_event.value);
      break;
    case BUTTON_Y:
      _gamepad_Y_port.write(_event.value);
      break;
    case BUTTON_LB:
      _gamepad_lb_port.write(_event.value);
      break;
    case BUTTON_RB:
      _gamepad_rb_port.write(_event.value);
      break;
    case BUTTON_BCK:
      _gamepad_back_port.write(_event.value);
      break;
    case BUTTON_STRT:
      _gamepad_start_port.write(_event.value);
      break;
    case BUTTON_LOG:
      _gamepad_logitech_port.write(_event.value);
      break;
    case BUTTON_AXISL:
      _gamepad_laxisbutton_port.write(_event.value);
      break;
    case BUTTON_AXISR:
      _gamepad_raxisbutton_port.write(_event.value);
      break;
  }
}

void GamePad::decodeAxes()
{
  switch(_event.number)
  {
    case AXIS_LEFTX:
      _laxis[0]   = transformData(_event.value);
      _gamepad_laxis_port.write(_laxis);
      break;
    case AXIS_LEFTY:
      _laxis[1]   = transformData(_event.value);
      _gamepad_laxis_port.write(_laxis);
      break;
    case AXIS_RIGHTX:
      _raxis[0]   = transformData(_event.value);
      _gamepad_raxis_port.write(_raxis);
      break;
    case AXIS_RIGHTY:
      _raxis[1]   = transformData(_event.value);
      _gamepad_raxis_port.write(_raxis);
      break;
    case AXIS_LT:
      _gamepad_lt_port.write(transformData(_event.value));
      break;
    case AXIS_RT:
      _gamepad_rt_port.write(transformData(_event.value));
      break;
    case AXIS_LR:
      _gamepad_left_port.write(_event.value == -MAXVALUE);
      _gamepad_right_port.write(_event.value == MAXVALUE);
      break;
    case AXIS_UD:
      _gamepad_up_port.write(_event.value == -MAXVALUE);
      _gamepad_down_port.write(_event.value == MAXVALUE);
      break;
  }
}

double GamePad::transformData(int value)
{
  return ((double)value)*100./MAXVALUE;
}

void GamePad::stopHook()
{
  USBInterface::stopHook();
}


ORO_LIST_COMPONENT_TYPE(GamePad)
