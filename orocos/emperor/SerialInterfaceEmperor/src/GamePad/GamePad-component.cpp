#include "GamePad-component.hpp"
#include <iostream>
#include <math.h>

GamePad::GamePad(std::string const& name):USBInterface(name), _gamepad_laxis(2), _gamepad_raxis(2)
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

  _gamepad_A = false;
  _gamepad_B = false;
  _gamepad_X = false;
  _gamepad_Y = false;
  _gamepad_back = false;
  _gamepad_start = false;
  _gamepad_logitech = false;
  _gamepad_laxisbutton = false;
  _gamepad_raxisbutton = false;
  _gamepad_up = false;
  _gamepad_down = false;
  _gamepad_left = false;
  _gamepad_right = false;
  _gamepad_lb = false;
  _gamepad_rb = false;
  _gamepad_lt = 0.0;
  _gamepad_rt = 0.0;
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
  uint8_t buffer[10*sizeof(_event)];
  unsigned int k = 0;
  int numbytes = readBytes(buffer,10*sizeof(_event));

  while(numbytes>0){
    GAMEPAD_DEBUG_PRINT("Bytes received: " << numbytes)
    memcpy(&_event,buffer+k,sizeof(_event));

    if (_event.type == GAMEPAD_EVENT_BUTTON) {
      decodeButtons();
    }
    if (_event.type == GAMEPAD_EVENT_AXIS) {
      decodeAxes();
    }

    k += sizeof(_event);
    numbytes -= sizeof(_event);
  }
  writePorts();
}

void GamePad::writePorts()
{
  // buttons
  _gamepad_A_port.write(_gamepad_A);
  _gamepad_B_port.write(_gamepad_B);
  _gamepad_X_port.write(_gamepad_X);
  _gamepad_Y_port.write(_gamepad_Y);
  _gamepad_lb_port.write(_gamepad_lb);
  _gamepad_rb_port.write(_gamepad_rb);
  _gamepad_back_port.write(_gamepad_back);
  _gamepad_start_port.write(_gamepad_start);
  _gamepad_logitech_port.write(_gamepad_logitech);
  _gamepad_laxisbutton_port.write(_gamepad_laxisbutton);
  _gamepad_raxisbutton_port.write(_gamepad_raxisbutton);
  // axes
  _gamepad_laxis_port.write(_gamepad_laxis);
  _gamepad_raxis_port.write(_gamepad_raxis);
  _gamepad_lt_port.write(_gamepad_lt);
  _gamepad_rt_port.write(_gamepad_rt);

  _gamepad_left_port.write(_gamepad_left);
  _gamepad_right_port.write(_gamepad_right);
  _gamepad_up_port.write(_gamepad_up);
  _gamepad_down_port.write(_gamepad_down);
}

void GamePad::decodeButtons()
{
  switch(_event.number)
  {
    case BUTTON_A:
      _gamepad_A = _event.value;
      break;
    case BUTTON_B:
      _gamepad_B = _event.value;
      break;
    case BUTTON_X:
      _gamepad_X = _event.value;
      break;
    case BUTTON_Y:
      _gamepad_Y = _event.value;
      break;
    case BUTTON_LB:
      _gamepad_lb = _event.value;
      break;
    case BUTTON_RB:
      _gamepad_rb = _event.value;
      break;
    case BUTTON_BCK:
      _gamepad_back = _event.value;
      break;
    case BUTTON_STRT:
      _gamepad_start = _event.value;
      break;
    case BUTTON_LOG:
      _gamepad_logitech = _event.value;
      break;
    case BUTTON_AXISL:
      _gamepad_laxisbutton = _event.value;
      break;
    case BUTTON_AXISR:
      _gamepad_raxisbutton = _event.value;
      break;
  }
}

void GamePad::decodeAxes()
{
  switch(_event.number)
  {
    case AXIS_LEFTX:
      _gamepad_laxis[0]   = transformData(_event.value);
      break;
    case AXIS_LEFTY:
      _gamepad_laxis[1]   = transformData(_event.value);
      break;
    case AXIS_RIGHTX:
      _gamepad_raxis[0]   = transformData(_event.value);
      break;
    case AXIS_RIGHTY:
      _gamepad_raxis[1]   = transformData(_event.value);
      break;
    case AXIS_LT:
      _gamepad_lt = transformData(_event.value);
      break;
    case AXIS_RT:
      _gamepad_rt = transformData(_event.value);
      break;
    case AXIS_LR:
      _gamepad_left = (_event.value == -MAXVALUE);
      _gamepad_right = (_event.value == MAXVALUE);
      break;
    case AXIS_UD:
      _gamepad_up = (_event.value == -MAXVALUE);
      _gamepad_down = (_event.value == MAXVALUE);
      break;
  }
}

// void GamePad::decodeButtons()
// {
//   switch(_event.number)
//   {
//     case BUTTON_A:
//       _gamepad_A_port.write(_event.value);
//       break;
//     case BUTTON_B:
//       _gamepad_B_port.write(_event.value);
//       break;
//     case BUTTON_X:
//       _gamepad_X_port.write(_event.value);
//       break;
//     case BUTTON_Y:
//       _gamepad_Y_port.write(_event.value);
//       break;
//     case BUTTON_LB:
//       _gamepad_lb_port.write(_event.value);
//       break;
//     case BUTTON_RB:
//       _gamepad_rb_port.write(_event.value);
//       break;
//     case BUTTON_BCK:
//       _gamepad_back_port.write(_event.value);
//       break;
//     case BUTTON_STRT:
//       _gamepad_start_port.write(_event.value);
//       break;
//     case BUTTON_LOG:
//       _gamepad_logitech_port.write(_event.value);
//       break;
//     case BUTTON_AXISL:
//       _gamepad_laxisbutton_port.write(_event.value);
//       break;
//     case BUTTON_AXISR:
//       _gamepad_raxisbutton_port.write(_event.value);
//       break;
//   }
// }

// void GamePad::decodeAxes()
// {
//   switch(_event.number)
//   {
//     case AXIS_LEFTX:
//       _laxis[0]   = transformData(_event.value);
//       _gamepad_laxis_port.write(_laxis);
//       break;
//     case AXIS_LEFTY:
//       _laxis[1]   = transformData(_event.value);
//       _gamepad_laxis_port.write(_laxis);
//       break;
//     case AXIS_RIGHTX:
//       _raxis[0]   = transformData(_event.value);
//       _gamepad_raxis_port.write(_raxis);
//       break;
//     case AXIS_RIGHTY:
//       _raxis[1]   = transformData(_event.value);
//       _gamepad_raxis_port.write(_raxis);
//       break;
//     case AXIS_LT:
//       _gamepad_lt_port.write(transformData(_event.value));
//       break;
//     case AXIS_RT:
//       _gamepad_rt_port.write(transformData(_event.value));
//       break;
//     case AXIS_LR:
//       _gamepad_left_port.write(_event.value == -MAXVALUE);
//       _gamepad_right_port.write(_event.value == MAXVALUE);
//       break;
//     case AXIS_UD:
//       _gamepad_up_port.write(_event.value == -MAXVALUE);
//       _gamepad_down_port.write(_event.value == MAXVALUE);
//       break;
//   }
// }

double GamePad::transformData(int value)
{
  return ((double)value)*100./MAXVALUE;
}

void GamePad::stopHook()
{
  USBInterface::stopHook();
}


ORO_LIST_COMPONENT_TYPE(GamePad)
