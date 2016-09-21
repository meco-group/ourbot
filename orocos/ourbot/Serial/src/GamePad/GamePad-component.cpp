#include "GamePad-component.hpp"
#include <iostream>
#include <math.h>

GamePad::GamePad(std::string const& name):USBInterface(name),
_gamepad_laxis(2), _gamepad_raxis(2), _cmd_velocity(3), _cmd_velocity_prev(3),
_enable_velcmd(false), _filter_state(3, 0.0), _filter_bandwidth(2)
{
  ports()->addPort("gamepad_laxis_port", _gamepad_laxis_port).doc("X and Y value for left axis");
  ports()->addPort("gamepad_raxis_port", _gamepad_raxis_port).doc("X and Y value for right axis");
  ports()->addPort("gamepad_A_port", _gamepad_A_port).doc("Bool for A button");
  ports()->addPort("gamepad_B_port", _gamepad_B_port).doc("Bool for B button");
  ports()->addPort("gamepad_X_port", _gamepad_X_port).doc("Bool for X button");
  ports()->addPort("gamepad_Y_port", _gamepad_Y_port).doc("Bool for Y button");
  ports()->addPort("gamepad_back_port", _gamepad_back_port).doc("Bool for back button");
  ports()->addPort("gamepad_start_port", _gamepad_start_port).doc("Bool for start button");
  ports()->addPort("gamepad_logitech_port", _gamepad_logitech_port).doc("Bool for logitech button");
  ports()->addPort("gamepad_laxisbutton_port", _gamepad_laxisbutton_port).doc("Bool for button of left axis");
  ports()->addPort("gamepad_raxisbutton_port", _gamepad_raxisbutton_port).doc("Bool for button of right axis");
  ports()->addPort("gamepad_up_port", _gamepad_up_port).doc("Bool for up button");
  ports()->addPort("gamepad_down_port", _gamepad_down_port).doc("Bool for down button");
  ports()->addPort("gamepad_left_port", _gamepad_left_port).doc("Bool for left button");
  ports()->addPort("gamepad_right_port", _gamepad_right_port).doc("Bool for right button");
  ports()->addPort("gamepad_lb_port", _gamepad_lb_port).doc("Bool for lb button");
  ports()->addPort("gamepad_rb_port", _gamepad_rb_port).doc("Bool for rb button");
  ports()->addPort("gamepad_lt_port", _gamepad_lt_port).doc("Double for lt button");
  ports()->addPort("gamepad_rt_port", _gamepad_rt_port).doc("Double for rt button");
  ports()->addPort("cmd_velocity_port", _cmd_velocity_port).doc("Velocity command for actuator");

  addOperation("setVelocity", &GamePad::setVelocity, this).doc("Set velocity cmd manually");
  addOperation("enableVelocityCmd", &GamePad::enableVelocityCmd, this).doc("Enable writing of cmd_velocity_port");
  addOperation("disableVelocityCmd", &GamePad::disableVelocityCmd, this).doc("Disable writing of cmd_velocity_port");
  addProperty("max_velocity", _max_velocity);
  addProperty("max_omega", _max_omega);
  addProperty("velcmd_sample_rate", _velcmd_sample_rate).doc("Frequency to update velocity commander");
  addProperty("filter_bandwidth", _filter_bandwidth).doc("Bandwidth of low pass filter (Hz)");
  addProperty("enable_LPF", _enable_LPF).doc("Enable low pass filter");

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

  _gamepad_A_prev = false;
  _gamepad_B_prev = false;
  _gamepad_X_prev = false;
  _gamepad_Y_prev = false;
  _gamepad_back_prev = false;
  _gamepad_start_prev = false;
  _gamepad_logitech_prev = false;
  _gamepad_laxisbutton_prev = false;
  _gamepad_raxisbutton_prev = false;
  _gamepad_up_prev = false;
  _gamepad_down_prev = false;
  _gamepad_left_prev = false;
  _gamepad_right_prev = false;
  _gamepad_lb_prev = false;
  _gamepad_rb_prev = false;
  _gamepad_lt_prev = 0.0;
  _gamepad_rt_prev = 0.0;

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
  disableUnknownStuff();
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
  VelocityCommand();
  writePorts();
}

void GamePad::writePorts()
{
  // buttons
  if (_gamepad_A != _gamepad_A_prev){
    _gamepad_A_port.write(_gamepad_A);
    _gamepad_A_prev = _gamepad_A;
  }
  if (_gamepad_B != _gamepad_B_prev){
    _gamepad_B_port.write(_gamepad_B);
    _gamepad_B_prev = _gamepad_B;
  }
  if (_gamepad_X != _gamepad_X_prev){
    _gamepad_X_port.write(_gamepad_X);
    _gamepad_X_prev = _gamepad_X;
  }
  if (_gamepad_Y != _gamepad_Y_prev){
    _gamepad_Y_port.write(_gamepad_Y);
    _gamepad_Y_prev = _gamepad_Y;
  }
  if (_gamepad_lb != _gamepad_lb_prev){
    _gamepad_lb_port.write(_gamepad_lb);
    _gamepad_lb_prev = _gamepad_lb;
  }
  if (_gamepad_rb != _gamepad_rb_prev){
    _gamepad_rb_port.write(_gamepad_rb);
    _gamepad_rb_prev = _gamepad_rb;
  }
  if (_gamepad_back != _gamepad_back_prev){
    _gamepad_back_port.write(_gamepad_back);
    _gamepad_back_prev = _gamepad_back;
  }
  if (_gamepad_start != _gamepad_start_prev){
    _gamepad_start_port.write(_gamepad_start);
    _gamepad_start_prev = _gamepad_start;
  }
  if (_gamepad_logitech != _gamepad_logitech_prev){
    _gamepad_logitech_port.write(_gamepad_logitech);
    _gamepad_logitech_prev = _gamepad_logitech;
  }
  if (_gamepad_laxisbutton != _gamepad_laxisbutton_prev){
    _gamepad_laxisbutton_port.write(_gamepad_laxisbutton);
    _gamepad_laxisbutton_prev = _gamepad_laxisbutton;
  }
  if (_gamepad_raxisbutton != _gamepad_raxisbutton_prev){
    _gamepad_raxisbutton_port.write(_gamepad_raxisbutton);
    _gamepad_raxisbutton_prev = _gamepad_raxisbutton;
  }
  // axes
  if (_gamepad_laxis != _gamepad_laxis_prev){
    _gamepad_laxis_port.write(_gamepad_laxis);
    _gamepad_laxis_prev = _gamepad_laxis;
  }
  if (_gamepad_raxis != _gamepad_raxis_prev){
    _gamepad_raxis_port.write(_gamepad_raxis);
    _gamepad_raxis_prev = _gamepad_raxis;
  }
  if (_gamepad_lt != _gamepad_lt_prev){
    _gamepad_lt_port.write(_gamepad_lt);
    _gamepad_lt_prev = _gamepad_lt;
  }
  if (_gamepad_rt != _gamepad_rt_prev){
    _gamepad_rt_port.write(_gamepad_rt);
    _gamepad_rt_prev = _gamepad_rt;
  }
  if (_gamepad_left != _gamepad_left_prev){
    _gamepad_left_port.write(_gamepad_left);
    _gamepad_left_prev = _gamepad_left;
  }
  if (_gamepad_right != _gamepad_right_prev){
    _gamepad_right_port.write(_gamepad_right);
    _gamepad_right_prev = _gamepad_right;
  }
  if (_gamepad_up != _gamepad_up_prev){
    _gamepad_up_port.write(_gamepad_up);
    _gamepad_up_prev = _gamepad_up;
  }
  if (_gamepad_down != _gamepad_down_prev){
    _gamepad_down_port.write(_gamepad_down);
    _gamepad_down_prev = _gamepad_down;
  }
  // velocity command
  if (_enable_velcmd and (_cmd_velocity != _cmd_velocity_prev)){
    _cmd_velocity_port.write(_cmd_velocity);
    _cmd_velocity_prev = _cmd_velocity;
  }
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
      _gamepad_laxis[0] = transformData(_event.value);
      break;
    case AXIS_LEFTY:
      _gamepad_laxis[1] = transformData(_event.value);
      break;
    case AXIS_RIGHTX:
      _gamepad_raxis[0] = transformData(_event.value);
      break;
    case AXIS_RIGHTY:
      _gamepad_raxis[1] = transformData(_event.value);
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

double GamePad::transformData(int value)
{
  return ((double)value)/MAXVALUE;
}

void GamePad::stopHook()
{
  USBInterface::stopHook();
}

void GamePad::VelocityCommand(){
  std::vector<double> sample(2);
  _cmd_velocity[0] = -treshold(_gamepad_raxis[1])*_max_velocity;
  if (_gamepad_rb == true)
  {
    _cmd_velocity[1] = 0.0;
  }
  else
  {
    _cmd_velocity[1] = -treshold(_gamepad_raxis[0])*_max_velocity;
  }
  _cmd_velocity[2] = -treshold(_gamepad_laxis[0])*_max_omega;
  if (_enable_LPF){
    _cmd_velocity = lowPassFilter(_cmd_velocity);
  }
}

double GamePad::treshold(double data){
  double data_tf;
  if (data > 0.05){
    data_tf = (100./95.)*(data-0.05);
  }
  else if (data < -0.05){
    data_tf = (100./95.)*(data+0.05);
  }
  else{
    data_tf = 0.;
  }
  return data_tf;
}

std::vector<double> GamePad::lowPassFilter(std::vector<double> input) {
  double alpha = (_filter_bandwidth*2*M_PI)/(_velcmd_sample_rate);
  _filter_state[0] = (1.-alpha)*_filter_state[0] + alpha*input[0];
  _filter_state[1] = (1.-alpha)*_filter_state[1] + alpha*input[1];
  _filter_state[2] = (1.-alpha)*_filter_state[2] + alpha*input[2];
  for (int k=0; k<3; k++){
    _filter_state[k] = (_filter_state[k] < 0.05 && _filter_state[k] > -0.05) ? 0.:_filter_state[k];
  }
  return _filter_state;
}

void GamePad::setVelocity(double vx, double vy, double w){
  _cmd_velocity[0] = vx;
  _cmd_velocity[1] = vy;
  _cmd_velocity[2] = w;
}

void GamePad::enableVelocityCmd(){
  _enable_velcmd = true;
}

void GamePad::disableVelocityCmd(){
  _enable_velcmd = false;
}
ORO_LIST_COMPONENT_TYPE(GamePad)
