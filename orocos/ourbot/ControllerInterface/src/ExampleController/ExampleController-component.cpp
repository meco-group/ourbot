#include "ExampleController-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


ExampleController::ExampleController(std::string const& name) : ControllerInterface(name), _error(3){
  _kp[0] = 1.;
  _kp[1] = 1.;
  _kp[2] = 1.;
  _ki[0] = 1.;
  _ki[1] = 1.;
  _ki[2] = 1.;
}

bool ExampleController::initialize(){
  std::fill(_error.begin(), _error.end(), 0.);
  return true;
}

bool ExampleController::controlUpdate(){
  std::vector<double> est_pose          = getEstPose();
  std::vector<double> reference         = getRefPose();
  std::vector<double> reference_ffw     = getRefFfw();
  std::vector<double> set_velocity      = getSetVelocity();
  double sample_rate                    = getSampleRate();

  // Error
  std::vector<double> error(3);
  for (int i=0; i<3; i++){
    error[i] = reference[i] - est_pose[i];
  }

  // PI controller
  std::vector<double> set_velocity_new(3);
  for (int i=0; i<3; i++){
    set_velocity_new[i] = set_velocity[i] + _kp[i]*error[i] + (sample_rate*_ki[i] - _kp[i])*_error[i];
  }

  // Velocity feedforward
  for (int i=0; i<3; i++){
    set_velocity_new[i] += reference_ffw[i];
  }

  // Save error
  _error = error;

  // Write velocity setpoint
  setSetVelocity(set_velocity_new);

  return true;
}

ORO_CREATE_COMPONENT(ExampleController);
