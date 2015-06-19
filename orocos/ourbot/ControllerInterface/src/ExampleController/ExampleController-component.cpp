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
  std::vector<double> ref_pose          = getRefPose();
  std::vector<double> ref_ffw           = getRefFfw();
  std::vector<double> cmd_velocity      = getCmdVelocity();
  double control_sample_rate            = getControlSampleRate();

  // Error
  std::vector<double> error(3);
  for (int i=0; i<3; i++){
    error[i] = ref_pose[i] - est_pose[i];
  }

  // PI controller
  std::vector<double> cmd_velocity_new(3);
  for (int i=0; i<3; i++){
    cmd_velocity_new[i] = cmd_velocity[i] + _kp[i]*error[i] + ((1./control_sample_rate)*_ki[i] - _kp[i])*_error[i];
  }

  // Velocity feedforward
  for (int i=0; i<3; i++){
    cmd_velocity_new[i] += ref_ffw[i];
  }

  // Save error
  _error = error;

  // Write velocity setpoint
  setCmdVelocity(cmd_velocity_new);

  std::cout<<"Ref pose: ("<<ref_pose.at(0)<<","<<ref_pose.at(1)<<","<<ref_pose.at(2)<<")"<<std::endl;
  std::cout<<"Est pose: ("<<est_pose.at(0)<<","<<est_pose.at(1)<<","<<est_pose.at(2)<<")"<<std::endl;
  std::cout<<"Cmd vel: ("<<cmd_velocity_new.at(0)<<","<<cmd_velocity_new.at(1)<<","<<cmd_velocity_new.at(2)<<")"<<std::endl;

  return true;
}

ORO_LIST_COMPONENT_TYPE(ExampleController);
