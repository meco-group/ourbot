#include "Reference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Reference::Reference(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose_sample(3), _ref_velocity_sample(3), _index(0), _new_data(false){
  ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
  ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
  ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");

  ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

  ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
  ports()->addPort("ref_velocity_port", _ref_velocity_port).doc("Velocity reference sample");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("pathupd_sample_rate", _pathupd_sample_rate).doc("Frequency to update the path");

  addOperation("writeSample",&Reference::writeSample, this).doc("Set data sample on output ports");

  _cur_ref_pose_trajectory = _ref_pose_trajectory_1;
  _cur_ref_velocity_trajectory = _ref_velocity_trajectory_1;

  _nxt_ref_pose_trajectory = _ref_pose_trajectory_2;
  _nxt_ref_velocity_trajectory = _ref_velocity_trajectory_2;
}

void Reference::writeSample(){
  std::vector<double> example(3, 0.0);
  _ref_pose_port.write(example);
  _ref_velocity_port.write(example);
}

bool Reference::configureHook(){
  // Compute trajectory length
  _trajectory_length = static_cast<int>(_control_sample_rate/_pathupd_sample_rate);

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _cur_ref_pose_trajectory[i].resize(_trajectory_length);
    _cur_ref_velocity_trajectory[i].resize(_trajectory_length);

    _nxt_ref_pose_trajectory[i].resize(_trajectory_length);
    _nxt_ref_velocity_trajectory[i].resize(_trajectory_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _ref_pose_port.setDataSample(example);
  _ref_velocity_port.setDataSample(example);

  // Reset index & checks
  _index = 0;
  _new_data = false;
  for (int i=0; i<3; i++){
    _got_ref_pose_trajectory[i] = false;
    _got_ref_velocity_trajectory[i] = false;
  }
  return true;
}

bool Reference::startHook(){
  // Check connections
  for(int i=0; i<3; i++){
    _con_ref_pose_trajectory[i] = _ref_pose_trajectory_port[i].connected();
    _con_ref_velocity_trajectory[i] = _ref_velocity_trajectory_port[i].connected();
  }

  std::cout << "Reference started !" <<std::endl;
  return true;
}

void Reference::updateHook(){
  // Check for new data and read ports
  readPorts();

  // Get next sample
  for (int i=0; i<3; i++){
    _ref_pose_sample.at(i) = _cur_ref_pose_trajectory[i].at(_index);
    _ref_velocity_sample.at(i) = _cur_ref_velocity_trajectory[i].at(_index);
  }
  _ref_pose_port.write(_ref_pose_sample);
  _ref_velocity_port.write(_ref_velocity_sample);

  // Update index/trajectory vector
  if( (_index+1) == _trajectory_length){
    if( _new_data){
      // Swap current and next pointers
      std::vector<double>* swap_pose = _cur_ref_pose_trajectory;
      std::vector<double>* swap_velocity = _cur_ref_velocity_trajectory;
      _cur_ref_pose_trajectory = _nxt_ref_pose_trajectory;
      _cur_ref_velocity_trajectory = _nxt_ref_velocity_trajectory;
      _nxt_ref_pose_trajectory = swap_pose;
      _nxt_ref_velocity_trajectory = swap_velocity;

      // Reset index & checks
      _index = 0;
      _new_data = false;
      for (int i=0; i<3; i++){
        _got_ref_pose_trajectory[i] = false;
        _got_ref_velocity_trajectory[i] = false;
      }
    }
    else{
      log(Warning)<<"No new trajectory ! Staying at last sample."<<endlog();
    }
  }
  else
  {
    _index++;
  }
}

void Reference::stopHook() {
  std::cout << "Reference stopped !" <<std::endl;
}

void Reference::readPorts(){
  for(int i=0; i<3; i++){
    std::vector<double> trajectory(_trajectory_length);
    if ( _con_ref_pose_trajectory[i] ){
      if (_ref_pose_trajectory_port[i].read(trajectory) == RTT::NewData){
        _nxt_ref_pose_trajectory[i] = trajectory;
        _got_ref_pose_trajectory[i] = true;
      }
    }
    if ( _con_ref_velocity_trajectory[i] ){
      if (_ref_velocity_trajectory_port[i].read(trajectory) == RTT::NewData){
        _nxt_ref_velocity_trajectory[i] = trajectory;
        _got_ref_velocity_trajectory[i] = true;
      }
    }
  }
  _new_data = true;
  for(int i=0; i<3; i++){
    if ( _con_ref_pose_trajectory[i] != _got_ref_pose_trajectory[i]){ _new_data = false; }
    if ( _con_ref_velocity_trajectory[i] != _got_ref_velocity_trajectory[i]){ _new_data = false; }
  }
}

ORO_CREATE_COMPONENT_LIBRARY();
ORO_LIST_COMPONENT_TYPE(Reference);
