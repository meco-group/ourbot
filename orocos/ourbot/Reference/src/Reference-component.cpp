// #define DEBUG

#include "Reference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>

Reference::Reference(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose_sample(3), _ref_velocity_sample(3), _just_started(true),
    _index1(0), _index2(0), _new_data(false), _offline_trajectory(false){
  ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
  ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
  ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");

  ports()->addPort("ref_pose_trajectory_x_tx_port", _ref_pose_trajectory_tx_port[0]).doc("x reference trajectory for tx over wifi");
  ports()->addPort("ref_pose_trajectory_y_tx_port", _ref_pose_trajectory_tx_port[1]).doc("y reference trajectory for tx over wifi");
  ports()->addPort("ref_pose_trajectory_t_tx_port", _ref_pose_trajectory_tx_port[2]).doc("t reference trajectory for tx over wifi");


  ports()->addPort("predict_shift_port", _predict_shift_port).doc("Trigger for motion planning");

  ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
  ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

  ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
  ports()->addPort("ref_velocity_port", _ref_velocity_port).doc("Velocity reference sample");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("pathupd_sample_rate", _pathupd_sample_rate).doc("Frequency to update the path");
  addProperty("trajectory_path", _trajectory_path).doc("Path of offline trajectory");
  addProperty("repeat_offline_trajectory", _repeat_offline_trajectory).doc("Repeat offline trajectory");

  addOperation("writeSample",&Reference::writeSample, this).doc("Set data sample on output ports");
  addOperation("loadTrajectory",&Reference::loadTrajectory, this).doc("Load offline trajectory");
  addOperation("ready",&Reference::ready, this).doc("Is the reference ready?");

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
  _update_length = int(_control_sample_rate/_pathupd_sample_rate);
  _trajectory_length = 3*_update_length;
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
  _index1 = 0;
  _index2 = 0;
  _new_data = false;
  for (int i=0; i<3; i++){
    _got_ref_pose_trajectory[i] = false;
    _got_ref_velocity_trajectory[i] = false;
  }
  return true;
}

bool Reference::loadTrajectory(){
  std::vector<double> ref_pose_trajectory[3];
  std::vector<double> ref_velocity_trajectory[3];
  std::ifstream file;
  file.open(_trajectory_path.c_str());
  std::string line, str;
  double value;
  if (!file.good()){
    return false;
  }
  while(file.good()){
    std::getline(file, line);
    std::stringstream iss(line);
    for (int k=0; k<6; k++){
      std::getline(iss, str, ',');
      std::stringstream convertor(str);
      convertor >> value;
      if (k < 3){
        ref_pose_trajectory[k].push_back(value);
      } else {
        ref_velocity_trajectory[k-3].push_back(value);
      }
    }
  }
  file.close();
  // last value is rubbish
  for (int k=0; k<3; k++){
    ref_pose_trajectory[k].pop_back();
    ref_velocity_trajectory[k].pop_back();
  }

  for (int k=0; k<3; k++){
    _cur_ref_pose_trajectory[k].resize(ref_pose_trajectory[k].size());
    _cur_ref_pose_trajectory[k] = ref_pose_trajectory[k];
    _cur_ref_velocity_trajectory[k].resize(ref_velocity_trajectory[k].size());
    _cur_ref_velocity_trajectory[k] = ref_velocity_trajectory[k];
  }
  _trajectory_length = ref_pose_trajectory[0].size();
  _offline_trajectory =  true;
  _index1 = 0;
  _index2 = 0;
  for (int k=0; k<3; k++){
    _ref_pose_trajectory_tx_port[k].write(_cur_ref_pose_trajectory[k]);
  }
  return true;
}

bool Reference::ready(){
  return !_just_started;
}

bool Reference::startHook(){
  // Check connections
  for(int i=0; i<3; i++){
    _con_ref_pose_trajectory[i] = _ref_pose_trajectory_port[i].connected();
    _con_ref_velocity_trajectory[i] = _ref_velocity_trajectory_port[i].connected();
    if (!_con_ref_pose_trajectory[i]){
      log(Warning) << "ref_pose_trajectory_port is not connected at index "<<i<<"!" <<endlog();
    }
    if (!_con_ref_velocity_trajectory[i]){
      log(Warning) << "ref_velocity_trajectory_port is not connected at index "<<i<<"!" <<endlog();
    }
  }
  _just_started = true;
  _first_time = true;
  // std::cout << "Reference started !" <<std::endl;
  return true;
}

void Reference::updateHook(){
  if (!_offline_trajectory){
    // Check for new data and read ports
    readPorts();
    // If not retrieved first trajectory yet: do nothing
    std::cout << "just started: " << _just_started << ", new data: " << _new_data << std::endl;
    if (_just_started){
      if (_first_time){
        // trigger motion planning for first time
        _predict_shift_port.write(0);
        _first_time = false;
      }
      return;
    }
    // Update index/trajectory vector
    if(_index1 >= 2*_update_length){
      log(Error)<<"I did not receive a new trajectory for a whole update cycle!"<<endlog();
      error();
    }
    if(_index1 >= _update_length){
      if(_new_data){
        // Swap current and next pointers
        std::vector<double>* swap_pose = _cur_ref_pose_trajectory;
        std::vector<double>* swap_velocity = _cur_ref_velocity_trajectory;
        _cur_ref_pose_trajectory = _nxt_ref_pose_trajectory;
        _cur_ref_velocity_trajectory = _nxt_ref_velocity_trajectory;
        _nxt_ref_pose_trajectory = swap_pose;
        _nxt_ref_velocity_trajectory = swap_velocity;
        // Reset index & checks
        _index2 = _index1%_update_length;
        _index1 = 0;
        _new_data = false;
        for (int i=0; i<3; i++){
          _got_ref_pose_trajectory[i] = false;
          _got_ref_velocity_trajectory[i] = false;
        }
        // put on tx port
        for (int i=0; i<3; i++){
          _ref_pose_trajectory_tx_port[i].write(_cur_ref_pose_trajectory[i]);
        }
      }
      else{
        log(Warning)<<"No new trajectory ! Proceeding with previous trajectory."<<endlog();
      }
    }
    // Retrigger motion planner and send the predict shift which is stored at _index2
    if (_index1 == 0){
      _predict_shift_port.write(_index2);
    }
  } else {
    if (_index2 >= _trajectory_length){
      if (_repeat_offline_trajectory){
      _index1 = 0;
      _index2 = 0;

      } else {
      _index1--;
      _index2--;
      }
    }
  }
  // Get next sample
  for (int i=0; i<3; i++){
    if(fabs(_cur_ref_pose_trajectory[i].at(_index2)) > 1.e-3){
      _ref_pose_sample.at(i) = _cur_ref_pose_trajectory[i].at(_index2);
    } else{
      _ref_pose_sample.at(i) = 0.0;
    }
    if(fabs(_cur_ref_velocity_trajectory[i].at(_index2)) > 1.e-3){
      _ref_velocity_sample.at(i) = _cur_ref_velocity_trajectory[i].at(_index2);
    } else{
      _ref_velocity_sample.at(i) = 0.0;
    }
  }
  _ref_pose_port.write(_ref_pose_sample);
  _ref_velocity_port.write(_ref_velocity_sample);
  #ifdef DEBUG
    std::cout << "index1: " << _index1 << ", index2: " << _index2 << std::endl;
    std::cout << "vel: (" << _ref_velocity_sample[0] <<","<<_ref_velocity_sample[1]<<","<<_ref_velocity_sample[2]<<")" <<std::endl;
    std::cout << "pos: (" << _ref_pose_sample[0] <<","<<_ref_pose_sample[1]<<_ref_pose_sample[2]<<")" <<std::endl;
  #endif
  // Update indices
  _index1++;
  _index2++;
}

void Reference::stopHook() {
  // std::cout << "Reference stopped !" <<std::endl;
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
  if (_new_data && _just_started){
    _just_started = false;
    _new_data = false;
    // Swap current and next pointers
    std::vector<double>* swap_pose = _cur_ref_pose_trajectory;
    std::vector<double>* swap_velocity = _cur_ref_velocity_trajectory;
    _cur_ref_pose_trajectory = _nxt_ref_pose_trajectory;
    _cur_ref_velocity_trajectory = _nxt_ref_velocity_trajectory;
    _nxt_ref_pose_trajectory = swap_pose;
    _nxt_ref_velocity_trajectory = swap_velocity;
    for (int i=0; i<3; i++){
      _got_ref_pose_trajectory[i] = false;
      _got_ref_velocity_trajectory[i] = false;
    }
  }
}

ORO_CREATE_COMPONENT_LIBRARY();
ORO_LIST_COMPONENT_TYPE(Reference);
