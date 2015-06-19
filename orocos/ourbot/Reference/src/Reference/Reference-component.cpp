#include "Reference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Reference::Reference(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose_sample(3), _ref_ffw_sample(3), _index(0), _new_data(false){
  ports()->addPort("ref_pose_path_x_port", _ref_pose_path_port[0]).doc("x reference path");
  ports()->addPort("ref_pose_path_y_port", _ref_pose_path_port[1]).doc("y reference path");
  ports()->addPort("ref_pose_path_t_port", _ref_pose_path_port[2]).doc("theta reference path");

  ports()->addPort("ref_ffw_path_x_port", _ref_ffw_path_port[0]).doc("x ffw reference path");
  ports()->addPort("ref_ffw_path_y_port", _ref_ffw_path_port[1]).doc("y ffw reference path");
  ports()->addPort("ref_ffw_path_t_port", _ref_ffw_path_port[2]).doc("theta ffw reference path");


  ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
  ports()->addPort("ref_ffw_port", _ref_ffw_port).doc("Feedforward reference sample");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("pathupd_sample_rate", _pathupd_sample_rate).doc("Frequency to update the path");

  _cur_ref_pose_path      = _ref_pose_path_1;
  _cur_ref_ffw_path       = _ref_ffw_path_1;

  _nxt_ref_pose_path      = _ref_pose_path_2;
  _nxt_ref_ffw_path       = _ref_ffw_path_2;
}

bool Reference::configureHook(){
  // Compute path length
  _path_length = static_cast<int>(_control_sample_rate/_pathupd_sample_rate);

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _cur_ref_pose_path[i].resize(_path_length);
    _cur_ref_ffw_path[i].resize(_path_length);

    _nxt_ref_pose_path[i].resize(_path_length);
    _nxt_ref_ffw_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _ref_pose_port.setDataSample(example);
  _ref_ffw_port.setDataSample(example);

  // Reset index & checks
  _index = 0;
  _new_data = false;
  for (int i=0; i<3; i++){
    _got_ref_pose_path[i]      = false;
    _got_ref_ffw_path[i]       = false;
  }
  return true;
}

bool Reference::startHook(){
  // Check connections
  for(int i=0; i<3; i++){
    _con_ref_pose_path[i]      = _ref_pose_path_port[i].connected();
    _con_ref_ffw_path[i]       = _ref_ffw_path_port[i].connected();
  }

  std::cout << "Reference started !" <<std::endl;
  return true;
}

void Reference::updateHook(){
  // Check for new data and read ports
  readPorts();

  // Get next sample
  for (int i=0; i<3; i++){
    _ref_pose_sample.at(i)      = _cur_ref_pose_path[i].at(_index);
    _ref_ffw_sample.at(i)       = _cur_ref_ffw_path[i].at(_index);
  }
  _ref_pose_port.write(_ref_pose_sample);
  _ref_ffw_port.write(_ref_ffw_sample);

  // Update index/path vector
  if( (_index+1) == _path_length){
    if( _new_data){
      // Swap current and next pointers
      std::vector<double>* swap_pose      = _cur_ref_pose_path;
      std::vector<double>* swap_ffw       = _cur_ref_ffw_path;
      _cur_ref_pose_path     = _nxt_ref_pose_path;
      _cur_ref_ffw_path      = _nxt_ref_ffw_path;
      _nxt_ref_pose_path     = swap_pose;
      _nxt_ref_ffw_path      = swap_ffw;

      // Reset index & checks
      _index = 0;
      _new_data = false;
      for (int i=0; i<3; i++){
        _got_ref_pose_path[i]      = false;
        _got_ref_ffw_path[i]       = false;
      }

      log(Info)<<"Start reading new path !"<<endlog();
    }
    else{
      log(Warning)<<"No new path ! Staying at last sample."<<endlog();
    }
  }
  else
  {
    _index++;
  }
  std::cout << "Reference updated !" <<std::endl;
}

void Reference::stopHook() {
  std::cout << "Reference stopped !" <<std::endl;
}

void Reference::readPorts(){
  for(int i=0; i<3; i++){
    std::vector<double> path(_path_length);
    if ( _con_ref_pose_path[i] ){
      if (_ref_pose_path_port[i].read(path) == RTT::NewData){
        _nxt_ref_pose_path[i] = path;
        _got_ref_pose_path[i] = true;
      }
    }
    if ( _con_ref_ffw_path[i] ){
      if (_ref_ffw_path_port[i].read(path) == RTT::NewData){
        _nxt_ref_ffw_path[i] = path;
        _got_ref_ffw_path[i] = true;
      }
    }
  }
  _new_data = true;
  for(int i=0; i<3; i++){
    if ( _con_ref_pose_path[i]     != _got_ref_pose_path[i]){ _new_data = false; }
    if ( _con_ref_ffw_path[i]      != _got_ref_ffw_path[i]){ _new_data = false; }
  }
}

ORO_CREATE_COMPONENT_LIBRARY();
ORO_LIST_COMPONENT_TYPE(Reference);
