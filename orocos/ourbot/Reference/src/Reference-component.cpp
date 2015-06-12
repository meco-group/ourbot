#include "Reference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Reference::Reference(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose_sample(3), _ref_ffw_sample(3), _ref_relposeA_sample(3), _ref_relposeB_sample(3), _new_data(false), _index(0){
  ports()->addPort("ref_pose_path_x_inport", _ref_pose_path_inport[0]).doc("x reference path");
  ports()->addPort("ref_pose_path_y_inport", _ref_pose_path_inport[1]).doc("y reference path");
  ports()->addPort("ref_pose_path_t_inport", _ref_pose_path_inport[2]).doc("theta reference path");

  ports()->addPort("ref_ffw_path_x_inport", _ref_ffw_path_inport[0]).doc("x ffw reference path");
  ports()->addPort("ref_ffw_path_y_inport", _ref_ffw_path_inport[1]).doc("y ffw reference path");
  ports()->addPort("ref_ffw_path_t_inport", _ref_ffw_path_inport[2]).doc("theta ffw reference path");

  ports()->addPort("ref_relposeA_path_x_inport", _ref_relposeA_path_inport[0]).doc("x relpose reference path wrt neighbour A");
  ports()->addPort("ref_relposeA_path_y_inport", _ref_relposeA_path_inport[1]).doc("y relpose reference path wrt neighbour A");
  ports()->addPort("ref_relposeA_path_t_inport", _ref_relposeA_path_inport[2]).doc("theta relpose reference path wrt neighbour A");

  ports()->addPort("ref_relposeB_path_x_inport", _ref_relposeB_path_inport[0]).doc("x relpose reference path wrt neighbour B");
  ports()->addPort("ref_relposeB_path_y_inport", _ref_relposeB_path_inport[1]).doc("y relpose reference path wrt neighbour B");
  ports()->addPort("ref_relposeB_path_t_inport", _ref_relposeB_path_inport[2]).doc("theta relpose reference path wrt neighbour B");

  ports()->addPort("ref_pose_outport", _ref_pose_outport).doc("Pose reference sample");
  ports()->addPort("ref_ffw_outport", _ref_ffw_outport).doc("Feedforward reference sample");
  ports()->addPort("ref_relposeA_outport", _ref_relposeA_outport).doc("Relative pose reference sample wrt neighbour A");
  ports()->addPort("ref_relposeB_outport", _ref_relposeB_outport).doc("Relative pose reference sample wrt neighbour B");

  _cur_ref_pose_path      = _ref_pose_path_1;
  _cur_ref_ffw_path       = _ref_ffw_path_1;
  _cur_ref_relposeA_path  = _ref_relposeA_path_1;
  _cur_ref_relposeB_path  = _ref_relposeB_path_1;

  _nxt_ref_pose_path      = _ref_pose_path_2;
  _nxt_ref_ffw_path       = _ref_ffw_path_2;
  _nxt_ref_relposeA_path  = _ref_relposeA_path_2;
  _nxt_ref_relposeB_path  = _ref_relposeB_path_2;
}

bool Reference::configureHook(){
  // Get configurator component and get sample rate
  TaskContext* configurator = getPeer("configurator");
  if (configurator==NULL){
    log(Error) << "No peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getPathLength = configurator->getOperation("getPathLength");
  if (!getPathLength.ready()){
    log(Error) << "No operation getPathLength of peer configurator !" <<endlog();
    return false;
  }
  _path_length = getPathLength();

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _cur_ref_pose_path[i].resize(_path_length);
    _cur_ref_ffw_path[i].resize(_path_length);
    _cur_ref_relposeA_path[i].resize(_path_length);
    _cur_ref_relposeB_path[i].resize(_path_length);

    _nxt_ref_pose_path[i].resize(_path_length);
    _nxt_ref_ffw_path[i].resize(_path_length);
    _nxt_ref_relposeB_path[i].resize(_path_length);
    _nxt_ref_relposeA_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _ref_pose_outport.setDataSample(example);
  _ref_ffw_outport.setDataSample(example);
  _ref_relposeA_outport.setDataSample(example);
  _ref_relposeB_outport.setDataSample(example);

  // Reset index & checks
  _index = 0;
  _new_data = false;
  for (int i=0; i<3; i++){
    _got_ref_pose_path[i]      = false;
    _got_ref_ffw_path[i]       = false;
    _got_ref_relposeA_path[i]  = false;
    _got_ref_relposeB_path[i]  = false;
  }
  return true;
}

bool Reference::startHook(){
  // Check connections
  for(int i=0; i<3; i++){
    _con_ref_pose_path[i]      = _ref_pose_path_inport[i].connected();
    _con_ref_ffw_path[i]       = _ref_ffw_path_inport[i].connected();
    _con_ref_relposeA_path[i]  = _ref_relposeA_path_inport[i].connected();
    _con_ref_relposeB_path[i]  = _ref_relposeB_path_inport[i].connected();
  }

  std::cout << "Reference started !" <<std::endl;
  return true;
}

void Reference::updateHook(){
  // Check for new data and read ports
  void readPorts();

  // Get next sample
  for (int i=0; i<3; i++){
    _ref_pose_sample.at(i)      = _cur_ref_pose_path[i].at(_index);
    _ref_ffw_sample.at(i)       = _cur_ref_ffw_path[i].at(_index);
    _ref_relposeA_sample.at(i)  = _cur_ref_relposeA_path[i].at(_index);
    _ref_relposeB_sample.at(i)  = _cur_ref_relposeB_path[i].at(_index);
  }

  _ref_pose_outport.write(_ref_pose_sample);
  _ref_ffw_outport.write(_ref_ffw_sample);
  _ref_relposeA_outport.write(_ref_relposeA_sample);
  _ref_relposeB_outport.write(_ref_relposeB_sample);

  // Update index/path vector
  if( (_index+1) == _path_length){
    if( _new_data){
      // Swap current and next pointers
      std::vector<double>* swap_pose      = _cur_ref_pose_path;
      std::vector<double>* swap_ffw       = _cur_ref_ffw_path;
      std::vector<double>* swap_relposeA  = _cur_ref_relposeA_path;
      std::vector<double>* swap_relposeB  = _cur_ref_relposeB_path;
      _cur_ref_pose_path     = _nxt_ref_pose_path;
      _cur_ref_ffw_path      = _nxt_ref_ffw_path;
      _cur_ref_relposeA_path = _nxt_ref_relposeA_path;
      _cur_ref_relposeB_path = _nxt_ref_relposeB_path;
      _nxt_ref_pose_path     = swap_pose;
      _nxt_ref_ffw_path      = swap_ffw;
      _nxt_ref_relposeA_path = swap_relposeA;
      _nxt_ref_relposeB_path = swap_relposeB;

      // Reset index & checks
      _index = 0;
      _new_data = false;
      for (int i=0; i<3; i++){
        _got_ref_pose_path[i]      = false;
        _got_ref_ffw_path[i]       = false;
        _got_ref_relposeA_path[i]  = false;
        _got_ref_relposeB_path[i]  = false;
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
}

void Reference::stopHook() {

  std::cout << "Reference stopped !" <<std::endl;
}

void Reference::readPorts(){
  for(int i=0; i<3; i++){
    std::vector<double> path(_path_length);
    if ( _con_ref_pose_path[i] ){
      if (_ref_pose_path_inport[i].read(path) == RTT::NewData){
        _nxt_ref_pose_path[i] = path;
        _got_ref_pose_path[i] = true;
      }
    }
    if ( _con_ref_ffw_path[i] ){
      if (_ref_ffw_path_inport[i].read(path) == RTT::NewData){
        _nxt_ref_ffw_path[i] = path;
        _got_ref_ffw_path[i] = true;
      }
    }
    if ( _con_ref_relposeA_path[i] ){
      if (_ref_relposeA_path_inport[i].read(path) == RTT::NewData){
        _nxt_ref_relposeA_path[i] = path;
        _got_ref_relposeA_path[i] = true;
      }
    }
    if ( _con_ref_relposeB_path[i] ){
      if (_ref_relposeB_path_inport[i].read(path) == RTT::NewData){
        _nxt_ref_relposeB_path[i] = path;
        _got_ref_relposeB_path[i] = true;
      }
    }
  }
  _new_data = true;
  for(int i=0; i<3; i++){
    if ( _con_ref_pose_path[i]     != _got_ref_pose_path[i]){ _new_data = false; }
    if ( _con_ref_ffw_path[i]      != _got_ref_ffw_path[i]){ _new_data = false; }
    if ( _con_ref_relposeA_path[i] != _got_ref_relposeA_path[i]){ _new_data = false; }
    if ( _con_ref_relposeB_path[i] != _got_ref_relposeB_path[i]){ _new_data = false; }
  }
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Reference)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Reference)
