#include "DistributedReference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedReference::DistributedReference(std::string const& name) : Reference(name),
    _ref_relposeL_sample(3), _ref_relposeR_sample(3){
  ports()->addPort("ref_relposeL_path_x_port", _ref_relposeL_path_port[0]).doc("x relpose reference path wrt neighbour via left going signal");
  ports()->addPort("ref_relposeL_path_y_port", _ref_relposeL_path_port[1]).doc("y relpose reference path wrt neighbour via left going signal");
  ports()->addPort("ref_relposeL_path_t_port", _ref_relposeL_path_port[2]).doc("theta relpose reference path wrt neighbour via left going signal");

  ports()->addPort("ref_relposeR_path_x_port", _ref_relposeR_path_port[0]).doc("x relpose reference path wrt neighbour via right going signal");
  ports()->addPort("ref_relposeR_path_y_port", _ref_relposeR_path_port[1]).doc("y relpose reference path wrt neighbour via right going signal");
  ports()->addPort("ref_relposeR_path_t_port", _ref_relposeR_path_port[2]).doc("theta relpose reference path wrt neighbour via right going signal");

  ports()->addPort("ref_relposeL_port", _ref_relposeL_port).doc("Relative pose reference sample wrt neighbour via left going signal");
  ports()->addPort("ref_relposeR_port", _ref_relposeR_port).doc("Relative pose reference sample wrt neighbour via right going signal");

  _cur_ref_relposeL_path  = _ref_relposeL_path_1;
  _cur_ref_relposeR_path  = _ref_relposeR_path_1;

  _nxt_ref_relposeL_path  = _ref_relposeL_path_2;
  _nxt_ref_relposeR_path  = _ref_relposeR_path_2;
}

bool DistributedReference::configureHook(){
  bool check = Reference::configureHook();

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _cur_ref_relposeL_path[i].resize(_path_length);
    _cur_ref_relposeR_path[i].resize(_path_length);

    _nxt_ref_relposeR_path[i].resize(_path_length);
    _nxt_ref_relposeL_path[i].resize(_path_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _ref_relposeL_port.setDataSample(example);
  _ref_relposeR_port.setDataSample(example);

  // Reset index & checks
  for (int i=0; i<3; i++){
    _got_ref_relposeL_path[i]  = false;
    _got_ref_relposeR_path[i]  = false;
  }
  return check;
}

bool DistributedReference::startHook(){
  // Check connections
  for(int i=0; i<3; i++){
    _con_ref_relposeL_path[i]  = _ref_relposeL_path_port[i].connected();
    _con_ref_relposeR_path[i]  = _ref_relposeR_path_port[i].connected();
  }

  return Reference::startHook();
}

void DistributedReference::updateHook(){
  // Check for new data and read ports
  readPorts();

  // Get next sample
  for (int i=0; i<3; i++){
    _ref_relposeL_sample.at(i)  = _cur_ref_relposeL_path[i].at(_index);
    _ref_relposeR_sample.at(i)  = _cur_ref_relposeR_path[i].at(_index);
  }
  _ref_relposeL_port.write(_ref_relposeL_sample);
  _ref_relposeR_port.write(_ref_relposeR_sample);

  // Update index/path vector
  if( (_index+1) == _path_length){
    if( _new_data){
      // Swap current and next pointers
      std::vector<double>* swap_relposeL  = _cur_ref_relposeL_path;
      std::vector<double>* swap_relposeR  = _cur_ref_relposeR_path;
      _cur_ref_relposeL_path = _nxt_ref_relposeL_path;
      _cur_ref_relposeR_path = _nxt_ref_relposeR_path;
      _nxt_ref_relposeL_path = swap_relposeL;
      _nxt_ref_relposeR_path = swap_relposeR;

      // Reset index & checks -> is partly done by parent
      for (int i=0; i<3; i++){
        _got_ref_relposeL_path[i]  = false;
        _got_ref_relposeR_path[i]  = false;
      }
    }
  }

  Reference::updateHook();
}

void DistributedReference::readPorts(){
  Reference::readPorts();
  for(int i=0; i<3; i++){
    std::vector<double> path(_path_length);
    if ( _con_ref_relposeL_path[i] ){
      if (_ref_relposeL_path_port[i].read(path) == RTT::NewData){
        _nxt_ref_relposeL_path[i] = path;
        _got_ref_relposeL_path[i] = true;
      }
    }
    if ( _con_ref_relposeR_path[i] ){
      if (_ref_relposeR_path_port[i].read(path) == RTT::NewData){
        _nxt_ref_relposeR_path[i] = path;
        _got_ref_relposeR_path[i] = true;
      }
    }
  }

  for(int i=0; i<3; i++){
    if ( _con_ref_relposeL_path[i] != _got_ref_relposeL_path[i]){ _new_data = false; }
    if ( _con_ref_relposeR_path[i] != _got_ref_relposeR_path[i]){ _new_data = false; }
  }
}


ORO_LIST_COMPONENT_TYPE(DistributedReference);
