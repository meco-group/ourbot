#include "DistributedReference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DistributedReference::DistributedReference(std::string const& name) : Reference(name),
    _ref_relposeL_sample(3), _ref_relposeR_sample(3){
  ports()->addPort("ref_relposeL_trajectory_x_port", _ref_relposeL_trajectory_port[0]).doc("x relpose reference trajectory wrt neighbour via left going signal");
  ports()->addPort("ref_relposeL_trajectory_y_port", _ref_relposeL_trajectory_port[1]).doc("y relpose reference trajectory wrt neighbour via left going signal");
  ports()->addPort("ref_relposeL_trajectory_t_port", _ref_relposeL_trajectory_port[2]).doc("theta relpose reference trajectory wrt neighbour via left going signal");

  ports()->addPort("ref_relposeR_trajectory_x_port", _ref_relposeR_trajectory_port[0]).doc("x relpose reference trajectory wrt neighbour via right going signal");
  ports()->addPort("ref_relposeR_trajectory_y_port", _ref_relposeR_trajectory_port[1]).doc("y relpose reference trajectory wrt neighbour via right going signal");
  ports()->addPort("ref_relposeR_trajectory_t_port", _ref_relposeR_trajectory_port[2]).doc("theta relpose reference trajectory wrt neighbour via right going signal");

  ports()->addPort("ref_relposeL_port", _ref_relposeL_port).doc("Relative pose reference sample wrt neighbour via left going signal");
  ports()->addPort("ref_relposeR_port", _ref_relposeR_port).doc("Relative pose reference sample wrt neighbour via right going signal");

  addOperation("writeSample",&DistributedReference::writeSample, this).doc("Set data sample on output ports");

  _cur_ref_relposeL_trajectory  = _ref_relposeL_trajectory_1;
  _cur_ref_relposeR_trajectory  = _ref_relposeR_trajectory_1;

  _nxt_ref_relposeL_trajectory  = _ref_relposeL_trajectory_2;
  _nxt_ref_relposeR_trajectory  = _ref_relposeR_trajectory_2;
}

void DistributedReference::writeSample(){
  std::vector<double> example(3, 0.0);
  _ref_relposeL_port.write(example);
  _ref_relposeR_port.write(example);
}

bool DistributedReference::configureHook(){
  bool check = Reference::configureHook();

  // Reserve required memory and initialize with zeros
  for(int i=0;i<3;i++){
    _cur_ref_relposeL_trajectory[i].resize(_trajectory_length);
    _cur_ref_relposeR_trajectory[i].resize(_trajectory_length);

    _nxt_ref_relposeR_trajectory[i].resize(_trajectory_length);
    _nxt_ref_relposeL_trajectory[i].resize(_trajectory_length);
  }

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _ref_relposeL_port.setDataSample(example);
  _ref_relposeR_port.setDataSample(example);

  // Reset index & checks
  for (int i=0; i<3; i++){
    _got_ref_relposeL_trajectory[i]  = false;
    _got_ref_relposeR_trajectory[i]  = false;
  }
  return check;
}

bool DistributedReference::startHook(){
  // Check connections
  for(int i=0; i<3; i++){
    _con_ref_relposeL_trajectory[i]  = _ref_relposeL_trajectory_port[i].connected();
    _con_ref_relposeR_trajectory[i]  = _ref_relposeR_trajectory_port[i].connected();
  }

  return Reference::startHook();
}

void DistributedReference::updateHook(){
  // Check for new data and read ports
  readPorts();

  // Get next sample
  for (int i=0; i<3; i++){
    _ref_relposeL_sample.at(i)  = _cur_ref_relposeL_trajectory[i].at(_index);
    _ref_relposeR_sample.at(i)  = _cur_ref_relposeR_trajectory[i].at(_index);
  }
  _ref_relposeL_port.write(_ref_relposeL_sample);
  _ref_relposeR_port.write(_ref_relposeR_sample);

  // Update index/trajectory vector
  if( (_index+1) == _trajectory_length){
    if( _new_data){
      // Swap current and next pointers
      std::vector<double>* swap_relposeL  = _cur_ref_relposeL_trajectory;
      std::vector<double>* swap_relposeR  = _cur_ref_relposeR_trajectory;
      _cur_ref_relposeL_trajectory = _nxt_ref_relposeL_trajectory;
      _cur_ref_relposeR_trajectory = _nxt_ref_relposeR_trajectory;
      _nxt_ref_relposeL_trajectory = swap_relposeL;
      _nxt_ref_relposeR_trajectory = swap_relposeR;

      // Reset index & checks -> is partly done by parent
      for (int i=0; i<3; i++){
        _got_ref_relposeL_trajectory[i]  = false;
        _got_ref_relposeR_trajectory[i]  = false;
      }
    }
  }

  Reference::updateHook();
}

void DistributedReference::readPorts(){
  Reference::readPorts();
  for(int i=0; i<3; i++){
    std::vector<double> trajectory(_trajectory_length);
    if ( _con_ref_relposeL_trajectory[i] ){
      if (_ref_relposeL_trajectory_port[i].read(trajectory) == RTT::NewData){
        _nxt_ref_relposeL_trajectory[i] = trajectory;
        _got_ref_relposeL_trajectory[i] = true;
      }
    }
    if ( _con_ref_relposeR_trajectory[i] ){
      if (_ref_relposeR_trajectory_port[i].read(trajectory) == RTT::NewData){
        _nxt_ref_relposeR_trajectory[i] = trajectory;
        _got_ref_relposeR_trajectory[i] = true;
      }
    }
  }

  for(int i=0; i<3; i++){
    if ( _con_ref_relposeL_trajectory[i] != _got_ref_relposeL_trajectory[i]){ _new_data = false; }
    if ( _con_ref_relposeR_trajectory[i] != _got_ref_relposeR_trajectory[i]){ _new_data = false; }
  }
}


ORO_LIST_COMPONENT_TYPE(DistributedReference);
