// #define DEBUG

#include "DistributedMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <algorithm>
#include <stdlib.h>

using namespace std;

DistributedMotionPlanning::DistributedMotionPlanning(std::string const& name) : MotionPlanningInterface(name),
_target_reached(false), _n_nghb(2), _neighbors(2, ""), _residuals(3), _rel_pos_c(2){
  for (int k=0; k<_n_nghb; k++){
    ports()->addPort("x_var_port_" + to_string(k), _x_var_port[k]);
    ports()->addPort("zl_ij_var_port_" + to_string(k), _zl_ij_var_port[k]);
    ports()->addPort("x_j_var_port_" + to_string(k), _x_j_var_port[k]);
    ports()->addPort("zl_ji_var_port_" + to_string(k), _zl_ji_var_port[k]);
  }

  addProperty("configuration_x", _configuration_x).doc("x coordinates relative to fleet center");
  addProperty("configuration_y", _configuration_y).doc("y coordinates relative to fleet center");
  addProperty("init_iter", _init_iter).doc("Number of initial ADMM iterations");
  addProperty("rho", _rho).doc("Penalty factor for ADMM");

  addOperation("setRelPoseC", &DistributedMotionPlanning::setRelPoseC, this).doc("Set rel_pos_c");
  addOperation("writeSample", &DistributedMotionPlanning::writeSample, this).doc("Write sample");
  addOperation("resetNeighbors", &DistributedMotionPlanning::resetNeighbors, this);
  addOperation("addNeighbor", &DistributedMotionPlanning::addNeighbor, this);
  addOperation("connectWithNeighbors", &DistributedMotionPlanning::connectWithNeighbors, this);

}

void DistributedMotionPlanning::resetNeighbors(){
  std::vector<double> est_pose(3);
  _est_pose_port.read(est_pose);
  _robot_poses.resize(0);
  _robot_names.resize(0);
  _robot_poses.push_back(est_pose);
  _robot_names.push_back(_host);
}

void DistributedMotionPlanning::addNeighbor(const std::vector<double>& neighbor_pose, const std::string& neighbor_name){
  _robot_poses.push_back(neighbor_pose);
  _robot_names.push_back(neighbor_name);
}


std::vector<double> DistributedMotionPlanning::setConfiguration(){
  int number_of_robots = _robot_poses.size();
  // build configuration vector
  std::vector<std::vector<double>> configuration(number_of_robots, std::vector<double>(2));
  for (int k=0; k<number_of_robots; k++){
    configuration[k][0] = _configuration_x[k];
    configuration[k][1] = _configuration_y[k];
  }
  // determine optimal midpoint and robot permutation
  std::vector<double> pose0(3, 0);
  std::vector<int> indices(number_of_robots);
  for (int k=0; k<number_of_robots; k++){
    indices[k] = k;
  }
  _robot_indices.resize(number_of_robots);
  double residual = 10000000000;
  do {
    std::vector<double> pose0_tmp(2);
    for (int k=0; k<number_of_robots; k++){
      pose0_tmp[0] += (1./number_of_robots)*(_robot_poses[indices[k]][0] - configuration[k][0]);
      pose0_tmp[1] += (1./number_of_robots)*(_robot_poses[indices[k]][1] - configuration[k][1]);
    }
    double res = 0;
    for (int k=0; k<number_of_robots; k++){
      res += pow(_robot_poses[indices[k]][0] - pose0_tmp[0] - configuration[k][0], 2);
      res += pow(_robot_poses[indices[k]][1] - pose0_tmp[1] - configuration[k][1], 2);
    }
    if (res < residual){
      residual = res;
      pose0[0] = pose0_tmp[0];
      pose0[1] = pose0_tmp[1];
      for (int k=0; k<number_of_robots; k++){
        _robot_indices[k] = indices[k];
      }
    }
  } while(std::next_permutation(indices.begin(),indices.end()));
  _host_index = 0;
  for (int k=0; k<number_of_robots; k++){
    if (_robot_indices[k] == 0){
      _host_index = k;
      break;
    }
  }
  _rel_pos_c = configuration[_host_index];
  return pose0;
}

bool DistributedMotionPlanning::connectWithNeighbors(){
  OperationCaller<std::string(const std::string&, const std::string&, const std::string&)> getSender = _communicator->getOperation("getSender");
  OperationCaller<bool(const std::string&, const std::string&, const std::string&, const std::string&)> addOutgoingConnection = _communicator->getOperation("addOutgoingConnection");
  OperationCaller<bool(const std::string&, const std::string&, const std::string&)> addIncomingConnection = _communicator->getOperation("addIncomingConnection");
  OperationCaller<void(const std::string&, const std::string&, const std::string&)> removeConnection = _communicator->getOperation("removeConnection");
  if (!getSender.ready()){
    log(Error) << "Could not find communicator.getSender operation!" << endlog();
    error();
    return false;
  }
  if (!addOutgoingConnection.ready()){
    log(Error) << "Could not find communicator.addOutgoingConnection operation!" << endlog();
    error();
    return false;
  }
  if (!addIncomingConnection.ready()){
    log(Error) << "Could not find communicator.addIncomingConnection operation!" << endlog();
    error();
    return false;
  }
  if (!removeConnection.ready()){
    log(Error) << "Could not find communicator.removeConnection operation!" << endlog();
    error();
    return false;
  }
  // remove possible previous connections
  for (int k=0; k<_n_nghb; k++){
    removeConnection("distrmotionplanning", "x_var_port_" + std::to_string(k), "x_var_" + _host);
    removeConnection("distrmotionplanning", "zl_ij_var_port_" + std::to_string(k), "zl_var_" + _host);
    removeConnection("distrmotionplanning", "x_j_var_port_" + std::to_string(k), "x_var_" + _neighbors[k]);
    removeConnection("distrmotionplanning", "zl_ji_var_port_" + std::to_string(k), "zl_var_" + _neighbors[k]);
    removeConnection("distrmotionplanning", "x_var_port_" + std::to_string(k), "x_var_" + _host + std::to_string(k));
    removeConnection("distrmotionplanning", "zl_ij_var_port_" + std::to_string(k), "zl_var_" + _host + std::to_string(k));
    removeConnection("distrmotionplanning", "x_j_var_port_" + std::to_string(k), "x_var_" + _neighbors[k] + std::to_string(k));
    removeConnection("distrmotionplanning", "zl_ji_var_port_" + std::to_string(k), "zl_var_" + _neighbors[k] + std::to_string(k));

  }
  // add new connections
  int number_of_robots = _robot_poses.size();
  _neighbors[0] = _robot_names[_robot_indices[(number_of_robots+_host_index+1)%number_of_robots]];
  _neighbors[1] = _robot_names[_robot_indices[(number_of_robots+_host_index-1)%number_of_robots]];
  for (int k=0; k<_n_nghb; k++){
    if (number_of_robots == 2){
      // If both neighbors are the same: make sure the connections' id are different. Otherwise both input ports receive all info. This leads to index mismatches!
      addOutgoingConnection("distrmotionplanning", "x_var_port_" + std::to_string(k), "x_var_" + _host + std::to_string(k), _neighbors[k]);
      addOutgoingConnection("distrmotionplanning", "zl_ij_var_port_" + std::to_string(k), "zl_var_" + _host + std::to_string(k), _neighbors[k]);
      addIncomingConnection("distrmotionplanning", "x_j_var_port_" + std::to_string(k), "x_var_" + _neighbors[k] + std::to_string(k));
      addIncomingConnection("distrmotionplanning", "zl_ji_var_port_" + std::to_string(k), "zl_var_" + _neighbors[k] + std::to_string(k));
    } else {
      addOutgoingConnection("distrmotionplanning", "x_var_port_" + std::to_string(k), "x_var_" + _host, _neighbors[k]);
      addOutgoingConnection("distrmotionplanning", "zl_ij_var_port_" + std::to_string(k), "zl_var_" + _host, _neighbors[k]);
      addIncomingConnection("distrmotionplanning", "x_j_var_port_" + std::to_string(k), "x_var_" + _neighbors[k]);
      addIncomingConnection("distrmotionplanning", "zl_ji_var_port_" + std::to_string(k), "zl_var_" + _neighbors[k]);
    }
  }
  return true;
}

void DistributedMotionPlanning::writeSample(){
  std::vector<double> example1(_n_shared + 1, 0.0);
  std::vector<double> example2(2*_n_shared + 1, 0.0);
  for (int k=0; k<_n_nghb; k++){
    _x_var_port[k].write(example1);
    _zl_ij_var_port[k].write(example2);
  }
}

bool DistributedMotionPlanning::config(){
  _communicator = getPeer("communicator");
  if (_communicator == NULL){
    log(Error) << "Could not find component communicator." << endlog();
    error();
    return false;
  }
  OperationCaller<std::string(void)> getHost = _communicator->getOperation("getHost");
  _host = getHost();

  omgf::Vehicle* vehicle = new omgf::Holonomic();
  if (_ideal_prediction){
    vehicle->setIdealPrediction(true);
  }
  _n_st = vehicle->getNState();
  _n_in = vehicle->getNInput();
  _problem = new omgf::FormationPoint2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length_full, _init_iter, _rho);
  _obstacles.resize(_problem->n_obs);
  _ref_pose.resize(_trajectory_length_full);
  _ref_velocity.resize(_trajectory_length_full);
  for(int k=0; k<_trajectory_length_full; k++){
    _ref_pose[k].resize(_n_st);
    _ref_velocity[k].resize(_n_in);
  }
  _n_obs = _problem->n_obs;
  _n_shared = _problem->n_shared;
  _x_var.resize(_n_shared);
  _x_j_var.resize(_n_nghb);
  _z_ji_var.resize(_n_nghb);
  _l_ji_var.resize(_n_nghb);
  _z_ij_var.resize(_n_nghb);
  _l_ij_var.resize(_n_nghb);
  _zl_ij_p_var.resize(_n_nghb);
  _zl_ji_p_var.resize(_n_nghb);
  _x_p_var.resize(_n_shared+1);
  _x_j_p_var.resize(_n_nghb);
  for(int k=0; k<_n_nghb; k++){
    _x_j_var[k].resize(_n_shared);
    _z_ji_var[k].resize(_n_shared);
    _l_ji_var[k].resize(_n_shared);
    _z_ij_var[k].resize(_n_shared);
    _l_ij_var[k].resize(_n_shared);
    _zl_ij_p_var[k].resize(2*_n_shared + 1);
    _zl_ji_p_var[k].resize(2*_n_shared + 1);
    _x_j_p_var[k].resize(_n_shared + 1);
  }
  _target_reached_nghb.resize(_n_nghb);
  std::vector<double> example1(_n_shared + 1, 0.0);
  std::vector<double> example2(2*_n_shared + 1, 0.0);
  for (int k=0; k<_n_nghb; k++){
    _x_var_port[k].setDataSample(example1);
    _zl_ij_var_port[k].setDataSample(example2);
  }
  std::cout << "Loaded distributed motion planning problem with " << _n_obs << " obstacles." << std::endl;
  return true;
}

bool DistributedMotionPlanning::initialize(){
  emptyPorts();
  _problem->reset();
  _problem->resetTime();
  return true;
}

void DistributedMotionPlanning::emptyPorts(){
  std::vector<double> dump(2*_n_shared + 1, 0.0);
  for (int k=0; k<_n_nghb; k++){
    _x_j_var_port[k].read(dump);
    _zl_ji_var_port[k].read(dump);
  }
}

bool DistributedMotionPlanning::trajectoryUpdate(){
  bool check;
  while(_problem->getIteration() < _init_iter){
    check = admmIteration(true);
    if (!check){ // initial crash -> let reference give up
      return false;
    }
  }
  return admmIteration(false);
}

double DistributedMotionPlanning::encode(int index, bool valid){
  code_t code;
  code.index = index;
  code.target_reached = _target_reached;
  code.valid = valid;
  double ret;
  memcpy((char*)(&ret), (char*)(&code), sizeof(ret));
  return ret;
}

code_t DistributedMotionPlanning::decode(double number){
  code_t code;
  memcpy((char*)(&code), (char*)(&number), sizeof(code));
  return code;
}

bool DistributedMotionPlanning::watchDog(bool initial, TimeService::ticks t0){
  if (!initial && TimeService::Instance()->secondsSince(t0) > 2){
    log(Error) << "Waiting on neighbor took more than 2s" << endlog();
    return false;
  }
  if (TimeService::Instance()->secondsSince(t0) > 10){
    log(Error) << "Waiting on neighbor took more than 10s" << endlog();
    return false;
  }
  return true;
}

bool DistributedMotionPlanning::admmIteration(bool initial){
  std::cout << "admm iteration " << _problem->getIteration() << std::endl;
  #ifdef DEBUG
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // compute agent's target pose
  std::vector<double> target_pose(3);
  for (int k=0; k<3; k++){
    target_pose[k] = _target_pose[k] + _rel_pos_c[k];
  }
  // get obstacles
  std::vector<omgf::obstacle_t> obstacles(_n_obs);
  getObstacles(obstacles);
  // update1: determine x_var
  std::vector<double> rpc(2);
  rpc[0] = -_rel_pos_c[0];
  rpc[1] = -_rel_pos_c[1];
  bool check = _problem->update1(_est_pose, target_pose, _ref_pose, _ref_velocity, _x_var, _z_ji_var, _l_ji_var, obstacles, rpc, _predict_shift);
  #ifdef DEBUG
  Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "update1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // add code and send this result to neighbors
  for (int i=0; i<_n_shared; i++){
    _x_p_var[i] = _x_var[i];
  }
  _x_p_var[_n_shared] = encode(_problem->getIteration(), check);
  for (int k=0; k<_n_nghb; k++){
    _x_var_port[k].write(_x_p_var);
  }
  // in the meantime, extract trajectories
  for (int k=0; k<_trajectory_length; k++){
    for (int j=0; j<_n_st; j++){
      _ref_pose_trajectory[j][k] = _ref_pose[k][j];
    }
    for (int j=0; j<_n_in; j++){
      _ref_velocity_trajectory[j][k] = _ref_velocity[k][j];
    }
  }
  for (int k=0; k<_trajectory_length_tx; k++){
    for (int j=0; j<2; j++){
      _ref_pose_trajectory_ss[j][k] = _ref_pose[k*_tx_subsample][j];
    }
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "extracting+sending1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // receive from neighbors (wait until new data)
  TimeService::ticks t0 = TimeService::Instance()->getTicks();
  for (int k=0; k<_n_nghb; k++){
    while ( _x_j_var_port[k].read(_x_j_p_var[k]) != RTT::NewData ){
      if (!watchDog(initial, t0)){
        disable();
        return false;
      }
    }
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "waiting1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // check code and copy data
  bool check_nghb = true;
  int nghb_iteration, iteration;
  iteration = _problem->getIteration();
  if (!check){
    iteration++;
  }
  for (int k=0; k<_n_nghb; k++){
    code_t code = decode(_x_j_p_var[k][_n_shared]);
    nghb_iteration = code.index;
    if (!code.valid){
      check_nghb = false;
      nghb_iteration++;
    }
    if (nghb_iteration != iteration){
      log(Error)<<"Index mismatch!"<< endlog();
      std::cout << nghb_iteration << " vs " << iteration << std::endl;
      error();
    }
    _target_reached_nghb[k] = code.target_reached;
    for (int i=0; i<_n_shared; i++){
      _x_j_var[k][i] = _x_j_p_var[k][i];
    }
  }
  if (check && !check_nghb){
    std::cout << "stepping back..." << std::endl;
    _problem->stepBack();
    return false;
  }
  if (!check){
    disable();
    return false;
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "checking1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // update2: determine z_i_var, z_ij_var, l_i_var and l_ij_var
  _problem->update2(_x_j_var, _z_ij_var, _l_ij_var, _residuals);
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "update2: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // add code and send results to neighbors
  for (int k=0; k<_n_nghb; k++){
    for (int i=0; i<_n_shared; i++){
      _zl_ij_p_var[k][i] = _z_ij_var[k][i];
      _zl_ij_p_var[k][i+_n_shared] = _l_ij_var[k][i];
    }
    _zl_ij_p_var[k][2*_n_shared] = encode(_problem->getIteration(), check);
    _zl_ij_var_port[k].write(_zl_ij_p_var[k]);
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "extracting+sending2: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // receive from neighbors (wait until new data)
  t0 = TimeService::Instance()->getTicks();
  for (int k=0; k<_n_nghb; k++){
    while ( _zl_ji_var_port[k].read(_zl_ji_p_var[k]) != RTT::NewData ){
      if (!watchDog(initial, t0)){
        return false;
      }
    }
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "waiting2: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // check code and copy data
  for (int k=0; k<_n_nghb; k++){
    for (int i=0; i<_n_shared; i++){
      _z_ji_var[k][i] = _zl_ji_p_var[k][i];
      _l_ji_var[k][i] = _zl_ji_p_var[k][i+_n_shared];
    }
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "checking2: " << time_elapsed << " s" << endl;
  #endif
  return check;
}

void DistributedMotionPlanning::setRelPoseC(vector<double> rel_pos_c){
  if (rel_pos_c.size() == _rel_pos_c.size()){
    _rel_pos_c = rel_pos_c;
  }
  else {
    log(Error)<<"Wrong size of rel_pos_c set!"<<endlog();
  }
}

bool DistributedMotionPlanning::targetReached(){
  double target_dist = 0.;
  double input_norm = 0.;

  std::vector<double> target_pose(3);
  for (int k=0; k<3; k++){
    target_pose[k] = _target_pose[k] + _rel_pos_c[k];
  }
  for (int i=0; i<3; i++){
    target_dist += pow(_target_pose[i] + _rel_pos_c[i] - _est_pose[i], 2);
    input_norm += pow(_ref_velocity_trajectory[i][_predict_shift], 2);
  }
  target_dist = sqrt(target_dist);
  input_norm = sqrt(input_norm);
  bool target_reached_now = (target_dist < 5e-2 && input_norm < 1e-2);
  if (!_target_reached){ // you should send it to your neighbor at least once
    _target_reached = target_reached_now;
    return false;
  }
  _target_reached = target_reached_now;
  std::cout << "self target reached: " << _target_reached << std::endl;
  for (int k=0; k<_n_nghb; k++){
    if (!(_target_reached && _target_reached_nghb[k])){
      return false;
    }
  }
  return true;
}

void DistributedMotionPlanning::getObstacles(std::vector<omgf::obstacle_t>& obstacles){
  for (int k=0; k<_n_obs; k++){
    obstacles[k].position = _obstacles[k].position;
    obstacles[k].velocity = _obstacles[k].velocity;
    obstacles[k].acceleration = _obstacles[k].acceleration;
    obstacles[k].checkpoints = _obstacles[k].checkpoints;
    obstacles[k].radii = _obstacles[k].radii;
    obstacles[k].avoid = _obstacles[k].avoid;
    // obstacles[k].avoid = false;
  }
}

ORO_LIST_COMPONENT_TYPE(DistributedMotionPlanning);
