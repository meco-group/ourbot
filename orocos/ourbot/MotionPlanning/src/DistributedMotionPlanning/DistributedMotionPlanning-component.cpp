// #define DEBUG

#include "DistributedMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

DistributedMotionPlanning::DistributedMotionPlanning(std::string const& name) : MotionPlanningInterface(name),
_state0(2), _stateT(2), _residuals(3), _rel_pos_c(2){

  ports()->addPort("x_var_port", _x_var_port).doc("x var port");
  string port_name;
  for (int k=0; k<2; k++){
    ports()->addPort("zl_ij_var_port_" + to_string(k), _zl_ij_var_port[k]);
    ports()->addPort("x_j_var_port_" + to_string(k), _x_j_var_port[k]);
    ports()->addPort("zl_ji_var_port_" + to_string(k), _zl_ji_var_port[k]);
  }

  addProperty("nghb_index", _nghb_index).doc("Index numbers of neighbouring agents");
  addProperty("rel_pos_c", _rel_pos_c).doc("Relative pose wrt fleet center");
  addProperty("init_iter", _init_iter).doc("Number of initial ADMM iterations");
  addProperty("rho", _rho).doc("Penalty factor for ADMM");

  addOperation("setRelPoseC", &DistributedMotionPlanning::setRelPoseC, this).doc("Set rel_pos_c");
  addOperation("writeSample", &DistributedMotionPlanning::writeSample, this).doc("Write sample");
}

void DistributedMotionPlanning::writeSample(){
  std::vector<double> example1(_n_shared + 1, 0.0);
  std::vector<double> example2(2*_n_shared + 1, 0.0);
  _x_var_port.write(example1);
  for (int k=0; k<_n_nghb; k++){
    _zl_ij_var_port[k].write(example2);
  }
}

bool DistributedMotionPlanning::config(){
  omgf::Vehicle* vehicle = new omgf::Holonomic();
  if (_ideal_prediction){
    vehicle->setIdealPrediction(true);
  }
  _problem = new omgf::FormationPoint2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length, _init_iter, _rho);
  _obstacles.resize(_problem->n_obs);
  _ref_pose.resize(_trajectory_length);
  _ref_velocity.resize(_trajectory_length);
  for(int k=0; k<_trajectory_length; k++){
    _ref_pose[k].resize(2);
    _ref_velocity[k].resize(2);
  }
  _n_nghb = _nghb_index.size();
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
  std::vector<double> example1(_n_shared + 1, 0.0);
  std::vector<double> example2(2*_n_shared + 1, 0.0);
  _x_var_port.setDataSample(example1);
  for (int k=0; k<_n_nghb; k++){
    _zl_ij_var_port[k].setDataSample(example2);
  }
  return true;
}

bool DistributedMotionPlanning::initialize(){
  _problem->reset();
  _problem->resetTime();
  //debug data
  _target_pose[0] = 3.5 - _rel_pos_c[0];
  _target_pose[1] = 3.5 - _rel_pos_c[1];
  _target_pose[2] = 0.;
  return true;
}

bool DistributedMotionPlanning::trajectoryUpdate(){
  while(_problem->getIteration() < _init_iter){
    admmIteration();
  }
  return admmIteration();
}

bool DistributedMotionPlanning::admmIteration(){
  //debug data
  _est_pose[0] = 0. - _rel_pos_c[0];
  _est_pose[1] = 0. - _rel_pos_c[1];
  _est_pose[2] = 0.;
  #ifdef DEBUG
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // get obstacles
  std::vector<omgf::obstacle_t> obstacles(_n_obs);
  getObstacles(obstacles);
  // update1: determine x_var
  bool check = _problem->update1(_est_pose, _target_pose, _ref_pose, _ref_velocity, _x_var, _z_ji_var, _l_ji_var, obstacles, _rel_pos_c, _predict_shift);
  #ifdef DEBUG
  Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "update1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // add index number and send this result to neighbors
  for (int i=0; i<_n_shared; i++){
    _x_p_var[i] = _x_var[i];
  }
  _x_p_var[_n_shared] = _problem->getIteration();
  _x_var_port.write(_x_p_var);
  // in the meantime, extract trajectories and put them on port
  for (int k=0; k<_trajectory_length; k++){
    for (int j=0; j<2; j++){
      _ref_velocity_trajectory[j][k] = _ref_velocity[k][j];
      _ref_pose_trajectory[j][k] = _ref_pose[k][j];
    }
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "extracting+sending1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // receive from neighbors (wait until new data)
  for (int k=0; k<_n_nghb; k++){
    while ( _x_j_var_port[k].read(_x_j_p_var[k]) != RTT::NewData );
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "waiting1: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // check index
  for (int k=0; k<_n_nghb; k++){
    if (_x_j_p_var[k][_n_shared] != _problem->getIteration()){
      log(Error)<<"Index mismatch!"<< endlog();
      error();
    }
    for (int i=0; i<_n_shared; i++){
      _x_j_var[k][i] = _x_j_p_var[k][i];
    }
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
  // add index number and send results to neighbors
  for (int k=0; k<_n_nghb; k++){
    for (int i=0; i<_n_shared; i++){
      _zl_ij_p_var[k][i] = _z_ij_var[k][i];
      _zl_ij_p_var[k][i+_n_shared] = _l_ij_var[k][i];
    }
    _zl_ij_p_var[k][2*_n_shared] = _problem->getIteration();
    _zl_ij_var_port[k].write(_zl_ij_p_var[k]);
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "extracting+sending2: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // receive from neighbors (wait until new data)
  for (int k=0; k<_n_nghb; k++){
    while ( _zl_ji_var_port[k].read(_zl_ji_p_var[k]) != RTT::NewData );
  }
  #ifdef DEBUG
  time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  cout << "waiting2: " << time_elapsed << " s" << endl;
  _timestamp = TimeService::Instance()->getTicks();
  #endif
  // check index
  for (int k=0; k<_n_nghb; k++){
    if (_zl_ji_p_var[k][2*_n_shared] != _problem->getIteration()){
      log(Error)<<"Index mismatch!"<< endlog();
      error();
    }
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

void DistributedMotionPlanning::getObstacles(std::vector<omgf::obstacle_t>& obstacles){
  for (int k=0; k<_n_obs; k++){
    obstacles[k].position = _obstacles[k].position;
    obstacles[k].velocity = _obstacles[k].velocity;
    obstacles[k].acceleration = _obstacles[k].acceleration;
    obstacles[k].checkpoints = _obstacles[k].checkpoints;
    obstacles[k].radii = _obstacles[k].radii;
    obstacles[k].avoid = _obstacles[k].avoid;
  }
}

ORO_LIST_COMPONENT_TYPE(DistributedMotionPlanning);
