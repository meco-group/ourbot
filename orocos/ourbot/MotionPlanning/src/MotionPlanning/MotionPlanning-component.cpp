#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name){
}

bool MotionPlanning::config(){
  omg::Vehicle* vehicle = new omg::Holonomic();
  if (_ideal_prediction){
    vehicle->setIdealPrediction(true);
  }
  _n_st = vehicle->getNState();
  _n_in = vehicle->getNInput();
  _p2p = new omg::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length_full);
  _ref_pose.resize(_trajectory_length_full);
  _ref_velocity.resize(_trajectory_length_full);
  for(int k=0; k<_trajectory_length_full; k++){
    _ref_pose[k].resize(_n_st);
    _ref_velocity[k].resize(_n_in);
  }
  _obstacles.resize(_p2p->n_obs);
  _n_obs = _p2p->n_obs;
  std::cout << "Loaded motion planning problem with " << _n_obs << " obstacles." << std::endl;
  return true;
}

bool MotionPlanning::initialize(){
  _p2p->reset();
  _p2p->resetTime();
  return true;
}

bool MotionPlanning::trajectoryUpdate(){
  // get obstacles
  std::vector<omg::obstacle_t> obstacles(_n_obs);
  for (int k=0; k<_n_obs; k++){
    obstacles[k].position.resize(2, 0.0);
    obstacles[k].velocity.resize(2, 0.0);
    obstacles[k].acceleration.resize(2, 0.0);
    obstacles[k].checkpoints.resize(2*4, 0.0);
    obstacles[k].traj_coeffs.resize(2*13, 0.0);
    obstacles[k].radii.resize(4, 0.0);
    obstacles[k].avoid = true;
  }
  getObstacles(obstacles);

  // update motion planning algorithm
  bool check = _p2p->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, obstacles, _predict_shift);
  for (int k=0; k<_trajectory_length; k++){
    for (int j=0; j<_n_st; j++){
      _ref_pose_trajectory[j][k] = _ref_pose[k][j];
    }
    for (int j=0; j<_n_in; j++){
      _ref_velocity_trajectory[j][k] = _ref_velocity[k][j];
    }
  }
  // subsample for transmission
  for (int k=0; k<_trajectory_length_tx; k++){
    for (int j=0; j<2; j++){
      _ref_pose_trajectory_ss[j][k] = _ref_pose[k*_tx_subsample][j];
    }
  }
  return check;
}

void MotionPlanning::getObstacles(std::vector<omg::obstacle_t>& obstacles){
  for (int k=0; k<_n_obs; k++){
    std::cout << "obstacle " << k << ": [" << _obstacles[k].position[0] <<  ", " << _obstacles[k].position[1] << "]" << std::endl;
    obstacles[k].position = _obstacles[k].position;
    obstacles[k].velocity = _obstacles[k].velocity;
    obstacles[k].acceleration = _obstacles[k].acceleration;
    obstacles[k].checkpoints = _obstacles[k].checkpoints;
    obstacles[k].radii = _obstacles[k].radii;
    obstacles[k].avoid = _obstacles[k].avoid;
  }
}

void MotionPlanning::sample_spline_trajs()
{
  if(1==0)
  {
    omg::Vehicle* vehicle = new omg::Holonomic();
    int num_parts = 100;
    std::vector<double> time_vector(num_parts, 0.);
    std::vector< std::vector<double> > velocity_samples(num_parts, std::vector<double> (2, 0. ) );
    std::vector< std::vector<double> > coeffs_vector_2D(13, std::vector<double> (2, 0. ) );
    for(int i=0;i<2;i++)
    {
      std::cout << "i -> " << i << std::endl;
      for(int j=0;j<13;j++)
      {
        coeffs_vector_2D[j][i]=i+j;
        std::cout << "coeff " << j << "-> " << coeffs_vector_2D[i][j] << std::endl;
      }
    }
    for(int i=1; i<100; i++)
    {
      time_vector[i] = time_vector[i-1]+_horizon_time/num_parts;
    }
    vehicle->splines2Input(coeffs_vector_2D, time_vector, velocity_samples);
    for(int i=0;i<num_parts;i++)
    {
      std::cout << "i : " << time_vector[i] << "->" << velocity_samples[0][i] << "," << velocity_samples[1][i] << std::endl;
    }
  }
  std::vector<omg::obstacle_t> obstacles(_n_obs);
  getObstacles(obstacles);
}

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
