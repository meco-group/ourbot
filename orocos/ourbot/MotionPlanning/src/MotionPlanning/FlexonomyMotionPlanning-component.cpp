#include "FlexonomyMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


using namespace std;

FlexonomyMotionPlanning::FlexonomyMotionPlanning(std::string const& name) : MotionPlanning(name){

  //Other vehicle trajectory input port
  ports()->addPort("obstacle_trajectory_port", _obstacle_trajectory_port).doc("Other vehicle Obstacle trajectory port");
  ports()->addPort("host_obstacle_trajectory_port", _host_obstacle_trajectory_port).doc("Host vehicle Obstacle trajectory port");

  //Robot arm properties
  addProperty("robotarm_pos", _robotarm_pos).doc("Robot arm position");
  addProperty("robotarm_checkpts", _robotarm_checkpts).doc("Robot arm checkpoints");
  addProperty("robotarm_radii", _robotarm_radii).doc("Robot arm radius");
  addProperty("robotarm_avoid", _robotarm_avoid).doc("Robot arm avoid?");
  addProperty("obstacle_radius", _obstacle_radius).doc("Obstacle radius");
  addProperty("obstacle_checkpts", _obstacle_checkpts).doc("Obstacle checkpoints");

  //For motion time calculation
  addProperty("eps_position", _eps_position).doc("Epsilon in position for motion time calculation");
  addProperty("eps_velocity", _eps_velocity).doc("Epsilon in velocity for motion time calculation");
  addProperty("horizon_time", _horizon_time).doc("Trajectory horizon time");
  addProperty("vmax", _vmax).doc("Maximum velocity");

  //For rotation angle
  addProperty("num_secs_rot", _num_secs_rot).doc("Number of seconds of rotation");

  //Operations
  addOperation("writeHostObstTraj", &FlexonomyMotionPlanning::writeHostObstTraj, this).doc("Write to host obstacle trajectory port");
  addOperation("sample_spline_trajs", &FlexonomyMotionPlanning::sample_spline_trajs, this).doc("Write to host obstacle trajectory port");
}

bool FlexonomyMotionPlanning::config() {
  std::vector<double> example(26, 0.0);
  _host_obstacle_trajectory_port.setDataSample(example);
  _old_estimated_pose.resize(3,0.0);
  return MotionPlanning::config();
}

void FlexonomyMotionPlanning::writeHostObstTraj(int option) {
  //Case 1 - Host robot moving and avoiding - Send motion planning spline coefficients
  //Case 2 - Host robot moving and not avoiding - Send robothand position coefficients
  //Case 3 - Host robot idle - Send est_pose coefficients

  std::vector<double> traj_vector(26, 0.0);
  std::vector<double> coeff_vector(26, 0.0);
  std::vector<double> est_pose;
  if (_est_pose_port.read(est_pose) == RTT::NewData) {
    _old_estimated_pose = est_pose;
  }
  if (!_first_iteration) {
    coeff_vector = _p2p->spline_coeffs_vec;
  } else {
    for (int i=0;i<13;i++) {
      coeff_vector[i] = _old_estimated_pose[0] + ((_target_pose[0]-_old_estimated_pose[0])/12)*i;
      coeff_vector[i+13] = _old_estimated_pose[1] + ((_target_pose[1]-_old_estimated_pose[1])/12)*i;
    }
  }
  for(int i=0; i<13; i++) {
    if(option==1) {
      traj_vector[i] = coeff_vector[i]; //x-coeffs
      traj_vector[i+13] = coeff_vector[i+13]; //y-coeffs
    }
    if(option==2) {
      traj_vector[i] = _robotarm_pos[0]; //x-coeffs
      traj_vector[i+13] = _robotarm_pos[1]; //y-coeffs
    }
  }
  if(option==3) {
    for(int i=0; i<13; i++) {
      traj_vector[i] = _old_estimated_pose[0]; //x-coeffs
      traj_vector[i+13] = _old_estimated_pose[1]; //y-coeffs
    }
  }
  if(1==0) {
    std::cout << "host obstacle : Spline Trajectory" << std::endl;
    std::cout << "x coefficients : [";
    for(int i=0; i<13; i++)
    {
      std::cout << traj_vector[i];
      if(i<12) {
        std::cout << ",";
      }
    }
    std::cout << "]" << std::endl;
    std::cout << "y coefficients : [";
    for(int i=0; i<13; i++)
    {
      std::cout << traj_vector[i+13] << ",";
      if(i<12){
        std::cout << ",";
      }
    }
    std::cout << "]" << std::endl;

    std::vector<double> _obstacle_trajectory(26, 0.);

    if(_obstacle_trajectory_port.connected())
    {
      _obstacle_trajectory_port.read(_obstacle_trajectory);
      std::cout << "guest obstacle : Spline Trajectory" << std::endl;
      std::cout << "x coefficients : [";
      for(int i=0; i<13; i++)
      {
        std::cout << _obstacle_trajectory[i];
        if(i<12) {
          std::cout << ",";
        }
      }
      std::cout << "]" << std::endl;
      std::cout << "y coefficients : [";
      for(int i=0; i<13; i++)
      {
        std::cout << _obstacle_trajectory[i+13] << ",";
        if(i<12){
          std::cout << ",";
        }
      }
      std::cout << "]" << std::endl;
    }
  }

  _host_obstacle_trajectory_port.write(traj_vector);
}

void FlexonomyMotionPlanning::getObstacles(std::vector<omg::obstacle_t>& obstacles) {
  // obstacle 1 : Classical -  Robot arm

  obstacles[0].position = _robotarm_pos;
  obstacles[0].checkpoints = _robotarm_checkpts;
  obstacles[0].radii = _robotarm_radii;
  obstacles[0].avoid = _robotarm_avoid;

  if(_p2p->n_obs==2)
  {
    // obstacle 2 : Spline trajectory -  other vehicle
    std::vector<double> _obstacle_trajectory(26, 0.);
    if (_obstacle_trajectory_port.connected()){
      if(_obstacle_trajectory_port.read(_obstacle_trajectory) == RTT::NewData)
      {
        if(1==0)
        {
          std::cout << "obstacle 2 : Spline Trajectory" << std::endl;
          std::cout << "x coefficients : [";
          for(int i=0; i<13; i++)
          {
            std::cout << _obstacle_trajectory[i];
            if(i<12) {
              std::cout << ",";
            }
          }
          std::cout << "]" << std::endl;
          std::cout << "y coefficients : [";
          for(int i=0; i<13; i++)
          {
            std::cout << _obstacle_trajectory[i+13] << ",";
            if(i<12){
              std::cout << ",";
            }
          }
        std::cout << "]" << std::endl;
        }
        obstacles[1].traj_coeffs = _obstacle_trajectory;
        obstacles[1].avoid = true;
        obstacles[1].radii = _obstacle_radius;
        obstacles[1].checkpoints = _obstacle_checkpts;
      }
    }
    else
    {
      std::cout << "obstacle 2 Not connected, using all zero spline coefficients" << std::endl;
    }
  }
}

double FlexonomyMotionPlanning::getMotionTime(){
  // If in the current horizon, target location is not present -> motion_time from the same formula used while bidding
  // If the target location is present, then check along velocity trajectory where speed is zero
  double target_dist = 0.;
  double v_mean;
  double dist;
  int num_parts = 2000;

  std::vector<double> time_vector(num_parts, 0.);
  std::vector< std::vector<double> > position_samples(num_parts, std::vector<double> (2, 0. ) );
  std::vector< std::vector<double> > coeffs_vector_2D(2, std::vector<double> (13, 0. ) );
  std::vector<double> coeff_vector(26,0.0);
  coeff_vector = _p2p->spline_coeffs_vec;

  target_dist += pow(_target_pose[0] - coeff_vector[12], 2); // --> not coeffs !!
  target_dist += pow(_target_pose[1] - coeff_vector[25], 2); // --> not coeffs !!

  target_dist = sqrt(target_dist);

  if(target_dist > _eps_position)
  {
    v_mean = 0.87*_vmax;
    dist = sqrt(pow(_target_pose[0]-coeff_vector[0], 2) + pow(_target_pose[1]-coeff_vector[13], 2));
    return dist/v_mean;
  }
  else {
    double time_inst = 0;
    int k=0;
    while(k<_trajectory_length_full){
      //std::cout << "Time - " << time_inst << " --- [" << _ref_pose[k][0] << "," << _ref_pose[k][1] << "]" << std::endl;
      if(sqrt(pow(_ref_pose[k][0]-_target_pose[0], 2) + pow(_ref_pose[k][1]-_target_pose[1], 2))<=_eps_position) {
        k = _trajectory_length_full+1;
      }
      time_inst = time_inst + _sample_time;
      k+=1;
    }
    std::cout << "Reach in " << time_inst << "s" << std::endl;
    return time_inst;
  }
}

ORO_LIST_COMPONENT_TYPE(FlexonomyMotionPlanning);
