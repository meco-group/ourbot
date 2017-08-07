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

  //For motion time calculation
  addProperty("eps_position", _eps_position).doc("Epsilon in position for motion time calculation");
  addProperty("eps_velocity", _eps_velocity).doc("Epsilon in velocity for motion time calculation");
  addProperty("horizon_time", _horizon_time).doc("Trajectory horizon time");
  addProperty("vmax", _vmax).doc("Maximum velocity");

  //Methods
  addOperation("initHostObstTraj", &FlexonomyMotionPlanning::initHostObstTraj, this).doc("Initiaize host obstacle trajectory port");
  addOperation("writeHostObstTraj", &FlexonomyMotionPlanning::writeHostObstTraj, this).doc("Write to host obstacle trajectory port");
}

void FlexonomyMotionPlanning::initHostObstTraj()
{
  std::vector<double> example(26, 0.0);
  _host_obstacle_trajectory_port.setDataSample(example);
}

void FlexonomyMotionPlanning::writeHostObstTraj(int option)
{
  //Case 1 - Host robot moving and avoiding - Send motion planning spline coefficients
  //Case 2 - Host robot moving and not avoiding - Send robothand position coefficients
  //Case 3 - Host robot idle - Send est_pose coefficients
  std::vector<double> coeff_vector(26, 0.0);

  for(int i=0;i<13;i++)
  {
    if(option==1)
    {
    coeff_vector[i] = _ref_pose_trajectory[0][i]; //x-coeffs
    coeff_vector[i+13] = _ref_pose_trajectory[1][i]; //y-coeffs
    }
    if(option==2)
    {
    coeff_vector[i] = _robotarm_pos[0]; //x-coeffs
    coeff_vector[i+13] = _robotarm_pos[1]; //y-coeffs
    }
    if(option==3)
    {
    coeff_vector[i] = _est_pose[0]; //x-coeffs
    coeff_vector[i+13] = _est_pose[1]; //y-coeffs
    }
  }
  _host_obstacle_trajectory_port.write(coeff_vector);
}

void FlexonomyMotionPlanning::getObstacles(std::vector<omg::obstacle_t>& obstacles){

    //Obstacle 1 : Classical -  Robot arm
    std::cout << "obstacle 1 : Robot arm at [" <<  _robotarm_pos[0] << "," << _robotarm_pos[1] << "]"  << std::endl;
    obstacles[0].position = _robotarm_pos;
    obstacles[0].checkpoints = _robotarm_checkpts;
    obstacles[0].radii = _robotarm_radii;
    obstacles[0].avoid = _robotarm_avoid;

    //Obstacle 2 : Spline trajectory -  other vehicle
    std::vector<double> _obstacle_trajectory(26,0);
    if (_obstacle_trajectory_port.connected()){
      if(_obstacle_trajectory_port.read(_obstacle_trajectory) == RTT::NewData){
        std::cout << "obstacle 2 : Spline Trajectory" << std::endl;
        std::cout << "x coefficients : [";
        for(int i=0; i<13; i++)
        {
          std::cout << _obstacle_trajectory[i];
          if(i<12){ std::cout << ","; }
        }
        std::cout << "]" << std::endl;

        std::cout << "y coefficients : [";
        for(int i=0; i<13; i++)
        {
          std::cout << _obstacle_trajectory[i+13] << ",";
          if(i<12){ std::cout << ","; }
        }
        std::cout << "]" << std::endl;

        obstacles[1].traj_coeffs = _obstacle_trajectory;
      }
    }
    else
    {
        std::cout << "obstacle 2 Not connected, using all zero spline coefficients" << std::endl;
    }

  }

double FlexonomyMotionPlanning::getMotionTime(){
  // If in the current horizon, target location is not present -> motion_time from the same formula used while bidding
  // If the target location is present, then check along velocity trajectory where speed is zero
  double target_dist = 0.;
  double v_mean;
  double dist;
  int num_parts = 100;
  std::vector<double> time_vector(num_parts,0);
  std::vector< std::vector<double> > velocity_samples (2, std::vector<double> (num_parts, 0 ) );

  for (int i=0; i<2; i++){
    target_dist += pow(_target_pose[i] - _ref_pose_trajectory[i][12], 2);
  }
  target_dist = sqrt(target_dist);
  if(target_dist>_eps_position)
  {
    v_mean = 0.87*_vmax;
    dist = sqrt(pow(_target_pose[0]-_ref_pose_trajectory[0][0], 2) + pow(_target_pose[1]-_ref_pose_trajectory[1][0], 2));
    return dist/v_mean;
  }
  else
  {
    //Create a vector of time instances with horizon time, sample velocity splines there, see where velocity is zero
    for(int i=1;i<100;i++)
    {
      time_vector[i] = time_vector[i-1]+_horizon_time/num_parts;
    }
    omg::Vehicle* vehicle = new omg::Holonomic();
    vehicle->splines2State(_ref_velocity_trajectory, time_vector, velocity_samples);
    int i=0;
    while(i<100)
    {
      if(sqrt(pow(velocity_samples[0][i], 2) + pow(velocity_samples[1][i], 2))<=_eps_velocity)
      {
        return time_vector[i];
      }
      i+=1;
    }
  }
}

ORO_LIST_COMPONENT_TYPE(FlexonomyMotionPlanning);
