#include "FlexonomyMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


using namespace std;

FlexonomyMotionPlanning::FlexonomyMotionPlanning(std::string const& name) : MotionPlanning(name){

  //Other vehicle trajectory input port
  ports()->addPort("obstacle_trajectory_port", _obstacle_trajectory_port).doc("Other vehicle Obstacle trajectory port");
  ports()->addPort("host_obstacle_trajectory_port", _host_obstacle_trajectory_port).doc("Host vehicle Obstacle trajectory port");

  //Robot arm properties
  addProperty("robotarm_pose", _robotarm_pose).doc("Robot arm pose [x, y, t]");
  addProperty("robotarm_size", _robotarm_size).doc("Robot arm size [width, height]");
  addProperty("neighbor_size", _neighbor_size).doc("Neighbor ourbot size [width, height]");

  //For motion time calculation
  addProperty("vmax", _vmax).doc("Maximum velocity");

  //Operations
  addOperation("writeHostObstTraj", &FlexonomyMotionPlanning::writeHostObstTraj, this).doc("Write to host obstacle trajectory port");
}

bool FlexonomyMotionPlanning::config() {
  std::vector<double> example(26, 0.0);
  _host_obstacle_trajectory_port.setDataSample(example);
  _old_estimated_pose.resize(3,0.0);
  std::cout << "horizon time: " << _horizon_time << std::endl;
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
    _p2p->getCoefficients(coeff_vector);
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
      traj_vector[i] = _robotarm_pose[0]; //x-coeffs
      traj_vector[i+13] = _robotarm_pose[1]; //y-coeffs
    }
  }
  if(option==3) {
    for(int i=0; i<13; i++) {
      traj_vector[i] = _old_estimated_pose[0]; //x-coeffs
      traj_vector[i+13] = _old_estimated_pose[1]; //y-coeffs
    }
  }
  if (false) {
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
  obstacles[0].position = std::vector<double>({_robotarm_pose[0], _robotarm_pose[1]});
  obstacles[0].avoid = true;
  double orientation = (M_PI/180.)*_robotarm_pose[2];
  double width = _robotarm_size[0];
  double height = _robotarm_size[1];
  for (int i=0; i<4; i++){
    obstacles[0].radii[i] = 0.001;
  }
  obstacles[0].checkpoints[0] = 0.5*width*cos(orientation) - 0.5*height*sin(orientation);
  obstacles[0].checkpoints[1] = 0.5*width*sin(orientation) + 0.5*height*cos(orientation);
  obstacles[0].checkpoints[2] = 0.5*width*cos(orientation) + 0.5*height*sin(orientation);
  obstacles[0].checkpoints[3] = 0.5*width*sin(orientation) - 0.5*height*cos(orientation);
  obstacles[0].checkpoints[4] = -0.5*width*cos(orientation) + 0.5*height*sin(orientation);
  obstacles[0].checkpoints[5] = -0.5*width*sin(orientation) - 0.5*height*cos(orientation);
  obstacles[0].checkpoints[6] = -0.5*width*cos(orientation) - 0.5*height*sin(orientation);
  obstacles[0].checkpoints[7] = -0.5*width*sin(orientation) + 0.5*height*cos(orientation);

  if(_p2p->n_obs == 2) {
    // obstacle 2 : Spline trajectory -  other vehicle
    std::vector<double> obstacle_trajectory(26, 0.);
    if (_obstacle_trajectory_port.connected()) {
      if(_obstacle_trajectory_port.read(obstacle_trajectory) == RTT::NewData) {
        obstacles[1].traj_coeffs = obstacle_trajectory;
        if (false) {
          std::cout << "obstacle 2 : Spline Trajectory" << std::endl;
          std::cout << "x coefficients : [";
          for(int i=0; i<13; i++) {
            std::cout << obstacle_trajectory[i];
            if(i<12) {
              std::cout << ",";
            }
          }
          std::cout << "]" << std::endl;
          std::cout << "y coefficients : [";
          for(int i=0; i<13; i++) {
            std::cout << obstacle_trajectory[i+13] << ",";
            if(i<12) {
              std::cout << ",";
            }
          }
        std::cout << "]" << std::endl;
        }
      } else {
        std::cout << "No trajectory received from neighbor, using robot arm position" << std::endl;
        for (int i=0; i<13; i++) {
          obstacle_trajectory[i] = _robotarm_pose[0];
          obstacle_trajectory[i+13] = _robotarm_pose[1];
        }
        obstacles[1].traj_coeffs = obstacle_trajectory;
      }
      obstacles[1].avoid = true;
      obstacles[1].radii = std::vector<double>({0.5*max(_neighbor_size[0], _neighbor_size[1])});
      obstacles[1].checkpoints = std::vector<double>({0., 0.});
      std::cout << "received traj (rad=" << obstacles[1].radii[0] << "m): " << std::endl;
      for (int k=0; k<2; k++) {
        std::cout << "[";
        for (int i=0; i<13; i++) {
          std::cout << obstacle_trajectory[13*k+i] << " ";
        }
        std::cout << "]" << std::endl;
      }
    }
  }
}

double FlexonomyMotionPlanning::getMotionTime(){
  std::vector<double> coeff_vector;
  _p2p->getCoefficients(coeff_vector);
  uint n_cfs = coeff_vector.size()/2;
  // extrapolate theta
  double th0 = _est_pose[2];
  for (int k=0; k<int(_update_time/_sample_time); k++) {
    th0 += _ref_velocity[k+_predict_shift][2]*_sample_time;
  }
  double rotation_time = (_target_pose[2] - th0)/_orientation_interpolation_rate;
  double target_dist = sqrt(pow(_target_pose[0] - coeff_vector[n_cfs-1], 2) + pow(_target_pose[1] - coeff_vector[2*n_cfs-1], 2));
  if(target_dist > 0.02) { // end of trajectory is not on destination yet
    double v_mean = 0.87*_vmax;
    double dist = sqrt(pow(_target_pose[0]-coeff_vector[0], 2) + pow(_target_pose[1]-coeff_vector[n_cfs], 2));
    return max(dist/v_mean, rotation_time);
  }
  else {
    int k;
    for (k=0; k<_trajectory_length_full; k++) {
      if(sqrt(pow(_ref_pose[k][0]-_target_pose[0], 2) + pow(_ref_pose[k][1]-_target_pose[1], 2)) <= 0.02) {
        break;
      }
    }
    std::cout << "motion time = " << k*_sample_time << "s" << std::endl;
    return max(k*_sample_time, rotation_time);
  }
}

ORO_LIST_COMPONENT_TYPE(FlexonomyMotionPlanning);
