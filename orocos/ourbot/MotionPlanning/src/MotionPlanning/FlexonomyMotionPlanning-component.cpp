#include "FlexonomyMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


using namespace std;

FlexonomyMotionPlanning::FlexonomyMotionPlanning(std::string const& name) : MotionPlanning(name), _robotarm_pose(3){

  //Other vehicle trajectory input port
  ports()->addPort("obstacle_trajectory_port", _obstacle_trajectory_port).doc("Other vehicle Obstacle trajectory port");
  ports()->addPort("host_obstacle_trajectory_port", _host_obstacle_trajectory_port).doc("Host vehicle Obstacle trajectory port");
  ports()->addPort("robot_markers_port", _robot_markers_port).doc("Markers of robot arm table");

  //Robot arm properties
  addProperty("robotarm_size", _robotarm_size).doc("Robot arm size [width, height]");
  addProperty("neighbor_size", _neighbor_size).doc("Neighbor ourbot size [width, height]");
  addProperty("robotarm_marker_locations", _robotarm_marker_locations).doc("Locations of markers in robot table frame");

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

void FlexonomyMotionPlanning::getRobotArmPose(std::vector<double>& robotarm_pose) {
  if (_robot_markers_port.read(_robot_markers) == RTT::NewData) {
    robotarm_pose[0] = (_robot_markers[0] + _robot_markers[2] + _robot_markers[4])/3.;
    robotarm_pose[1] = (_robot_markers[1] + _robot_markers[3] + _robot_markers[5])/3.;
    robotarm_pose[2] = atan2((_robot_markers[3] - _robot_markers[1]), (_robot_markers[2] - _robot_markers[0])) - 0.5*M_PI;

    double robot_marker_x = -(_robotarm_marker_locations[0] + _robotarm_marker_locations[2] + _robotarm_marker_locations[4])/3.;
    double robot_marker_y = -(_robotarm_marker_locations[1] + _robotarm_marker_locations[3] + _robotarm_marker_locations[5])/3.;

    robotarm_pose[0] += robot_marker_x*cos(robotarm_pose[2]) - robot_marker_y*sin(robotarm_pose[2]);
    robotarm_pose[1] += robot_marker_x*sin(robotarm_pose[2]) + robot_marker_y*cos(robotarm_pose[2]);

  } else {
    std::cout << "No robot table position detected, putting it out of reach." << std::endl;
    robotarm_pose[0] = -_robotarm_size[0];
    robotarm_pose[1] = -_robotarm_size[1];
    robotarm_pose[2] = 0.;
  }

}

void FlexonomyMotionPlanning::getObstacles(std::vector<omg::obstacle_t>& obstacles) {
  // update pose of robot arm
  getRobotArmPose(_robotarm_pose);

  // obstacle 1 : Classical -  Robot arm
  obstacles[0].position = std::vector<double>({_robotarm_pose[0], _robotarm_pose[1]});
  obstacles[0].avoid = true;
  double orientation = _robotarm_pose[2];
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
      // std::cout << "received traj (rad=" << obstacles[1].radii[0] << "m): " << std::endl;
      // for (int k=0; k<2; k++) {
      //   std::cout << "[";
      //   for (int i=0; i<13; i++) {
      //     std::cout << obstacle_trajectory[13*k+i] << " ";
      //   }
      //   std::cout << "]" << std::endl;
      // }
    }
  }
}

ORO_LIST_COMPONENT_TYPE(FlexonomyMotionPlanning);
