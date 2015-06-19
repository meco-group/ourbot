#include "ExampleEstimator-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ExampleEstimator::ExampleEstimator(std::string const& name) : EstimatorInterface(name), _cnt(0){

}

bool ExampleEstimator::initialize(){

  return true;
}

bool ExampleEstimator::estimateUpdate(){
  // Measurements
  std::vector<std::vector<double> > lidar_data  = getLidarData();
  std::vector<std::vector<double> > ir_data     = getIRData();
  std::vector<double> imul_trans_acc            = getImuLTransAcc();
  double imul_dorientation                      = getImuLDOrientation();
  double imul_orientation                       = getImuLOrientation();
  std::vector<double> imur_trans_acc            = getImuRTransAcc();
  double imur_dorientation                      = getImuRDOrientation();
  double imur_orientation                       = getImuROrientation();
  std::vector<double> enc_pose                  = getEncPose();
  std::vector<double> motor_current             = getMotorCurrent();
  std::vector<double> cal_velocity              = getCalVelocity();

  // Dummy state update
  std::vector<double> est_pose(3, 0.0);
  std::vector<double> est_velocity(3, 0.0);
  std::vector<double> est_acceleration(3, 0.0);

  est_pose.at(0) = _cnt;
  est_pose.at(1) = _cnt;
  est_pose.at(2) = _cnt;

  _cnt++;

  // Write estimated parameters
  setEstPose(est_pose);
  setEstVelocity(est_velocity);
  setEstAcceleration(est_acceleration);

  return true;
}

ORO_LIST_COMPONENT_TYPE(ExampleEstimator);
