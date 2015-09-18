#include "Sensors-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Sensors::Sensors(std::string const& name) : TaskContext(name, PreOperational){
  ports()->addPort("cal_lidar_distances_port", _cal_lidar_distances_port).doc("Distances wrt center from LIDAR");
  ports()->addPort("cal_lidar_angles_port", _cal_lidar_angles_port).doc("Angles from LIDAR");
  ports()->addPort("cal_ir_distances_port", _cal_ir_distances_port).doc("Distances wrt center from IR's");
  ports()->addPort("cal_ir_angles_port", _cal_ir_angles_port).doc("Angles from IR's");
  ports()->addPort("cal_imul_transacc_port", _cal_imul_transacc_port).doc("Translational accelerations of IMU left");
  ports()->addPort("cal_imul_dorientation_port", _cal_imul_dorientation_port).doc("Derivative of orientation of IMU left");
  ports()->addPort("cal_imul_orientation_port", _cal_imul_orientation_port).doc("Orientation of IMU left");
  ports()->addPort("cal_imur_transacc_port", _cal_imur_transacc_port).doc("Translational accelerations of IMU right");
  ports()->addPort("cal_imur_dorientation_port", _cal_imur_dorientation_port).doc("Derivative of orientation of right left");
  ports()->addPort("cal_imur_orientation_port", _cal_imur_orientation_port).doc("Orientation of IMU right");
  ports()->addPort("cal_enc_pose_port", _cal_enc_pose_port).doc("Pose derived from encoders");
  ports()->addPort("cal_motor_current_port", _cal_motor_current_port).doc("Current of 4 motors");
  ports()->addPort("cal_velocity_port", _cal_velocity_port).doc("Velocity input of system");

  addProperty("lidar_data_length", _lidar_data_length).doc("Length of lidar data");
  addProperty("ir_data_length", _ir_data_length).doc("Length of ir data (= number of ir's)");
  addProperty("obs_data_length", _obs_data_length).doc("Length of obstacle data");
}

bool Sensors::configureHook(){
  // Set data sample for real-time data flow
  std::vector<double> example_lidar(_lidar_data_length, 0.0);
  _cal_lidar_distances_port.setDataSample(example_lidar);
  _cal_lidar_angles_port.setDataSample(example_lidar);
  std::vector<double> example_ir(_ir_data_length, 0.0);
  _cal_ir_distances_port.setDataSample(example_ir);
  _cal_ir_angles_port.setDataSample(example_ir);
  std::vector<double> example(3,0.0);
  _cal_imul_transacc_port.setDataSample(example);
  _cal_imur_transacc_port.setDataSample(example);
  _cal_enc_pose_port.setDataSample(example);
  std::vector<double> example2(4,0.0);
  _cal_motor_current_port.setDataSample(example2);
  _cal_velocity_port.setDataSample(example);

  return true;
}

bool Sensors::startHook(){
  std::cout << "Sensors started !" <<std::endl;
  return true;
}

void Sensors::updateHook(){
  std::vector<double> example_lidar(_lidar_data_length, 0.0);
  _cal_lidar_distances_port.write(example_lidar);
  _cal_lidar_angles_port.write(example_lidar);
  std::vector<double> example_ir(_ir_data_length, 0.0);
  _cal_ir_distances_port.write(example_ir);
  _cal_ir_angles_port.write(example_ir);
  std::vector<double> example(3,0.0);
  _cal_imul_transacc_port.write(example);
  _cal_imur_transacc_port.write(example);
  _cal_enc_pose_port.write(example);
  std::vector<double> example2(4,0.0);
  _cal_motor_current_port.write(example2);
  _cal_velocity_port.write(example);

  _cal_imul_dorientation_port.write(1.);
  _cal_imul_orientation_port.write(1.);
  _cal_imur_dorientation_port.write(1.);
  _cal_imur_orientation_port.write(1.);
}

void Sensors::stopHook() {
  std::cout << "Sensors stopped !" <<std::endl;
}

ORO_CREATE_COMPONENT(Sensors)
