#include "EstimatorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

EstimatorInterface::EstimatorInterface(std::string const& name) : TaskContext(name, PreOperational),
    _cal_imul_transacc(3), _cal_imul_dorientation(0.), _cal_imul_orientation(0.),
    _cal_imur_transacc(3), _cal_imur_dorientation(0.), _cal_imur_orientation(0.),
    _cal_enc_pose(3), _cal_motor_current(4), _cal_velocity(3),
    _est_pose(3), _est_velocity(3), _est_acceleration(3), _est_global_offset(3){

  _est_pose_port.keepLastWrittenValue(true);

  ports()->addPort("cal_lidar_distances_port", _cal_lidar_x_port).doc("Observed x positions wrt local frame from LIDAR");
  ports()->addPort("cal_lidar_angles_port", _cal_lidar_y_port).doc("Observed y positions wrt local frame from LIDAR");
  ports()->addPort("cal_ir_distances_port", _cal_ir_x_port).doc("Observed x positions wrt local frame from IR's");
  ports()->addPort("cal_ir_angles_port", _cal_ir_y_port).doc("Observed y positions wrt local frame from IR's");
  ports()->addPort("cal_imul_transacc_port", _cal_imul_transacc_port).doc("Translational accelerations of IMU left");
  ports()->addPort("cal_imul_dorientation_port", _cal_imul_dorientation_port).doc("Derivative of orientation of IMU left");
  ports()->addPort("cal_imul_orientation_port", _cal_imul_orientation_port).doc("Orientation of IMU left");
  ports()->addPort("cal_imur_transacc_port", _cal_imur_transacc_port).doc("Translational accelerations of IMU right");
  ports()->addPort("cal_imur_dorientation_port", _cal_imur_dorientation_port).doc("Derivative of orientation of right left");
  ports()->addPort("cal_imur_orientation_port", _cal_imur_orientation_port).doc("Orientation of IMU right");
  ports()->addPort("cal_enc_pose_port", _cal_enc_pose_port).doc("Pose derived from encoders");
  ports()->addPort("cal_motor_current_port", _cal_motor_current_port).doc("Current of 4 motors");
  ports()->addPort("cal_motor_voltage_port", _cal_motor_voltage_port).doc("Voltage of 4 motors");
  ports()->addPort("cal_velocity_port", _cal_velocity_port).doc("Velocity input of system");

  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose wrt to initial frame");
  ports()->addPort("est_velocity_port", _est_velocity_port).doc("Estimated velocity wrt to initial frame");
  ports()->addPort("est_acceleration_port", _est_acceleration_port).doc("Estimated acceleration wrt to initial frame");
  ports()->addPort("est_global_offset_port", _est_global_offset_port).doc("Estimated offset of initial frame wrt world frame");
  ports()->addPort("map_obstacles_port", _map_obstacles_port).doc("Estimated obstacle data wrt to world frame");

  addProperty("control_sample_rate", _control_sample_rate).doc("Frequency to update the control loop");
  addProperty("lidar_data_length", _lidar_data_length).doc("Length of lidar data");
  addProperty("ir_data_length", _ir_data_length).doc("Length of ir data (= number of ir's)");
  addProperty("obs_data_length", _obs_data_length).doc("Length of obstacle data");

  addOperation("writeSample",&EstimatorInterface::writeSample, this).doc("Set data sample on output ports");

}

void EstimatorInterface::writeSample(){
  std::vector<double> example(3, 0.0);
  _est_pose_port.write(example);
  _est_velocity_port.write(example);
  _est_acceleration_port.write(example);
  _est_global_offset_port.write(example);
  std::vector<double> exampleobs(_obs_data_length, 0.0);
  _map_obstacles_port.write(exampleobs);
}

bool EstimatorInterface::configureHook(){
  // Reserve required memory and initialize with zeros
  _cal_lidar_x.resize(_lidar_data_length);
  _cal_lidar_y.resize(_lidar_data_length);
  _cal_ir_x.resize(_ir_data_length);
  _cal_ir_y.resize(_ir_data_length);
  _map_obstacles.resize(_obs_data_length);

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _est_pose_port.setDataSample(example);
  _est_velocity_port.setDataSample(example);
  _est_acceleration_port.setDataSample(example);
  _est_global_offset_port.setDataSample(example);
  std::vector<double> exampleobs(_obs_data_length, 0.0);
  _map_obstacles_port.setDataSample(exampleobs);
  return true;
}

bool EstimatorInterface::startHook(){
  // Check if input ports are connected
  bool check = true;
  if (!_cal_lidar_x_port.connected()){
    log(Warning) << "_cal_lidar_distances_port not connected !" <<endlog();
  }
  if (!_cal_lidar_y_port.connected()){
    log(Warning) << "_cal_lidar_angles_port not connected !" <<endlog();
  }
  if (!_cal_ir_x_port.connected()){
    log(Warning) << "_cal_ir_distances_port not connected !" <<endlog();
  }
  if (!_cal_ir_y_port.connected()){
    log(Warning) << "_cal_ir_angles_port not connected !" <<endlog();
  }
  if (!_cal_imul_transacc_port.connected()){
    log(Warning) << "_cal_imul_transacc_port not connected !" <<endlog();
  }
  if (!_cal_imul_dorientation_port.connected()){
    log(Warning) << "_cal_imul_dorientation_port not connected !" <<endlog();
  }
  if (!_cal_imul_orientation_port.connected()){
    log(Warning) << "_cal_imul_orientation_port not connected !" <<endlog();
  }
  if (!_cal_imur_transacc_port.connected()){
    log(Warning) << "_cal_imur_transacc_port not connected !" <<endlog();
  }
  if (!_cal_imur_dorientation_port.connected()){
    log(Warning) << "_cal_imur_dorientation_port not connected !" <<endlog();
  }
  if (!_cal_imur_orientation_port.connected()){
    log(Warning) << "_cal_imur_orientation_port not connected !" <<endlog();
  }
  if (!_cal_enc_pose_port.connected()){
    log(Warning) << "_cal_enc_pose_port not connected !" <<endlog();
  }
  if (!_cal_motor_current_port.connected()){
    log(Warning) << "_cal_motor_current_port not connected !" <<endlog();
  }
  if (!_cal_motor_current_port.connected()){
    log(Warning) << "_cal_motor_voltage_port not connected !" <<endlog();
  }
  if (!_cal_velocity_port.connected()){
    log(Warning) << "_cal_velocity_port not connected !" <<endlog();
  }
  if (!initialize()){
    log(Error) << "Error occured in initialize() !" <<endlog();
    check = false;
  }
  if (!check){
    return false;
  }
  std::cout << "Estimator started !" <<std::endl;
  return true;
}

void EstimatorInterface::updateHook(){
  // Read from sensors
  // if (_cal_lidar_x_port.read(_cal_lidar_x) == RTT::NoData) {log(Error) << "No data on _cal_lidar_x_port !" <<endlog(); error();}
  // if (_cal_lidar_y_port.read(_cal_lidar_y) == RTT::NoData) {log(Error) << "No data on _cal_lidar_y_port !" <<endlog(); error();}
  // if (_cal_ir_x_port.read(_cal_ir_x) == RTT::NoData) {log(Error) << "No data on _cal_ir_x_port !" <<endlog(); error();}
  // if (_cal_ir_y_port.read(_cal_ir_y) == RTT::NoData) {log(Error) << "No data on _cal_ir_y_port !" <<endlog(); error();}
  // if (_cal_imul_transacc_port.read(_cal_imul_transacc) == RTT::NoData) {log(Error) << "No data on _cal_imul_transacc_port !" <<endlog(); error();}
  // if (_cal_imul_dorientation_port.read(_cal_imul_dorientation) == RTT::NoData) {log(Error) << "No data on _cal_imul_dorientation_port !" <<endlog(); error();}
  // if (_cal_imul_orientation_port.read(_cal_imul_orientation) == RTT::NoData) {log(Error) << "No data on _cal_imul_orientation_port !" <<endlog(); error();}
  // if (_cal_imur_transacc_port.read(_cal_imur_transacc) == RTT::NoData) {log(Error) << "No data on _cal_imur_transacc_port !" <<endlog(); error();}
  // if (_cal_imur_dorientation_port.read(_cal_imur_dorientation) == RTT::NoData) {log(Error) << "No data on _cal_imur_dorientation_port !" <<endlog(); error();}
  // if (_cal_imur_orientation_port.read(_cal_imur_orientation) == RTT::NoData) {log(Error) << "No data on _cal_imur_orientation_port !" <<endlog(); error();}
  // if (_cal_enc_pose_port.read(_cal_enc_pose) == RTT::NoData) {log(Error) << "No data on _cal_enc_pose_port !" <<endlog(); error();}
  // if (_cal_motor_current_port.read(_cal_motor_current) == RTT::NoData) {log(Error) << "No data on _cal_motor_current_port !" <<endlog(); error();}
  // if (_cal_motor_voltage_port.read(_cal_motor_voltage) == RTT::NoData) {log(Error) << "No data on _cal_motor_voltage_port !" <<endlog(); error();}

  // Read velocity command
  std::vector<double> sample(3,0.0);
  if (_cal_velocity_port.read(sample) == RTT::NoData) {
    log(Warning) << "No data on _cal_velocity_port !" <<endlog();
  }
  else {
    _cal_velocity = sample;
  }

  _cal_lidar_x_port.read(_cal_lidar_x);
  _cal_lidar_y_port.read(_cal_lidar_y);
  _cal_ir_x_port.read(_cal_ir_x);
  _cal_ir_y_port.read(_cal_ir_y);
  _cal_imul_transacc_port.read(_cal_imul_transacc);
  _cal_imul_dorientation_port.read(_cal_imul_dorientation);
  _cal_imul_orientation_port.read(_cal_imul_orientation);
  _cal_imur_transacc_port.read(_cal_imur_transacc);
  _cal_imur_dorientation_port.read(_cal_imur_dorientation);
  _cal_imur_orientation_port.read(_cal_imur_orientation);
  _cal_enc_pose_port.read(_cal_enc_pose);
  _cal_motor_current_port.read(_cal_motor_current);
  _cal_motor_voltage_port.read(_cal_motor_voltage);

  // Apply estimation update
  estimateUpdate();

  // Write estimated values
  _est_pose_port.write(_est_pose);
  _est_velocity_port.write(_est_velocity);
  _est_acceleration_port.write(_est_acceleration);
  _est_global_offset_port.write(_est_global_offset);
  _map_obstacles_port.write(_map_obstacles);
}

void EstimatorInterface::stopHook() {
  std::cout << "Estimator stopped !" <<std::endl;
}

double EstimatorInterface::getControlSampleRate(){ return _control_sample_rate; }
int EstimatorInterface::getLidarDataLength(){ return _lidar_data_length; }
int EstimatorInterface::getIRDataLength(){ return _ir_data_length; }
int EstimatorInterface::getObsDataLength(){ return _obs_data_length; }

std::vector<std::vector<double> > EstimatorInterface::getLidarData(){
  std::vector<std::vector<double> > lidardata(2, std::vector<double>(_lidar_data_length));
  lidardata.at(0) = _cal_lidar_x;
  lidardata.at(1) = _cal_lidar_y;
  return lidardata;
}
std::vector<std::vector<double> > EstimatorInterface::getIRData(){
  std::vector<std::vector<double> > irdata(2, std::vector<double>(_ir_data_length));
  irdata.at(0) = _cal_ir_x;
  irdata.at(1) = _cal_ir_y;
  return irdata;
}

std::vector<double> EstimatorInterface::getImuLTransAcc(){ return _cal_imul_transacc; }
double EstimatorInterface::getImuLDOrientation(){ return _cal_imul_dorientation; }
double EstimatorInterface::getImuLOrientation(){ return _cal_imul_orientation; }
std::vector<double> EstimatorInterface::getImuRTransAcc(){ return _cal_imur_transacc; }
double EstimatorInterface::getImuRDOrientation(){return _cal_imur_dorientation; }
double EstimatorInterface::getImuROrientation(){return _cal_imur_orientation; }
std::vector<double> EstimatorInterface::getEncPose(){ return _cal_enc_pose; }
std::vector<double> EstimatorInterface::getMotorCurrent(){ return _cal_motor_current; }
std::vector<double> EstimatorInterface::getMotorVoltage(){ return _cal_motor_voltage; }
std::vector<double> EstimatorInterface::getCalVelocity(){ return _cal_velocity; }

void EstimatorInterface::setEstPose(std::vector<double> const& est_pose){ _est_pose = est_pose; }
void EstimatorInterface::setEstVelocity(std::vector<double> const& est_velocity){ _est_velocity = est_velocity; }
void EstimatorInterface::setEstAcceleration(std::vector<double> const& est_acceleration){ _est_acceleration = est_acceleration; }
void EstimatorInterface::setEstGlobalOffset(std::vector<double> const& est_global_offset){ _est_global_offset = est_global_offset; }
void EstimatorInterface::setObstacleData(std::vector<double> const& map_obstacles){ _map_obstacles = map_obstacles; } // can be reimplemented...

ORO_CREATE_COMPONENT_LIBRARY()
