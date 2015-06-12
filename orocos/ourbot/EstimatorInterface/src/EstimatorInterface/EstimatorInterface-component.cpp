#include "EstimatorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

EstimatorInterface::EstimatorInterface(std::string const& name) : TaskContext(name),
    _cal_imul_transacc(3), _cal_imul_dorientation(0.), _cal_imul_orientation(0.),
    _cal_imur_transacc(3), _cal_imur_dorientation(0.), _cal_imur_orientation(0.),
    _cal_enc_pose(3), _cal_motor_current(4), _cmd_velocity(3),
    _est_pose(3), _est_velocity(3), _est_acceleration(3), _est_global_offset(3){

  ports()->addPort("cal_lidar_distances_inport", _cal_lidar_distances_inport).doc("Distances wrt center from LIDAR");
  ports()->addPort("cal_lidar_angles_inport", _cal_lidar_angles_inport).doc("Angles from LIDAR");
  ports()->addPort("cal_ir_distances_inport", _cal_ir_distances_inport).doc("Distances wrt center from IR's");
  ports()->addPort("cal_ir_angles_inport", _cal_ir_angles_inport).doc("Angles from IR's");
  ports()->addPort("cal_imul_transacc_inport", _cal_imul_transacc_inport).doc("Translational accelerations of IMU left");
  ports()->addPort("cal_imul_dorientation_inport", _cal_imul_dorientation_inport).doc("Derivative of orientation of IMU left");
  ports()->addPort("cal_imul_orientation_inport", _cal_imul_orientation_inport).doc("Orientation of IMU left");
  ports()->addPort("cal_imur_transacc_inport", _cal_imur_transacc_inport).doc("Translational accelerations of IMU right");
  ports()->addPort("cal_imur_dorientation_inport", _cal_imur_dorientation_inport).doc("Derivative of orientation of right left");
  ports()->addPort("cal_imur_orientation_inport", _cal_imur_orientation_inport).doc("Orientation of IMU right");
  ports()->addPort("cal_enc_pose_inport", _cal_enc_pose_inport).doc("Pose derived from encoders");
  ports()->addPort("cal_motor_current_inport", _cal_motor_current_inport).doc("Current of 4 motors");
  ports()->addPort("cmd_velocity_inport", _cmd_velocity_inport).doc("Velocity command for actuator");

  ports()->addPort("est_pose_outport", _est_pose_outport).doc("Estimated pose wrt to initial frame");
  ports()->addPort("est_velocity_outport", _est_velocity_outport).doc("Estimated velocity wrt to initial frame");
  ports()->addPort("est_acceleration_outport", _est_acceleration_outport).doc("Estimated acceleration wrt to initial frame");
  ports()->addPort("est_global_offset_outport", _est_global_offset_outport).doc("Estimated offset of initial frame wrt world frame");
  ports()->addPort("map_obstacles_outport", _map_obstacles_outport).doc("Estimated obstacle data wrt to world frame");
}

bool EstimatorInterface::configureHook(){
  // Get configurator peer and sample rate
  TaskContext* configurator = getPeer("configurator");
  if (configurator==NULL){
    log(Error) << "No peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getControlSampleRate = configurator->getOperation("getControlSampleRate");
  if (!getControlSampleRate.ready()){
    log(Error) << "No operation getControlSampleRate of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getLidarDataLength = configurator->getOperation("getLidarDataLength");
  if (!getLidarDataLength.ready()){
    log(Error) << "No operation getLidarDataLength of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getNrofIR = configurator->getOperation("getNrofIR");
  if (!getNrofIR.ready()){
    log(Error) << "No operation getNrofIR of peer configurator !" <<endlog();
    return false;
  }
  OperationCaller<int(void)> getObsDataLength = configurator->getOperation("getObsDataLength");
  if (!getObsDataLength.ready()){
    log(Error) << "No operation getObsDataLength of peer configurator !" <<endlog();
    return false;
  }

  _sample_rate = getControlSampleRate();
  _lidar_data_length = getLidarDataLength();
  _nrof_ir = getNrofIR();
  _obs_data_length = getObsDataLength();

  // Reserve required memory and initialize with zeros
  _cal_lidar_distances.resize(_lidar_data_length);
  _cal_lidar_angles.resize(_lidar_data_length);
  _cal_ir_distances.resize(_nrof_ir);
  _cal_ir_angles.resize(_nrof_ir);
  _map_obstacles.resize(_obs_data_length);

  // Show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _est_pose_outport.setDataSample(example);
  _est_velocity_outport.setDataSample(example);
  _est_acceleration_outport.setDataSample(example);
  _est_global_offset_outport.setDataSample(example);
  std::vector<double> exampleobs(_obs_data_length, 0.0);
  _map_obstacles_outport.setDataSample(exampleobs);
  return true;
}

bool EstimatorInterface::startHook(){
  // Check if input ports are connected
  bool check = true;
  if (!_cal_lidar_distances_inport.connected()){
    log(Error) << "_cal_lidar_distances_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_lidar_angles_inport.connected()){
    log(Error) << "_cal_lidar_angles_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_ir_distances_inport.connected()){
    log(Error) << "_cal_ir_distances_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_ir_angles_inport.connected()){
    log(Error) << "_cal_ir_angles_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_imul_transacc_inport.connected()){
    log(Error) << "_cal_imul_transacc_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_imul_dorientation_inport.connected()){
    log(Error) << "_cal_imul_dorientation_inport not connected !" <<endlog();
    check = false;
  }

  if (!_cal_imul_orientation_inport.connected()){
    log(Error) << "_cal_imul_orientation_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_imur_transacc_inport.connected()){
    log(Error) << "_cal_imur_transacc_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_imur_dorientation_inport.connected()){
    log(Error) << "_cal_imur_dorientation_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_imur_orientation_inport.connected()){
    log(Error) << "_cal_imur_orientation_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_enc_pose_inport.connected()){
    log(Error) << "_cal_enc_pose_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cal_motor_current_inport.connected()){
    log(Error) << "_cal_motor_current_inport not connected !" <<endlog();
    check = false;
  }
  if (!_cmd_velocity_inport.connected()){
    log(Error) << "_cmd_velocity_inport not connected !" <<endlog();
    check = false;
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
  if (_cal_lidar_distances_inport.read(_cal_lidar_distances) == RTT::NoData) {log(Error) << "No data on _cal_lidar_distances_inport !" <<endlog(); fatal();}
  if (_cal_lidar_angles_inport.read(_cal_lidar_angles) == RTT::NoData) {log(Error) << "No data on _cal_lidar_angles_inport !" <<endlog(); fatal();}
  if (_cal_ir_distances_inport.read(_cal_ir_distances) == RTT::NoData) {log(Error) << "No data on _cal_ir_distances_inport !" <<endlog(); fatal();}
  if (_cal_ir_angles_inport.read(_cal_ir_angles) == RTT::NoData) {log(Error) << "No data on _cal_ir_angles_inport !" <<endlog(); fatal();}
  if (_cal_imul_transacc_inport.read(_cal_imul_transacc) == RTT::NoData) {log(Error) << "No data on _cal_imul_transacc_inport !" <<endlog(); fatal();}
  if (_cal_imul_dorientation_inport.read(_cal_imul_dorientation) == RTT::NoData) {log(Error) << "No data on _cal_imul_dorientation_inport !" <<endlog(); fatal();}
  if (_cal_imul_orientation_inport.read(_cal_imul_orientation) == RTT::NoData) {log(Error) << "No data on _cal_imul_orientation_inport !" <<endlog(); fatal();}
  if (_cal_imur_transacc_inport.read(_cal_imur_transacc) == RTT::NoData) {log(Error) << "No data on _cal_imur_transacc_inport !" <<endlog(); fatal();}
  if (_cal_imur_dorientation_inport.read(_cal_imur_dorientation) == RTT::NoData) {log(Error) << "No data on _cal_imur_dorientation_inport !" <<endlog(); fatal();}
  if (_cal_imur_orientation_inport.read(_cal_imur_orientation) == RTT::NoData) {log(Error) << "No data on _cal_imur_orientation_inport !" <<endlog(); fatal();}
  if (_cal_enc_pose_inport.read(_cal_enc_pose) == RTT::NoData) {log(Error) << "No data on _cal_enc_pose_inport !" <<endlog(); fatal();}
  if (_cal_motor_current_inport.read(_cal_motor_current) == RTT::NoData) {log(Error) << "No data on _cal_motor_current_inport !" <<endlog(); fatal();}

  // Read velocity command
  if (_cmd_velocity_inport.read(_cmd_velocity) == RTT::NoData) {log(Error) << "No data on _cmd_velocity_inport !" <<endlog(); fatal();}

  // Apply estimation update
  estimateUpdate();

  // Write vestimated values
  _est_pose_outport.write(_est_pose);
  _est_velocity_outport.write(_est_velocity);
  _est_acceleration_outport.write(_est_acceleration);
  _est_global_offset_outport.write(_est_global_offset);
  _map_obstacles_outport.write(_map_obstacles);

  log(Info) << "Estimator updated !" <<endlog();
}

void EstimatorInterface::stopHook() {
  std::cout << "Estimator stopped !" <<std::endl;
}

int EstimatorInterface::getSampleRate(){ return _sample_rate; }
int EstimatorInterface::getLidarDataLength(){ return _lidar_data_length; }
int EstimatorInterface::getNrofIR(){ return _nrof_ir; }
int EstimatorInterface::getObsDataLength(){ return _obs_data_length; }

std::vector<std::vector<double> > EstimatorInterface::getLidarData(){
  std::vector<std::vector<double> > lidardata(2, std::vector<double>(_lidar_data_length));
  lidardata.at(0) = _cal_lidar_distances;
  lidardata.at(1) = _cal_lidar_angles;
  return lidardata;
}
std::vector<std::vector<double> > EstimatorInterface::getIRData(){
  std::vector<std::vector<double> > irdata(2, std::vector<double>(_nrof_ir));
  irdata.at(0) = _cal_ir_distances;
  irdata.at(1) = _cal_ir_angles;
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
std::vector<double> EstimatorInterface::getCmdVelocity(){ return _cmd_velocity; }

void EstimatorInterface::setEstPose(std::vector<double> const& est_pose){ _est_pose = est_pose; }
void EstimatorInterface::setEstVelocity(std::vector<double> const& est_velocity){ _est_velocity = est_velocity; }
void EstimatorInterface::setEstAcceleration(std::vector<double> const& est_acceleration){ _est_acceleration = est_acceleration; }
void EstimatorInterface::setEstGlobalOffset(std::vector<double> const& est_global_offset){ _est_global_offset = est_global_offset; }
void EstimatorInterface::setObstacleData(std::vector<double> const& map_obstacles){ _map_obstacles = map_obstacles; } // can be reimplemented...
