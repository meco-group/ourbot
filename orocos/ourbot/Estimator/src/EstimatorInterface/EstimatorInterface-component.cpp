#include "EstimatorInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

EstimatorInterface::EstimatorInterface(std::string const& name) : TaskContext(name, PreOperational),
    _cal_imul_transacc(3), _cal_imul_orientation_3d(3), _cal_imul_orientation(0.),
    _cal_imul_dorientation_3d(3), _cal_imul_dorientation(0.),
    _cal_imur_transacc(3), _cal_imur_orientation_3d(3), _cal_imur_orientation(0.),
    _cal_imur_dorientation_3d(3), _cal_imur_dorientation(0.),
    _cal_enc_pose(3), _cal_motor_current(4), _cal_motor_voltage(4), _cal_velocity(3),
    _est_pose(3), _est_velocity(3), _est_acceleration(3) {

  _est_pose_port.keepLastWrittenValue(true);

  // lidar
  ports()->addPort("cal_lidar_x_port", _cal_lidar_x_port).doc("lidar: Observed x positions wrt local frame");
  ports()->addPort("cal_lidar_y_port", _cal_lidar_y_port).doc("lidar: Observed y positions wrt local frame");
  // imu left
  ports()->addPort("cal_imul_transacc_port", _cal_imul_transacc_port).doc("imul: Translational accelerations");
  ports()->addPort("cal_imul_orientation_3d_port", _cal_imul_orientation_3d_port).doc("imul: Roll Pitch Yaw");
  ports()->addPort("cal_imul_orientation_port", _cal_imul_orientation_port).doc("imul: Orientation around z-axis (Yaw)");
  ports()->addPort("cal_imul_dorientation_3d_port", _cal_imul_dorientation_3d_port).doc("imul: Derivative of Roll Pitch Yaw");
  ports()->addPort("cal_imul_dorientation_port", _cal_imul_dorientation_port).doc("imul: Derivative of orientation aroun z-axis (Yaw)");
  // imu right
  ports()->addPort("cal_imur_transacc_port", _cal_imur_transacc_port).doc("imur: Translational accelerations");
  ports()->addPort("cal_imur_orientation_3d_port", _cal_imur_orientation_3d_port).doc("imur: Roll Pitch Yaw");
  ports()->addPort("cal_imur_orientation_port", _cal_imur_orientation_port).doc("imur: Orientation around z-axis (Yaw)");
  ports()->addPort("cal_imur_dorientation_3d_port", _cal_imur_dorientation_3d_port).doc("imur: Derivative of Roll Pitch Yaw");
  ports()->addPort("cal_imur_dorientation_port", _cal_imur_dorientation_port).doc("imur: Derivative of orientation aroun z-axis (Yaw)");
  // teensy
  ports()->addPort("cal_enc_pose_port", _cal_enc_pose_port).doc("teensy: Pose derived from encoders");
  ports()->addPort("cal_motor_current_port", _cal_motor_current_port).doc("teensy: Current of 4 motors");
  ports()->addPort("cal_motor_voltage_port", _cal_motor_voltage_port).doc("teensy: Voltage of 4 motors");
  ports()->addPort("cal_velocity_port", _cal_velocity_port).doc("teensy: Velocity input of system");
  ports()->addPort("cmd_velocity_passthrough_port", _cmd_velocity_passthrough_port).doc("teensy: Velocity input of system");

  ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose wrt to initial frame");
  ports()->addPort("est_velocity_port", _est_velocity_port).doc("Estimated velocity wrt to initial frame");
  ports()->addPort("est_acceleration_port", _est_acceleration_port).doc("Estimated acceleration wrt to initial frame");

  addProperty("lidar_data_length", _lidar_data_length).doc("Length of lidar data");

  addOperation("valid", &EstimatorInterface::valid, this).doc("Is the last estimation valid?");
  addOperation("getEstimatedPose", &EstimatorInterface::getEstimatedPose, this).doc("Get last estimated pose");
  addOperation("getEstimatedVelocity", &EstimatorInterface::getEstimatedVelocity, this).doc("Get last estimated velocity");
  addOperation("getEstimatedAcceleration", &EstimatorInterface::getEstimatedAcceleration, this).doc("Get last estimated acceleration");
}

bool EstimatorInterface::valid() {
  return true;
}

std::vector<double> EstimatorInterface::getEstimatedPose() {
  return _est_pose;
}

std::vector<double> EstimatorInterface::getEstimatedVelocity() {
  return _est_velocity;
}

std::vector<double> EstimatorInterface::getEstimatedAcceleration() {
  return _est_acceleration;
}

bool EstimatorInterface::configureHook() {
  // reserve required memory and initialize with zeros
  _cal_lidar_x.resize(_lidar_data_length);
  _cal_lidar_y.resize(_lidar_data_length);

  // show example data sample to ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  _est_pose_port.setDataSample(example);
  _est_velocity_port.setDataSample(example);
  _est_acceleration_port.setDataSample(example);
  return true;
}

bool EstimatorInterface::startHook() {
  if (!initialize()) {
    log(Error) << "Error occured in initialize()!" << endlog();
    return false;
  }
  return true;
}

void EstimatorInterface::updateHook() {
  // read from sensors
  _cal_lidar_x_port.read(_cal_lidar_x);
  _cal_lidar_y_port.read(_cal_lidar_y);
  _cal_imul_transacc_port.read(_cal_imul_transacc);
  _cal_imul_orientation_3d_port.read(_cal_imul_orientation_3d);
  _cal_imul_orientation_port.read(_cal_imul_orientation);
  _cal_imul_dorientation_3d_port.read(_cal_imul_dorientation_3d);
  _cal_imul_dorientation_port.read(_cal_imul_dorientation);
  _cal_imur_transacc_port.read(_cal_imur_transacc);
  _cal_imur_orientation_3d_port.read(_cal_imur_orientation_3d);
  _cal_imur_orientation_port.read(_cal_imur_orientation);
  _cal_imur_dorientation_3d_port.read(_cal_imur_dorientation_3d);
  _cal_imur_dorientation_port.read(_cal_imur_dorientation);
  _cal_enc_pose_port.read(_cal_enc_pose);
  _cal_motor_current_port.read(_cal_motor_current);
  _cal_motor_voltage_port.read(_cal_motor_voltage);
  _cal_velocity_port.read(_cal_velocity);
  _cmd_velocity_passthrough_port.read(_cmd_velocity);
  // apply estimation update
  estimateHook();
  // write estimated values
  _est_pose_port.write(_est_pose);
  _est_velocity_port.write(_est_velocity);
  _est_acceleration_port.write(_est_acceleration);
}

std::vector<std::vector<double> > EstimatorInterface::getLidarData() {
  std::vector<std::vector<double> > lidar_data(2, std::vector<double>(_lidar_data_length));
  lidar_data.at(0) = _cal_lidar_x;
  lidar_data.at(1) = _cal_lidar_y;
  return lidar_data;
}

std::vector<double> EstimatorInterface::getImuLTransAcc() {
  return _cal_imul_transacc;
}

double EstimatorInterface::getImuLOrientation() {
  return _cal_imul_orientation;
}

std::vector<double> EstimatorInterface::getImuL3dOrientation() {
  return _cal_imul_orientation_3d;
}

double EstimatorInterface::getImuLDOrientation() {
  return _cal_imul_dorientation;
}

std::vector<double> EstimatorInterface::getImuL3dDOrientation() {
  return _cal_imul_dorientation_3d;
}

std::vector<double> EstimatorInterface::getImuRTransAcc() {
  return _cal_imur_transacc;
}

double EstimatorInterface::getImuROrientation() {
  return _cal_imur_orientation;
}

std::vector<double> EstimatorInterface::getImuR3dOrientation() {
  return _cal_imur_orientation_3d;
}

double EstimatorInterface::getImuRDOrientation() {
  return _cal_imur_dorientation;
}

std::vector<double> EstimatorInterface::getImuR3dDOrientation() {
  return _cal_imur_dorientation_3d;
}

std::vector<double> EstimatorInterface::getEncPose() {
  return _cal_enc_pose;
}

std::vector<double> EstimatorInterface::getMotorCurrent() {
  return _cal_motor_current;
}

std::vector<double> EstimatorInterface::getMotorVoltage() {
  return _cal_motor_voltage;
}

std::vector<double> EstimatorInterface::getCalVelocity() {
  return _cal_velocity;
}

std::vector<double> EstimatorInterface::getCmdVelocity() {
  return _cmd_velocity;
}

void EstimatorInterface::setEstPose(std::vector<double> const& est_pose) {
  _est_pose = est_pose;
}

void EstimatorInterface::setEstVelocity(std::vector<double> const& est_velocity) {
  _est_velocity = est_velocity;
}

void EstimatorInterface::setEstAcceleration(std::vector<double> const& est_acceleration) {
  _est_acceleration = est_acceleration;
}

ORO_CREATE_COMPONENT_LIBRARY()
