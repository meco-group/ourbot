#include "Kalman-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>

Kalman::Kalman(std::string const& name) : EstimatorInterface(name),
_sigma_odo(3), _cal_velocity(3), _est_pose(3), _marker_data(7)
{
  // add ports
  ports()->addPort("markers_port", _markers_port).doc("Markers + timestamp detected by camera.");

  // add properties
  addProperty("sigma_odo", _sigma_odo).doc("Uncertainty on measured velocity (m/s)^2");
  addProperty("sigma_markers", _sigma_markers).doc("Uncertainty on measured markers (m)^2");
  addProperty("marker_locations", _marker_loc).doc("Location of markers in local frame [x1, y1, x2, y2, x3, y3]");
  addProperty("enable_odo", _enable_odo).doc("Enable odometry measurements");
  addProperty("enable_markers", _enable_markers).doc("Enable marker measurements");
  addProperty("time_offset", _time_offset);
  addProperty("max_cov_det", _max_cov_det).doc("Maximum allowed covariance determinant");
  addProperty("print_cov_det", _print_cov_det).doc("Print covariance determinant");

  _format = Eigen::IOFormat(5, Eigen::DontAlignCols, " ", "\n", "[", "]", "[", "]");
}

bool Kalman::initialize(){
  _kf = new OdometryFilter<3>(_sigma_odo[0], _sigma_odo[1], _sigma_odo[2]);
  _start_time = captureTime();
  _kf->unknown(_start_time);
  _control_sample_rate = getControlSampleRate();
  _Mref << _marker_loc[0], _marker_loc[1], _marker_loc[2], _marker_loc[3], _marker_loc[4], _marker_loc[5];
  return true;
}

double Kalman::captureTime(){
  uint32_t ms;
  double s;
  ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
  s = ms * double(std::chrono::milliseconds::period::num) / std::chrono::milliseconds::period::den;
  return s;
}

bool Kalman::estimateUpdate(){
  _time = captureTime();
  // odometry velocity
  if (_enable_odo){
    _cal_velocity = getCalVelocity();
    _kf->observe_odo(_time, _cal_velocity[0], _cal_velocity[1], _cal_velocity[2]);
  }
  // marker measurements
  if (_markers_port.read(_marker_data) == RTT::NewData){
    _Mmeas << _marker_data[0], _marker_data[1], _marker_data[2], _marker_data[3], _marker_data[4], _marker_data[5];
    _marker_time = _marker_data[6] - _time_offset;
    if (_marker_time > _start_time){
      if (_enable_markers){
        _kf->observe_markers(_marker_time, _Mmeas, _Mref, _sigma_markers);
      }
      _transmit_estimate = true;
    }
  }
  // _time = captureTime();
  _kf->predict(_time, _state, _P);
  _est_pose[0] = _state[0];
  _est_pose[1] = _state[1];
  _est_pose[2] = _state[2];

  // check if estimation is valid
  setValidEstimation(_P.determinant() < _max_cov_det);

  if (_print_cov_det){
    std::cout << "cov det: " << _P.determinant() << std::endl;
  }

  setEstPose(_est_pose);
  return true;
}

ORO_LIST_COMPONENT_TYPE(Kalman);
