#include "Kalman-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <iomanip>

Kalman::Kalman(std::string const& name) : EstimatorInterface(name),
_psd_state(3), _sigma_odo(3), _cal_velocity(3), _est_pose(3), _marker_data(7)
{
  // add ports
  ports()->addPort("markers_port", _markers_port).doc("Markers + timestamp detected by camera.");

  // add properties
  addProperty("psd_state", _psd_state).doc("Acceleration-like squared noise levels for x,y,theta (m/s^2)^2");
  addProperty("sigma_odo", _sigma_odo).doc("Uncertainty on measured velocity (m/s)^2");
  addProperty("sigma_markers", _sigma_markers).doc("Uncertainty on measured markers (m)^2");

  _format = Eigen::IOFormat(5, Eigen::DontAlignCols, " ", "\n", "[", "]", "[", "]");

}

bool Kalman::initialize(){
  _kf = new OdometryFilter<3>(_psd_state[0], _psd_state[1], _psd_state[2], 100);
  _kf->unknown(0);
  _control_sample_rate = getControlSampleRate();
  _time = 0.0;
  return true;
}

bool Kalman::estimateUpdate(){
  // odometry velocity
  _cal_velocity = getCalVelocity()
  _kf->observe_odo(_time, _cal_velocity[0], _cal_velocity[1], _cal_velocity[2], _sigma_odo[0], _sigma_odo[1], _sigma_odo[2]);

  // marker measurements
  if (_markers_port.read(_marker_data) == RTT::NewData){
    _Mmeas << _marker_data[0], _marker_data[1], _marker_data[2], _marker_data[3], _marker_data[4], _marker_data[5];
    _kf->observe_markers(_marker_data[6], _Mmeas, _Mref, _sigma_markers);
  }
  _kf->predict(_time, _state, _P);
  _time += 1./_control_sample_rate;
  _est_pose[0] = _state[OFF_X];
  _est_pose[1] = _state[OFF_Y];
  _est_pose[2] = _state[OFF_THETA];
  setEstPose(_est_pose);
  return true;
}

ORO_LIST_COMPONENT_TYPE(Kalman);
