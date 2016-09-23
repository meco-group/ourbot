#ifndef OROCOS_KALMAN_COMPONENT_HPP
#define OROCOS_KALMAN_COMPONENT_HPP

#include "kalman_odometry.hpp"
#include "../EstimatorInterface/EstimatorInterface-component.hpp"

using namespace RTT;

class Kalman : public EstimatorInterface{
  private:
    // ports
    InputPort <std::vector<double> >  _markers_port;

    // properties
    std::vector<double> _psd_state;
    std::vector<double> _sigma_odo;
    double _sigma_markers;
    std::vector<double> _marker_loc;

    // kalman variables
    OdometryFilter<3>* _kf;
    Eigen::IOFormat _format;
    M<6, 1> _state;
    M<6, 6> _P;
    M<3, 2> _Mref;
    M<3, 2> _Mmeas;

    double _time;
    double _start_time;
    double _control_sample_rate;
    double _time_offset;
    double _marker_time;

    std::vector<double> _cal_velocity;
    std::vector<double> _est_pose;
    std::vector<double> _est_velocity;
    std::vector<double> _marker_data;

    double captureTime();

  public:
    Kalman(std::string const& name);
    bool estimateUpdate();
    bool initialize();
};

#endif
