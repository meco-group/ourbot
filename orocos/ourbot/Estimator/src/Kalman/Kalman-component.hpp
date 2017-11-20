#ifndef OROCOS_KALMAN_COMPONENT_HPP
#define OROCOS_KALMAN_COMPONENT_HPP

#include "async_kalman/kalman_odometry.hpp"
#include "../EstimatorInterface/EstimatorInterface-component.hpp"

using namespace RTT;

class Kalman : public EstimatorInterface {
  private:
    InputPort <std::vector<double> >  _detected_pose_port;

    std::vector<double> _psd_state;
    std::vector<double> _sigma_odo;
    double _sigma_markers;
    bool _enable_odo;
    bool _enable_markers;
    double _max_cov_det;
    bool _print_cov_det;

    OdometryFilter<3>* _kf;
    M<6, 1> _state;
    M<6, 6> _cov;

    M<3, 2> _Mref;
    M<3, 2> _Mmeas;

    double _time;
    double _start_time;
    double _marker_time;

    std::vector<double> _cal_velocity;
    std::vector<double> _est_pose;
    std::vector<double> _est_velocity;
    std::vector<double> _detected_pose;

    double captureTime();

  public:
    Kalman(std::string const& name);
    bool valid();
    bool estimateHook();
    bool initialize();
};

#endif
