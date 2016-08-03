#ifndef OROCOS_SCANMATCHER_COMPONENT_HPP
#define OROCOS_SCANMATCHER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <iostream>
#include <vector>
#include <csm/csm_all.h>

class Scanmatcher : public RTT::TaskContext{

  private:
    // Inputs
    RTT::InputPort<std::vector<double> > _cor_lidar_angle_port;
  	RTT::InputPort<std::vector<double> > _cor_lidar_distance_port;
    RTT::InputPort<std::vector<double> > _cal_enc_pose_port;
    RTT::InputPort<bool> _trigger_scanmatcher_port;
    // Outputs
  	RTT::OutputPort<std::vector<double> > _scanmatch_pose_port;
  	RTT::OutputPort<std::vector<double> > _scanmatch_covariance_port;

    int _sensenumber;
    int _lidar_data_length;
  	double _max_sense_range;
    double _max_angular_correction_deg;
    double _max_linear_correction;
    int _use_corr_tricks;
    int _max_iterations;
    double _epsilon_xy;
    double _epsilon_theta;
    double _sigma;
    double _max_correspondence_dist;
    double _orientation_neighbourhood;
    double _outliers_maxPerc;
    int _scans;
    double _outliers_adaptive_order;
    double _outliers_adaptive_mult;
    int _use_point_to_line_distance;
    int _use_odometry_guess;
    int _do_compute_covariance;

    std::vector<double> _enc_pose;
    std::vector<double> _estimated_position_change;
    std::vector<double> _prev_lidar_distances;
    std::vector<double> _prev_lidar_angles;
    std::vector<double> _available_lidar_distances;
    std::vector<double> _available_lidar_angles;
    std::vector<double> _lidar_distances;
    std::vector<double> _lidar_angles;
    std::vector<double> _covariance;

    int _rows;
    int _columns;

    double _lastX;
    double _lastY;
    double _lastTheta;


    sm_params ls;

  public:
    Scanmatcher(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void quickSort(double angles[], double distances[] ,int low, int high);
    void correctInformation(std::vector<double> laser_scan_distance, std::vector<double> laser_scan_angle ,std::vector<double> delta_odo_scan);
};
#endif
