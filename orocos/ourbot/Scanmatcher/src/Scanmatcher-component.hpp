#ifndef OROCOS_SCANMATCHER_COMPONENT_HPP
#define OROCOS_SCANMATCHER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <csm/csm_all.h>
#include "rapidxml.hpp"


class Scanmatcher : public RTT::TaskContext{

  private:
    // Inputs
    RTT::InputPort<std::vector<double> > _cor_lidar_angle_port;
  	RTT::InputPort<std::vector<double> > _cor_lidar_distance_port;
    RTT::InputPort<std::vector<double> > _cal_enc_pose_port;
    RTT::InputPort<bool> _trigger_scanmatcher_port;
    // RTT::InputPort<bool> _scanstart_pose_port; TODO
    // Outputs
    //@@@Michiel: add extra output port with pose at start scan
    RTT::InputPort<std::vector<double> > _scanstart_pose_port;
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
    double _precision;
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

    std::vector< std::vector<double> > _environment_circles;
    std::vector< std::vector< std::vector<double> > > _environment_polygons;
    std::vector<double> _start_pose;

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
    void artificialLidar();
    double getIntersectDistancePolygon(double const& x_a, double const& y_a, double const& x_b, double const& y_b, double const& angle);
    double getIntersectDistanceCircle(double const& x_centre, double const& y_centre, double const& radius, double const& angle);
    bool inRange(double const& boundarie_1, double const& boundarie_2, double const& value);
    bool greaterThan(double const& boundarie, double const& value);
    bool smallerThan(double const& boundarie, double const& value);
    bool correctIntersection(double const& x, double const& y, double const& x_intersect, double const& y_intersect, double const& angle);
    void loadEnvironment();
};
#endif
