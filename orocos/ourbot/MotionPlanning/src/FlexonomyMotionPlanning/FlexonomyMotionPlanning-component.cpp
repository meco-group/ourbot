#include "FlexonomyMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

FlexonomyMotionPlanning::FlexonomyMotionPlanning(std::string const& name) : MotionPlanning(name) {
}

bool FlexonomyMotionPlanning::config() {
    p2pflex::Vehicle* vehicle = new p2pflex::Holonomic();
    if (_ideal_prediction) {
        vehicle->setIdealPrediction(true);
    }
    _n_st = vehicle->getNState();
    _n_in = vehicle->getNInput();
    _problem = new Problem<p2pflex::Point2Point>(new p2pflex::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length));

    _ref_pose.resize(_trajectory_length);
    _ref_velocity.resize(_trajectory_length);
    for (int k=0; k<_trajectory_length; k++) {
        _ref_pose[k].resize(_n_st);
        _ref_velocity[k].resize(_n_in);
    }
    std::cout << "Loaded motion planning problem with " << n_obstacles() << " obstacles." << std::endl;
    return true;
}

bool FlexonomyMotionPlanning::updatePositionTrajectory() {
    // update motion planning algorithm
    p2pflex::Point2Point* problem = ((Problem<p2pflex::Point2Point>*)_problem)->get();
    bool check = problem->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, *(std::vector<p2pflex::obstacle_t>*)&_obstacles, _predict_shift);
    save(_ref_pose, _ref_velocity);
    return check;
}

ORO_LIST_COMPONENT_TYPE(FlexonomyMotionPlanning);
