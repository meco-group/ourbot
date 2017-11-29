#include "MotionPlanningDynamicObstacle-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanningDynamicObstacle::MotionPlanningDynamicObstacle(std::string const& name) : MotionPlanning(name) {
}

bool MotionPlanningDynamicObstacle::config() {
    p2pdynobst::Vehicle* vehicle = new p2pdynobst::Holonomic();
    if (_ideal_prediction) {
        vehicle->setIdealPrediction(true);
    }
    _n_st = vehicle->getNState();
    _n_in = vehicle->getNInput();
    _problem = new Problem<p2pdynobst::Point2Point>(new p2pdynobst::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length));

    _ref_pose.resize(_trajectory_length);
    _ref_velocity.resize(_trajectory_length);
    for (int k=0; k<_trajectory_length; k++) {
        _ref_pose[k].resize(_n_st);
        _ref_velocity[k].resize(_n_in);
    }
    std::cout << "Loaded motion planning problem with " << n_obstacles() << " obstacles." << std::endl;
    return true;
}

bool MotionPlanningDynamicObstacle::updatePositionTrajectory() {
    // update motion planning algorithm
    p2pdynobst::Point2Point* problem = ((Problem<p2pdynobst::Point2Point>*)_problem)->get();
    bool check = problem->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, *(std::vector<p2pdynobst::obstacle_t>*)&_obstacles, _predict_shift);
    save(_ref_pose, _ref_velocity);
    return check;
}

ORO_LIST_COMPONENT_TYPE(MotionPlanningDynamicObstacle);
