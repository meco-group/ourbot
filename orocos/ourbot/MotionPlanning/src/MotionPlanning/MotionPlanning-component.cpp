#include "MotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

MotionPlanning::MotionPlanning(std::string const& name) : MotionPlanningInterface(name) {
    addProperty("ideal_prediction", _ideal_prediction).doc("Use prediction based on computed trajectories and not on state estimation");
    addOperation("getCoefficients", &MotionPlanning::getCoefficients, this).doc("Get spline coefficients of last trajectory");
    addOperation("getBasisLength", &MotionPlanning::getBasisLength, this).doc("Get length of basis of spline trajectory");
}

bool MotionPlanning::config() {
    p2p::Vehicle* vehicle = new p2p::Holonomic();
    if (_ideal_prediction) {
        vehicle->setIdealPrediction(true);
    }
    _n_st = vehicle->getNState();
    _n_in = vehicle->getNInput();
    _problem = new Problem<p2p::Point2Point>(new p2p::Point2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length));

    _ref_pose.resize(_trajectory_length);
    _ref_velocity.resize(_trajectory_length);
    for(int k=0; k<_trajectory_length; k++) {
        _ref_pose[k].resize(_n_st);
        _ref_velocity[k].resize(_n_in);
    }
    std::cout << "Loaded motion planning problem with " << n_obstacles() << " obstacles." << std::endl;
    return true;
}

bool MotionPlanning::initialize() {
    _problem->reset();
    _problem->resetTime();
    return true;
}

void MotionPlanning::patchup() {
    _problem->recover();
}

bool MotionPlanning::updatePositionTrajectory() {
    // update motion planning algorithm
    p2p::Point2Point* problem = ((Problem<p2p::Point2Point>*)_problem)->get();
    bool check = problem->update(_est_pose, _target_pose, _ref_pose, _ref_velocity, *(std::vector<p2p::obstacle_t>*)&_obstacles, _predict_shift);
    save(_ref_pose, _ref_velocity);
    return check;
}

void MotionPlanning::save(const std::vector<std::vector<double> >& ref_pose, const std::vector<std::vector<double> >& ref_velocity) {
    for (int k=0; k<_trajectory_length; k++) {
        for (int j=0; j<_n_st; j++) {
            _ref_pose_trajectory[j][k] = ref_pose[k][j];
        }
        for (int j=0; j<_n_in; j++) {
            _ref_velocity_trajectory[j][k] = ref_velocity[k][j];
        }
    }
}

double MotionPlanning::getMotionTime() {
    double position_time;
    std::vector<double> coeff_vector = getCoefficients();
    uint n_cfs = coeff_vector.size()/2;
    double target_dist = sqrt(pow(_target_pose[0] - coeff_vector[n_cfs-1], 2) + pow(_target_pose[1] - coeff_vector[2*n_cfs-1], 2));
    if (target_dist > _target_dist_tol) { // end of trajectory is not on destination yet -> use heuristic
        double v_mean = 0.87*_max_vel_position;
        double dist = sqrt(pow(_target_pose[0]-coeff_vector[0], 2) + pow(_target_pose[1]-coeff_vector[n_cfs], 2));
        position_time = dist/v_mean;
    } else {
        int k;
        for (k=0; k<_trajectory_length; k++) {
            if(sqrt(pow(_ref_pose_trajectory[0][k]-_target_pose[0], 2) + pow(_ref_pose_trajectory[1][k]-_target_pose[1], 2)) <= _target_dist_tol) {
                break;
            }
        }
        position_time = k*_sample_time;
    }
    return std::max(position_time, _rotation_time);
}

std::vector<double> MotionPlanning::getCoefficients() {
    std::vector<double> coeffs;
    _problem->getCoefficients(coeffs);
    return coeffs;
}

int MotionPlanning::getBasisLength() {
    return _problem->getLenBasis();
}

ORO_LIST_COMPONENT_TYPE(MotionPlanning);
