// #define DEBUG

#include "FormationMotionPlanning-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <algorithm>
#include <stdlib.h>

using namespace std;

FormationMotionPlanning::FormationMotionPlanning(std::string const& name) : MotionPlanning(name),
_target_reached(false), _neighbors(2), _residuals(3), _rel_pos_c(2){
    for (int k=0; k<2; k++){
        ports()->addPort("x_var_port_" + to_string(k), _x_var_port[k]);
        ports()->addPort("zl_ij_var_port_" + to_string(k), _zl_ij_var_port[k]);
        ports()->addPort("x_j_var_port_" + to_string(k), _x_j_var_port[k]);
        ports()->addPort("zl_ji_var_port_" + to_string(k), _zl_ji_var_port[k]);
    }

    addProperty("host", _host).doc("Name of this host");
    addProperty("configuration", _configuration).doc("Pose relative to the fleet center");
    addProperty("init_admm_iter", _init_admm_iter).doc("Number of initial ADMM iterations");
    addProperty("rho", _rho).doc("Penalty factor for ADMM");

    addOperation("resetRobotos", &FormationMotionPlanning::resetRobots, this);
    addOperation("addRobot", &FormationMotionPlanning::addRobot, this);
    addOperation("getNeighbors", &FormationMotionPlanning::getNeighbors, this);
    addOperation("buildFormation", &FormationMotionPlanning::buildFormation, this);
}

void FormationMotionPlanning::setTargetPose(const std::vector<double>& target_pose) {
    MotionPlanning::setTargetPose(target_pose);
    for (int k=0; k<3; k++) {
        _target_pose[k] += _rel_pos_c[k];
    }
}

void FormationMotionPlanning::resetRobots(){
    _robots.resize(0);
}

void FormationMotionPlanning::addRobot(const std::string& name, const std::vector<double>& pose) {
    robot_t nghb;
    nghb.name = name;
    nghb.pose = pose;
    _robots.push_back(nghb);
}

std::vector<std::string> FormationMotionPlanning::getNeighbors() {
    std::vector<std::string> names(2);
    for (int k=0; k<2; k++) {
        names[k] = _neighbors[k].name;
    }
    return names;
}

std::vector<double> FormationMotionPlanning::buildFormation() {
    int n_robots = _robots.size();
    // determine formation center and robot permutation
    std::vector<double> center(2, 0);
    std::vector<int> indices(n_robots);
    for (int k=0; k<n_robots; k++) {
        indices[k] = k;
    }
    std::vector<int> formation_indices(n_robots);
    double residual = 10000000000;
    do {
        std::vector<double> center_tmp(3);
        for (int k=0; k<n_robots; k++) {
            center_tmp[0] += (1./n_robots)*(_robots[indices[k]].pose[0] - _configuration[3*k+0]);
            center_tmp[1] += (1./n_robots)*(_robots[indices[k]].pose[1] - _configuration[3*k+1]);
        }
        double res = 0;
        for (int k=0; k<n_robots; k++) {
            res += pow(_robots[indices[k]].pose[0] - center_tmp[0] - _configuration[3*k+0], 2);
            res += pow(_robots[indices[k]].pose[1] - center_tmp[1] - _configuration[3*k+1], 2);
        }
        if (res < residual) {
            residual = res;
            center = center_tmp;
            for (int k=0; k<n_robots; k++) {
                formation_indices[k] = indices[k];
            }
        }
    } while (std::next_permutation(indices.begin(), indices.end()));
    int host_index = 0;
    for (int k=0; k<n_robots; k++) {
        if (formation_indices[k] == 0) {
            host_index = k;
            break;
        }
    }
    _rel_pos_c.resize(2);
    _rel_pos_c[0] = _configuration[3*host_index+0];
    _rel_pos_c[1] = _configuration[3*host_index+1];
    // build neighbor vector
    _neighbors.resize(2);
    _neighbors[0] = _robots[formation_indices[(n_robots+host_index+1)%n_robots]];
    _neighbors[1] = _robots[formation_indices[(n_robots+host_index-1)%n_robots]];
    return center;
}

bool FormationMotionPlanning::config() {
    resetRobots();
    p2pf::Vehicle* vehicle = new p2pf::Holonomic();
    if (_ideal_prediction) {
        vehicle->setIdealPrediction(true);
    }
    _n_st = vehicle->getNState();
    _n_in = vehicle->getNInput();

    _problem = new ADMMProblem<p2pf::FormationPoint2Point>(new p2pf::FormationPoint2Point(vehicle, _update_time, _sample_time, _horizon_time, _trajectory_length, _init_admm_iter, _rho));
    _ref_pose.resize(_trajectory_length);
    _ref_velocity.resize(_trajectory_length);
    for(int k=0; k<_trajectory_length; k++) {
        _ref_pose[k].resize(_n_st);
        _ref_velocity[k].resize(_n_in);
    }
    _n_shared = ((ADMMProblem<p2pf::FormationPoint2Point>*)_problem)->n_shared();
    _x_var.resize(_n_shared);
    _x_j_var.resize(2);
    _z_ji_var.resize(2);
    _l_ji_var.resize(2);
    _z_ij_var.resize(2);
    _l_ij_var.resize(2);
    _zl_ij_p_var.resize(2);
    _zl_ji_p_var.resize(2);
    _x_p_var.resize(_n_shared+1);
    _x_j_p_var.resize(2);
    for(int k=0; k<2; k++) {
        _x_j_var[k].resize(_n_shared);
        _z_ji_var[k].resize(_n_shared);
        _l_ji_var[k].resize(_n_shared);
        _z_ij_var[k].resize(_n_shared);
        _l_ij_var[k].resize(_n_shared);
        _zl_ij_p_var[k].resize(2*_n_shared + 1);
        _zl_ji_p_var[k].resize(2*_n_shared + 1);
        _x_j_p_var[k].resize(_n_shared + 1);
    }
    _target_reached_nghb.resize(2);
    std::vector<double> example1(_n_shared + 1, 0.0);
    std::vector<double> example2(2*_n_shared + 1, 0.0);
    for (int k=0; k<2; k++) {
        _x_var_port[k].setDataSample(example1);
        _zl_ij_var_port[k].setDataSample(example2);
    }
    std::cout << "Loaded formation motion planning problem with " << n_obstacles() << " obstacles." << std::endl;
    return true;
}

bool FormationMotionPlanning::initialize() {
    emptyPorts();
    _problem->reset();
    _problem->resetTime();
    return true;
}

void FormationMotionPlanning::emptyPorts() {
    std::vector<double> dump(2*_n_shared + 1, 0.0);
    for (int k=0; k<2; k++) {
        _x_j_var_port[k].read(dump);
        _zl_ji_var_port[k].read(dump);
    }
}

double FormationMotionPlanning::encode(int index, bool valid) {
    std::cout << "encode with index " << index << std::endl;
    code_t code;
    code.index = index;
    code.target_reached = _target_reached;
    code.valid = valid;
    double ret;
    memcpy((char*)(&ret), (char*)(&code), sizeof(ret));
    return ret;
}

code_t FormationMotionPlanning::decode(double number) {
    code_t code;
    memcpy((char*)(&code), (char*)(&number), sizeof(code));
    std::cout << "code with index " << code.index << std::endl;
    return code;
}

bool FormationMotionPlanning::updatePositionTrajectory() {
    bool check;
    while (((ADMMProblem<p2pf::FormationPoint2Point>*)_problem)->iteration() < _init_admm_iter) {
        check = admmIteration(true);
        if (!check) {
            return false;
        }
    }
    return admmIteration(false);
}

bool FormationMotionPlanning::watchDog(bool initial, TimeService::ticks t0) {
    if (!initial && TimeService::Instance()->secondsSince(t0) > 2) {
        log(Error) << "Waiting for neighbor took more than 2s!" << endlog();
        return false;
    }
    if (TimeService::Instance()->secondsSince(t0) > 10) {
        log(Error) << "Waiting for neighbor took more than 10s!" << endlog();
        return false;
    }
    return true;
}

bool FormationMotionPlanning::admmIteration(bool initial) {
    ADMMProblem<p2pf::FormationPoint2Point>* problem = ((ADMMProblem<p2pf::FormationPoint2Point>*)_problem);
    std::cout << "admm iteration " << problem->iteration() << std::endl;
    #ifdef DEBUG
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // update1: determine x_var
    std::vector<double> rpc = {-_rel_pos_c[0], -_rel_pos_c[1]};
    bool check = problem->get()->update1(_est_pose, _target_pose, _ref_pose, _ref_velocity, _x_var, _z_ji_var, _l_ji_var, *(std::vector<p2pf::obstacle_t>*)&_obstacles, rpc, _predict_shift);
    #ifdef DEBUG
    Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "update1: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // add code and send this result to neighbors
    for (int i=0; i<_n_shared; i++) {
        _x_p_var[i] = _x_var[i];
    }
    _x_p_var[_n_shared] = encode(problem->iteration(), check);
    for (int k=0; k<2; k++) {
        _x_var_port[k].write(_x_p_var);
    }
    // in the meantime, save trajectories
    save(_ref_pose, _ref_velocity);
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "saving+sending1: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // receive from neighbors
    TimeService::ticks t0 = TimeService::Instance()->getTicks();
    for (int k=0; k<2; k++) {
        while ( _x_j_var_port[k].read(_x_j_p_var[k]) != RTT::NewData ) {
            if (!watchDog(initial, t0)) {
                error();
                return false;
            }
        }
    }
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "waiting1: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // check code and copy data
    bool check_nghb = true;
    int nghb_iteration, iteration;
    iteration = problem->iteration();
    if (!check) {
        iteration++;
    }
    for (int k=0; k<2; k++) {
        code_t code = decode(_x_j_p_var[k][_n_shared]);
        nghb_iteration = code.index;
        if (!code.valid) {
          check_nghb = false;
          nghb_iteration++;
        }
        if (nghb_iteration != iteration) {
            log(Error)<< "Index mismatch! (" << nghb_iteration << " vs " << iteration << ")" << endlog();
            error();
            return false;
        }
        _target_reached_nghb[k] = code.target_reached;
        for (int i=0; i<_n_shared; i++) {
            _x_j_var[k][i] = _x_j_p_var[k][i];
        }
    }
    // everyone follows cripple neighbor
    if (check && !check_nghb) {
        std::cout << "stepping back..." << std::endl;
        problem->stepBack();
        return false;
    }
    if (!check) {
        return false;
    }
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "checking1: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // update2: determine z_i_var, z_ij_var, l_i_var and l_ij_var
    problem->get()->update2(_x_j_var, _z_ij_var, _l_ij_var, _residuals);
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "update2: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // add code and send results to neighbors
    for (int k=0; k<2; k++) {
        for (int i=0; i<_n_shared; i++) {
            _zl_ij_p_var[k][i] = _z_ij_var[k][i];
            _zl_ij_p_var[k][i+_n_shared] = _l_ij_var[k][i];
        }
        _zl_ij_p_var[k][2*_n_shared] = encode(problem->iteration(), check);
        _zl_ij_var_port[k].write(_zl_ij_p_var[k]);
    }
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "extracting+sending2: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // receive from neighbors
    t0 = TimeService::Instance()->getTicks();
    for (int k=0; k<2; k++) {
        while ( _zl_ji_var_port[k].read(_zl_ji_p_var[k]) != RTT::NewData ) {
            if (!watchDog(initial, t0)) {
                error();
                return false;
            }
        }
    }
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "waiting2: " << time_elapsed << " s" << endl;
    _timestamp = TimeService::Instance()->getTicks();
    #endif
    // check code and copy data
    for (int k=0; k<2; k++) {
        for (int i=0; i<_n_shared; i++) {
            _z_ji_var[k][i] = _zl_ji_p_var[k][i];
            _l_ji_var[k][i] = _zl_ji_p_var[k][i+_n_shared];
        }
    }
    #ifdef DEBUG
    time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    cout << "checking2: " << time_elapsed << " s" << endl;
    #endif
    return check;
}

bool FormationMotionPlanning::targetReached() {
    bool target_reached = MotionPlanning::targetReached();
    if (!_target_reached) { // you should send it to your neighbor at least once
        _target_reached = target_reached;
        return false;
    }
    _target_reached = target_reached;
    for (int k=0; k<2; k++) {
        if (!(_target_reached && _target_reached_nghb[k])) {
            return false;
        }
    }
    return true;
}

ORO_LIST_COMPONENT_TYPE(FormationMotionPlanning);
