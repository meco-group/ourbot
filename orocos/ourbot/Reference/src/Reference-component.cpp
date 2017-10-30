#include "Reference-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>

Reference::Reference(std::string const& name) : TaskContext(name, PreOperational),
    _ref_pose(3), _ref_velocity(3), _pose_offset(3) {

    ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
    ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
    ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");
    ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
    ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
    ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

    ports()->addPort("ref_position_trajectory_x_port", _ref_position_trajectory_port[0]).doc("x reference trajectory for plotting");
    ports()->addPort("ref_position_trajectory_y_port", _ref_position_trajectory_port[1]).doc("y reference trajectory for plotting");

    ports()->addPort("ref_pose_port", _ref_pose_port).doc("Pose reference sample");
    ports()->addPort("ref_velocity_port", _ref_velocity_port).doc("Velocity reference sample");

    addProperty("repeat_trajectory", _repeat_trajectory).doc("Repeat trajectory when end is reached");
    addProperty("n_samples_plot", _n_samples_plot).doc("Number of samples to plot trajectory");
    addProperty("trajectory_path", _trajectory_path).doc("Path to csv file with trajectory to follow");

    addOperation("reset", &Reference::reset, this).doc("Reset index");
    addOperation("ready", &Reference::ready, this).doc("Complete trajectory followed?");
    addOperation("setPoseOffset", &Reference::setPoseOffset, this).doc("Set an offset to the reference pose (i.e. to start from current position)");
    addOperation("receiveTrajectory", &Reference::receiveTrajectory, this).doc("Receive reference trajectory from input port");
    addOperation("loadTrajectory", &Reference::loadTrajectory, this).doc("Load trajectory from csv file");
}

bool Reference::configureHook() {
    // show example data sample to ports to make data flow real-time
    std::vector<double> example(3, 0.0);
    _ref_pose_port.setDataSample(example);
    _ref_velocity_port.setDataSample(example);
    return true;
}

bool Reference::startHook() {
    // check connections
    for(int i=0; i<3; i++) {
        _con_ref_pose_trajectory[i] = _ref_pose_trajectory_port[i].connected();
        _con_ref_velocity_trajectory[i] = _ref_velocity_trajectory_port[i].connected();
    }
    reset();
    return true;
}

void Reference::updateHook() {
    // check index overflow
    if (_index >= _ref_pose_trajectory[0].size()) {
        if (_repeat_trajectory) {
            _index = 0;
        } else {
            _index--;
            _ready = true;
        }
    }
    // send sample to controller
    for (int i=0; i<3; i++) {
        if (fabs(_ref_pose_trajectory[i].at(_index)) > 1.e-3) {
          _ref_pose[i] = _ref_pose_trajectory[i].at(_index) + _pose_offset[i];
        } else {
          _ref_pose[i] = 0.0 + _pose_offset[i];
        }
        if (fabs(_ref_velocity_trajectory[i].at(_index)) > 1.e-3) {
          _ref_velocity[i] = _ref_velocity_trajectory[i].at(_index);
        } else {
          _ref_velocity[i] = 0.0;
        }
    }
    // std::cout << "[pose] (" << _ref_pose[0] << "," << _ref_pose[1] << "," << _ref_pose[2] << ")" << std::endl;
    // std::cout << "[velocity] (" << _ref_velocity[0] << "," << _ref_velocity[1] << "," << _ref_velocity[2] << ")" << std::endl;
    _ref_pose_port.write(_ref_pose);
    _ref_velocity_port.write(_ref_velocity);
    // update index
    _index++;
}

void Reference::reset() {
    _index = 0;
    for (int i=0; i<3; i++) {
        _ref_velocity[i] = 0.0;
    }
    _ref_velocity_port.write(_ref_velocity);
    _ready = false;
}

bool Reference::ready() {
    return _ready;
}

void Reference::setPoseOffset(double x, double y, double t) {
    _pose_offset = std::vector<double>({x, y, t});
    updatePositionTrajectory();
}

bool Reference::receiveTrajectory(int index) {
    bool new_data = true;
    for(int i=0; i<3; i++) {
        std::vector<double> trajectory;
        if ( _con_ref_pose_trajectory[i] ) {
            if (_ref_pose_trajectory_port[i].read(trajectory) == RTT::NewData) {
                _ref_pose_trajectory[i] = trajectory;
                new_data &= true;
            } else {
                new_data &= false;
            }
        }
        if ( _con_ref_velocity_trajectory[i] ) {
            if (_ref_velocity_trajectory_port[i].read(trajectory) == RTT::NewData) {
                _ref_velocity_trajectory[i] = trajectory;
                new_data &= true;
            } else {
                new_data &= false;
            }
        }
    }
    if (!new_data) {
        return false;
    }
    _index = index;
    updatePositionTrajectory();
    return true;
}

bool Reference::loadTrajectory(const std::string& path) {
    std::vector<double> ref_pose_trajectory[3];
    std::vector<double> ref_velocity_trajectory[3];
    std::ifstream file;
    file.open(path.c_str());
    std::string line, str;
    double value;
    if (!file.good()) {
        return false;
    }
    while (file.good()) {
        std::getline(file, line);
        std::stringstream iss(line);
        for (int i=0; i<6; i++) {
            std::getline(iss, str, ',');
            std::stringstream convertor(str);
            convertor >> value;
            if (i < 3) {
                ref_pose_trajectory[i].push_back(value);
            } else {
                ref_velocity_trajectory[i-3].push_back(value);
            }
        }
    }
    file.close();
    for (int i=0; i<3; i++) {
        ref_pose_trajectory[i].pop_back(); // last value is rubbish
        ref_velocity_trajectory[i].pop_back();
        _ref_pose_trajectory[i].resize(ref_pose_trajectory[i].size());
        _ref_velocity_trajectory[i].resize(ref_velocity_trajectory[i].size());
        _ref_pose_trajectory[i] = ref_pose_trajectory[i];
        _ref_velocity_trajectory[i] = ref_velocity_trajectory[i];
    }
    reset();
    updatePositionTrajectory();
    return true;
}

void Reference::updatePositionTrajectory() {
    int step = (_ref_pose_trajectory[0].size()-1)/(_n_samples_plot-1);
    for (int i=0; i<2; i++) {
        _ref_position_trajectory[i].resize(_n_samples_plot);
        for (int k=0; k<_n_samples_plot; k++) {
            _ref_position_trajectory[i][k] = _ref_pose_trajectory[i][k*step] + _pose_offset[i];
        }
        _ref_position_trajectory_port[i].write(_ref_position_trajectory[i]);
    }
}

ORO_CREATE_COMPONENT_LIBRARY();
ORO_LIST_COMPONENT_TYPE(Reference);
