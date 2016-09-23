#include "PoseController-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

PoseController::PoseController(std::string const& name) : ControllerInterface(name),
    _cmd_vel(3), _est_pose(3), _ref_pose(3), _error_gl(3), _error_loc(3),
    _ref_vel_gl(3), _ref_vel_loc(3){
    addProperty("enable_fb", _enable_fb).doc("Enable feedback action");
    addProperty("enable_ff", _enable_ff).doc("Enable feedforward action");
    addProperty("state_fb_par", _state_fb_par).doc("State feedback parameters: kx, ky, kt");
}

bool PoseController::initialize(){
    return true;
}

bool PoseController::controlUpdate(){
    for (int k=0; k<3; k++){
        _cmd_vel[k] = 0.;
    }
    _est_pose = getEstPose();
    // feedback action
    if (_enable_fb){
        _ref_pose = getRefPose();
        for (int k=0; k<3; k++){
            _error_gl[k] = _ref_pose[k] - _est_pose[k];
        }
        _error_loc = global2local(_error_gl, _est_pose[2]);
        for (int k=0; k<3; k++){
            _cmd_vel[k] = _state_fb_par[k]*_error_loc[k];
        }
    }
    // feedforward action
    if (_enable_ff){
        _ref_vel_gl = getRefVelocity();
        _ref_vel_loc = global2local(_ref_vel_gl, _est_pose[2]);
        for (int k=0; k<3; k++){
            _cmd_vel[k] += _ref_vel_loc[k];
        }
    }
    setCmdVelocity(_cmd_vel);
    return true;
}

std::vector<double> PoseController::global2local(const std::vector<double>& state_gl, double rot){
    std::vector<double> state_loc(3);
    double ct = cos(rot);
    double st = sin(rot);
    state_loc[0] = state_gl[0]*ct + state_gl[1]*st;
    state_loc[1] = state_gl[0]*st - state_gl[1]*ct;
    state_loc[2] = state_gl[2];
    return state_loc;
}

ORO_LIST_COMPONENT_TYPE(PoseController);
