#ifndef OROCOS_POSECONTROLLER_COMPONENT_HPP
#define OROCOS_POSECONTROLLER_COMPONENT_HPP

#include "../ControllerInterface/ControllerInterface-component.hpp"

class PoseController : public ControllerInterface {

  private:
    bool _enable_fb;
    bool _enable_ff;
    bool _correct_orientation;
    std::vector<double> _kp;
    std::vector<double> _ki;
    double _Ts;

    std::vector<double> _cmd_vel;
    std::vector<double> _cmd_vel_prev;
    std::vector<double> _est_pose;
    std::vector<double> _ref_pose;
    std::vector<double> _error_gl;
    std::vector<double> _error_loc;
    std::vector<double> _error_loc_prev;
    std::vector<double> _ref_vel_gl;
    std::vector<double> _ref_vel_loc;

    std::vector<double> global2local(const std::vector<double>& state_gl, double rot);

  public:
    PoseController(std::string const& name);
    bool controlHook();
    bool initialize();

};

#endif
