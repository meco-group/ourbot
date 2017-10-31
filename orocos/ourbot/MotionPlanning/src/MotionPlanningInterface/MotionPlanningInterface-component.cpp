#define DEBUG

#include "MotionPlanningInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

MotionPlanningInterface::MotionPlanningInterface(std::string const& name) : TaskContext(name, PreOperational),
    _mp_trigger_data(4), _est_pose(3), _target_pose(3),
    _ref_pose_trajectory(3), _ref_velocity_trajectory(3) {

    ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose wrt to initial frame");
    ports()->addEventPort("mp_trigger_port", _mp_trigger_port).doc("Trigger for motion planning: is composed of current estimated pose and start index of internal reference input vector");

    ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
    ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
    ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");

    ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
    ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
    ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

    ports()->addPort("motion_time_port", _motion_time_port).doc("Motion time of computed motion trajectory");

    addProperty("max_failures", _max_failures).doc("Maximum number of consecutive infeasible motionplanning updates");
    addProperty("max_recovers", _max_recovers).doc("Maximum number of consecutive motionplanning recoveries");
    addProperty("max_periods", _max_periods).doc("Maximum number of periods the motionplanning can take to compute");
    addProperty("control_rate", _control_rate).doc("Frequency to update the control loop");
    addProperty("motionplanning_rate", _motionplanning_rate).doc("Frequency to update the motionplanning");
    addProperty("horizon_time", _horizon_time).doc("Horizon to compute motion trajectory");
    addProperty("max_vel_position", _max_vel_position).doc("Maximum position velocity [m/s]");
    addProperty("max_vel_orientation", _max_vel_orientation).doc("Maximum orientation velocity [rad/s]");
    addProperty("max_acc_orientation", _max_acc_orientation).doc("Maximum orientation acceleration [rad/s^2]");
    addProperty("target_dist_tol", _target_dist_tol).doc("Tolerance for target position detection");
    addProperty("angle_dist_tol", _angle_dist_tol).doc("Tolerance for target orientation detection");
    addProperty("input_norm_tol", _input_norm_tol).doc("Tolerance for target input detection");
    addProperty("orientation_th", _orientation_th).doc("Threshold for orientation setpoint");

    addOperation("setTargetPose", &MotionPlanningInterface::setTargetPose, this).doc("Set target pose");
    addOperation("reset", &MotionPlanningInterface::reset, this).doc("Should be called before each new p2p task");
    addOperation("ready", &MotionPlanningInterface::ready, this).doc("We reached the target, p2p task ready");
    addOperation("busy", &MotionPlanningInterface::busy, this).doc("Busy with computing a trajectory");
    addOperation("valid", &MotionPlanningInterface::valid, this).doc("Last computed trajectory is valid (i.e. not infeasible)");
    addOperation("resetObstacles", &MotionPlanningInterface::resetObstacles, this).doc("Reset obstacle list");
    addOperation("addStaticRectObstacle", &MotionPlanningInterface::addStaticRectObstacle, this).doc("Add static rectangular obstacle to obstacle list");
    addOperation("addStaticCircObstacle", &MotionPlanningInterface::addStaticCircObstacle, this).doc("Add static circular obstacle to obstacle list");
    addOperation("addDynamicRectObstacle", &MotionPlanningInterface::addDynamicRectObstacle, this).doc("Add dynamic constant velocity rectangular obstacle to obstacle list");
    addOperation("addDynamicCircObstacle", &MotionPlanningInterface::addDynamicCircObstacle, this).doc("Add dynamic constant velocity circular obstacle to obstacle list");
    addOperation("addPeerRectObstacle", &MotionPlanningInterface::addPeerRectObstacle, this).doc("Add peer trajectory as rectangular obstacle to obstacle list");
    addOperation("addPeerCircObstacle", &MotionPlanningInterface::addPeerCircObstacle, this).doc("Add peer trajectory as circular obstacle to obstacle list");
    addOperation("getMotionTime", &MotionPlanningInterface::getMotionTime, this).doc("Get last motion time update");
}

void MotionPlanningInterface::setTargetPose(const std::vector<double>& target_pose) {
    _target_pose = target_pose;
    std::vector<double> est_pose;
    _est_pose_port.read(est_pose);

    _target_pose[2] += floor(est_pose[2]/(2*M_PI))*2*M_PI;
    if ((_target_pose[2] - est_pose[2]) > M_PI) {
        _target_pose[2] -= 2*M_PI;
    }
    if ((_target_pose[2] - est_pose[2]) < -M_PI) {
        _target_pose[2] += 2*M_PI;
    }
    std::cout << "target set: (" << _target_pose[0] << "," << _target_pose[1] << "," << _target_pose[2] << ")" << std::endl;
    _target_set = true;
    _ready = false;
}

bool MotionPlanningInterface::reset() { // call this one before each p2p task
    if (!initialize()) {
        log(Error) << "Error occured in initialize()!" << endlog();
        return false;
    }
    _target_set = false;
    _ready = false;
    _first_iteration = true;
    std::cout << "reset motionplanning" << std::endl;
    return true;
}


bool MotionPlanningInterface::ready() {
    return _ready;
}

bool MotionPlanningInterface::busy() {
    return _busy;
}

bool MotionPlanningInterface::valid() {
    return _valid;
}

bool MotionPlanningInterface::configureHook() {
    _update_time = 1./_motionplanning_rate;
    _sample_time = 1./_control_rate;
    _update_length = static_cast<int>(_control_rate/_motionplanning_rate);
    _trajectory_length = static_cast<int>((_horizon_time - _update_time)/_sample_time);
    // reserve required memory and initialize with zeros
    for(int i=0;i<3;i++) {
        _ref_pose_trajectory[i].resize(_trajectory_length);
        _ref_velocity_trajectory[i].resize(_trajectory_length);
    }
    // show example data sample to ports to make data flow real-time
    std::vector<double> example(_trajectory_length, 0.0);
    for(int i=0; i<3; i++) {
        _ref_pose_trajectory_port[i].setDataSample(example);
        _ref_velocity_trajectory_port[i].setDataSample(example);
    }
    if (!config()) {
        log(Error) << "Error occured in configure() !" << endlog();
        return false;
    }
    reset();
    return true;
}

void MotionPlanningInterface::updateHook() {
    _busy = true;
    _valid = false;
    DEBUG_PRINT("started mp update")
    _timestamp = TimeService::Instance()->getTicks();
    // read trigger data
    _mp_trigger_port.read(_mp_trigger_data);
    for (int k=0; k<3; k++) {
        _est_pose[k] = _mp_trigger_data[k];
    }
    _predict_shift = static_cast<int>(_mp_trigger_data[3]);
    // read target pose
    if (!_target_set) {
        log(Error) << "No target set!" << endlog();
        _busy = false;
        DEBUG_PRINT("stopped mp update");
        return;
    }
    // target-reached detection
    if (targetReached()) {
        _ready = true;
        _valid = true;
        _busy = false;
        DEBUG_PRINT("stopped mp update");
        return;
    }
    // set obstacle data
    loadObstacles();
    // update trajectory
    _valid = updatePoseTrajectory();
    _first_iteration = false;
    if (_valid) {
        // write trajectory
        for (int i=0; i<3; i++) {
            _ref_pose_trajectory_port[i].write(_ref_pose_trajectory[i]);
            _ref_velocity_trajectory_port[i].write(_ref_velocity_trajectory[i]);
        }
        // write motion time
        _motion_time_port.write(getMotionTime());
    } else {
        patchup();
    }
    _busy = false;
    Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
    DEBUG_PRINT("ended mp update in " << time_elapsed << " s")
}

bool MotionPlanningInterface::updatePoseTrajectory() {
    bool valid = updatePositionTrajectory();
    _rotation_time = updateOrientationTrajectory();
    return valid;
}

bool MotionPlanningInterface::targetReached() {
    double target_dist = 0.;
    double input_norm = 0.;
    for (int i=0; i<2; i++) {
        target_dist += pow(_target_pose[i] - _est_pose[i], 2);
        input_norm += pow(_ref_velocity_trajectory[i][_predict_shift], 2);
    }
    target_dist = sqrt(target_dist);
    input_norm = sqrt(input_norm);
    double angle_dist = fabs(_target_pose[2] - _est_pose[2]);
    if (target_dist < _target_dist_tol && input_norm < _input_norm_tol && angle_dist < _angle_dist_tol){
        return true;
    }
    return false;
}

void MotionPlanningInterface::patchup() {
    // this method patchups the motionplanning when an invalid trajectory was encountered.
    // this one should be called when you don't want to interrupt the motion.
}

double MotionPlanningInterface::updateOrientationTrajectory() {
    // trapezium velocity trajectory
    if (_first_iteration) {
        _increase = true;
    }
    double omega = _max_vel_orientation;
    double alpha = _max_acc_orientation;
    // get extrapolated theta0 and omega0
    double th0 = _est_pose[2];
    double om0 = _ref_velocity_trajectory[2][_predict_shift + static_cast<int>(_update_time/_sample_time)];
    for (int k=0; k<int(_update_time/_sample_time); k++) {
        th0 += _ref_velocity_trajectory[2][k+_predict_shift]*_sample_time;
    }
    double dth = _target_pose[2] - th0;
    if (dth < 0) {
        omega = -omega;
        alpha = -alpha;
    }
    double t_rise, t_tot, t_dec;
    if (_increase) {
        double Dt = omega/alpha;
        double dt = om0/alpha;
    if ((Dt*fabs(omega) - 0.5*dt*fabs(om0)) < fabs(dth)) {
        // trapezium
        t_rise = Dt-dt;
        t_tot = Dt + (0.5*fabs(om0/omega) -1)*dt + dth/omega;
        t_dec = t_tot-Dt;
        } else {
        // triangle
        omega = sqrt(alpha*dth+0.5*pow(om0, 2))*omega/fabs(omega);
        t_rise = omega/alpha-dt;
        t_tot = 2*t_rise-dt;
        t_dec = t_rise;
        }
    } else {
        t_dec = 0;
        t_tot = pow(om0, 2)/alpha;
    }
    int it_dec = static_cast<int>(t_dec*_control_rate);
    double th_ref = th0;
    double om_ref = om0;
    bool increase = _increase;

    if (fabs(dth) <= _orientation_th) {
        t_tot = 0.;
    }
    for (uint k=0; k<_ref_pose_trajectory[2].size(); k++) {
        if (fabs(th_ref-_target_pose[2]) <= _orientation_th) { // when arrived, stay there
            om_ref = 0;
            th_ref = _target_pose[2];
        } else {
            if (increase) {
                if (k >= it_dec) {
                    om_ref -= alpha*_sample_time;
                    increase = false;
                    if (k <= int(_update_time/_sample_time)) {
                        _increase = false;
                    }
                } else {
                    if (fabs(om_ref) >= fabs(omega)) {
                        om_ref = omega;
                    } else {
                        om_ref += alpha*_sample_time;
                    }
                }
            } else {
                om_ref -= alpha*_sample_time;
            }
            if (om_ref*omega < 0) {
                om_ref = 0;
                th_ref = _target_pose[2];
            } else {
                th_ref += om_ref*_sample_time;
            }
        }
        _ref_pose_trajectory[2][k] = th_ref;
        _ref_velocity_trajectory[2][k] = om_ref;
    }
    return t_tot;
}

void MotionPlanningInterface::resetObstacles() {
    _obstacles.clear();
}

void MotionPlanningInterface::addStaticRectObstacle(const std::vector<double>& pose, const std::vector<double>& size) {
    obstacle_t obstacle;
    obstacle.avoid = true;
    obstacle.position = std::vector<double>({pose[0], pose[1]});
    obstacle.velocity = std::vector<double>(2, 0);
    obstacle.acceleration = std::vector<double>(2, 0);
    double width = size[0];
    double height = size[1];
    obstacle.radii = std::vector<double>(4);
    obstacle.checkpoints = std::vector<double>(8);
    for (int i=0; i<4; i++) {
        obstacle.radii[i] = 1e-3;
        obstacle.checkpoints[0] = 0.5*width*cos(pose[2]) - 0.5*height*sin(pose[2]);
        obstacle.checkpoints[1] = 0.5*width*sin(pose[2]) + 0.5*height*cos(pose[2]);
        obstacle.checkpoints[2] = 0.5*width*cos(pose[2]) + 0.5*height*sin(pose[2]);
        obstacle.checkpoints[3] = 0.5*width*sin(pose[2]) - 0.5*height*cos(pose[2]);
        obstacle.checkpoints[4] = -0.5*width*cos(pose[2]) + 0.5*height*sin(pose[2]);
        obstacle.checkpoints[5] = -0.5*width*sin(pose[2]) - 0.5*height*cos(pose[2]);
        obstacle.checkpoints[6] = -0.5*width*cos(pose[2]) - 0.5*height*sin(pose[2]);
        obstacle.checkpoints[7] = -0.5*width*sin(pose[2]) + 0.5*height*cos(pose[2]);
    }
    _obstacles.push_back(obstacle);
}

void MotionPlanningInterface::addStaticCircObstacle(const std::vector<double>& pose, double radius) {
    obstacle_t obstacle;
    obstacle.avoid = true;
    obstacle.position = std::vector<double>({pose[0], pose[1]});
    obstacle.velocity = std::vector<double>(2, 0);
    obstacle.acceleration = std::vector<double>(2, 0);
    obstacle.radii = std::vector<double>({radius});
    obstacle.checkpoints = std::vector<double>({0., 0.});
    _obstacles.push_back(obstacle);
}


void MotionPlanningInterface::addDynamicRectObstacle(const std::vector<double>& pose, const std::vector<double>& velocity, const std::vector<double>& size) {
    addStaticRectObstacle(pose, size);
    int ind = _obstacles.size()-1;
    _obstacles[ind].velocity[0] = velocity[0];
    _obstacles[ind].velocity[1] = velocity[1];
}

void MotionPlanningInterface::addDynamicCircObstacle(const std::vector<double>& pose, const std::vector<double>& velocity, double radius) {
    addStaticCircObstacle(pose, radius);
    int ind = _obstacles.size()-1;
    _obstacles[ind].velocity[0] = velocity[0];
    _obstacles[ind].velocity[1] = velocity[1];
}

void MotionPlanningInterface::addPeerRectObstacle(const std::vector<double>& coeff_vector, const std::vector<double>& size) {
    obstacle_t obstacle;
    obstacle.avoid = true;
    obstacle.traj_coeffs = coeff_vector;

    double width = size[0];
    double height = size[1];
    obstacle.radii = std::vector<double>(4);
    obstacle.checkpoints = std::vector<double>(8);
    for (int i=0; i<4; i++) {
        obstacle.radii[i] = 1e-3;
        obstacle.checkpoints[0] = 0.5*width*cos(0.) - 0.5*height*sin(0.);
        obstacle.checkpoints[1] = 0.5*width*sin(0.) + 0.5*height*cos(0.);
        obstacle.checkpoints[2] = 0.5*width*cos(0.) + 0.5*height*sin(0.);
        obstacle.checkpoints[3] = 0.5*width*sin(0.) - 0.5*height*cos(0.);
        obstacle.checkpoints[4] = -0.5*width*cos(0.) + 0.5*height*sin(0.);
        obstacle.checkpoints[5] = -0.5*width*sin(0.) - 0.5*height*cos(0.);
        obstacle.checkpoints[6] = -0.5*width*cos(0.) - 0.5*height*sin(0.);
        obstacle.checkpoints[7] = -0.5*width*sin(0.) + 0.5*height*cos(0.);
    }
    _obstacles.push_back(obstacle);
}

void MotionPlanningInterface::addPeerCircObstacle(const std::vector<double>& coeff_vector, double radius) {
    obstacle_t obstacle;
    obstacle.avoid = true;
    obstacle.traj_coeffs = coeff_vector;

    obstacle.radii = std::vector<double>({radius});
    obstacle.checkpoints = std::vector<double>({0., 0.});
    _obstacles.push_back(obstacle);
}

void MotionPlanningInterface::loadObstacles() {
    int obst_size = _obstacles.size();
    if (obst_size > n_obstacles()) {
        log(Warning) << "Motion Planning got too much obstacles (" << _obstacles.size() << "), only accounting for first " << n_obstacles() << "!" << endlog();
        _obstacles.resize(n_obstacles());
        return;
    }
    for (int k=obst_size; k<n_obstacles(); k++) {
        // fill with dummy data -> far away
        obstacle_t obstacle;
        obstacle.avoid = true;
        obstacle.position = std::vector<double>({-1000, -1000});
        obstacle.velocity = std::vector<double>(2, 0);
        obstacle.acceleration = std::vector<double>(2, 0);
        obstacle.radii = std::vector<double>(4);
        obstacle.checkpoints = std::vector<double>(8);
        for (int i=0; i<4; i++) {
            obstacle.radii[i] = 1.e-3;
            for (int j=0; j<2; j++) {
                obstacle.checkpoints[2*i+j] = 0.;
            }
        }
        _obstacles.push_back(obstacle);
    }
}

ORO_CREATE_COMPONENT_LIBRARY()

