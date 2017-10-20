#define DEBUG

#include "MotionPlanningInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

MotionPlanningInterface::MotionPlanningInterface(std::string const& name) : TaskContext(name, PreOperational),
    _mp_trigger_data(4), _est_pose(3), _target_pose(3),
    _ref_pose_trajectory(3), _ref_velocity_trajectory(3) {

    ports()->addPort("obstacle_port", _obstacle_port).doc("Detected obstacles");
    ports()->addPort("est_pose_port", _est_pose_port).doc("Estimated pose wrt to initial frame");
    ports()->addEventPort("mp_trigger_port", _mp_trigger_port).doc("Trigger for motion planning: is composed of current estimated pose and start index of internal reference input vector");
    ports()->addPort("robobs_pose_port", _robobs_pose_port).doc("Pose information of a robot obstacle");
    ports()->addPort("robobs_velocity_port", _robobs_velocity_port).doc("Velocity information of a robot obstacle");

    ports()->addPort("ref_pose_trajectory_x_port", _ref_pose_trajectory_port[0]).doc("x reference trajectory");
    ports()->addPort("ref_pose_trajectory_y_port", _ref_pose_trajectory_port[1]).doc("y reference trajectory");
    ports()->addPort("ref_pose_trajectory_t_port", _ref_pose_trajectory_port[2]).doc("theta reference trajectory");

    ports()->addPort("ref_velocity_trajectory_x_port", _ref_velocity_trajectory_port[0]).doc("x velocity reference trajectory");
    ports()->addPort("ref_velocity_trajectory_y_port", _ref_velocity_trajectory_port[1]).doc("y velocity reference trajectory");
    ports()->addPort("ref_velocity_trajectory_t_port", _ref_velocity_trajectory_port[2]).doc("theta velocity reference trajectory");

    ports()->addPort("motion_time_port", _motion_time_port).doc("Motion time of computed motion trajectory");

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
    std::cout << "re-init motionplanning" << std::endl;
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
    // init obstacles
    initObstacles(_obstacles, n_obstacles());
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
        return;
    }
    // target-reached detection
    if (targetReached()) {
        _ready = true;
        _valid = true;
        _busy = false;
        return;
    }
    // set obstacle data
    fillObstacles(_obstacles);
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
    }
    for (int i=0; i<3; i++) {
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

void MotionPlanningInterface::initObstacles(std::vector<obstacle_t>& obstacles, int n_obs) {
    obstacles.resize(n_obs);
    for (int k=0; k<n_obs; k++){
        obstacles[k].position.resize(2, 0.0);
        obstacles[k].velocity.resize(2, 0.0);
        obstacles[k].acceleration.resize(2, 0.0);
        obstacles[k].checkpoints.resize(2*4, 0.0);
        obstacles[k].radii.resize(4, 0.0);
    }
}

void MotionPlanningInterface::fillObstacles(std::vector<obstacle_t>& obstacles) {
    std::vector<double> obstacle_data;
    _obstacle_port.read(obstacle_data);
    double orientation, width, height;
    for (int k=0; k<obstacles.size(); k++) {
        if (obstacle_data[5*k] == -100) {
            obstacles[k].avoid = false;
        } else {
            obstacles[k].avoid = true;
            obstacles[k].position[0] = obstacle_data[5*k+0];
            obstacles[k].position[1] = obstacle_data[5*k+1];
            obstacles[k].velocity[0] = 0.;
            obstacles[k].velocity[1] = 0.;
            obstacles[k].acceleration[0] = 0.;
            obstacles[k].acceleration[1] = 0.;
            // set checkpoints
            if (obstacle_data[5*k+3] == -100) {
                // circle
                for (int i=0; i<4; i++) {
                    obstacles[k].radii[i] = obstacle_data[5*k+2];
                    for (int j=0; j<2; j++) {
                        obstacles[k].checkpoints[2*i+j] = obstacle_data[5*k+j];
                    }
                }
            } else {
                // rectangle
                orientation = (M_PI/180.)*obstacle_data[5*k+2];
                width = obstacle_data[5*k+3];
                height = obstacle_data[5*k+4];
                for (int i=0; i<4; i++) {
                    obstacles[k].radii[i] = 0.001;
                }
                obstacles[k].checkpoints[0] = 0.5*width*cos(orientation) - 0.5*height*sin(orientation);
                obstacles[k].checkpoints[1] = 0.5*width*sin(orientation) + 0.5*height*cos(orientation);
                obstacles[k].checkpoints[2] = 0.5*width*cos(orientation) + 0.5*height*sin(orientation);
                obstacles[k].checkpoints[3] = 0.5*width*sin(orientation) - 0.5*height*cos(orientation);
                obstacles[k].checkpoints[4] = -0.5*width*cos(orientation) + 0.5*height*sin(orientation);
                obstacles[k].checkpoints[5] = -0.5*width*sin(orientation) - 0.5*height*cos(orientation);
                obstacles[k].checkpoints[6] = -0.5*width*cos(orientation) - 0.5*height*sin(orientation);
                obstacles[k].checkpoints[7] = -0.5*width*sin(orientation) + 0.5*height*cos(orientation);
            }
        }
    }
    std::vector<double> obst_pose;
    std::vector<double> obst_velocity;
    if (_robobs_pose_port.read(obst_pose) == RTT::NewData) {
        int index = obstacles.size()-1;
        obstacles[index].avoid = true;
        obstacles[index].position[0] = obst_pose[0];
        obstacles[index].position[1] = obst_pose[1];
        orientation = obst_pose[2];
        if (_robobs_velocity_port.read(obst_velocity) == RTT::NewData) {
            obstacles[index].velocity[0] = obst_velocity[0];
            obstacles[index].velocity[1] = obst_velocity[1];
        } else {
            obstacles[index].velocity[0] = 0.;
            obstacles[index].velocity[1] = 0.;
        }
        obstacles[index].acceleration[0] = 0.;
        obstacles[index].acceleration[1] = 0.;
        width = 0.55;
        height = 0.4;
        for (int i=0; i<4; i++) {
            obstacles[index].radii[i] = 0.001;
        }
        obstacles[index].checkpoints[0] = 0.5*width*cos(orientation) - 0.5*height*sin(orientation);
        obstacles[index].checkpoints[1] = 0.5*width*sin(orientation) + 0.5*height*cos(orientation);
        obstacles[index].checkpoints[2] = 0.5*width*cos(orientation) + 0.5*height*sin(orientation);
        obstacles[index].checkpoints[3] = 0.5*width*sin(orientation) - 0.5*height*cos(orientation);
        obstacles[index].checkpoints[4] = -0.5*width*cos(orientation) + 0.5*height*sin(orientation);
        obstacles[index].checkpoints[5] = -0.5*width*sin(orientation) - 0.5*height*cos(orientation);
        obstacles[index].checkpoints[6] = -0.5*width*cos(orientation) - 0.5*height*sin(orientation);
        obstacles[index].checkpoints[7] = -0.5*width*sin(orientation) + 0.5*height*cos(orientation);
    }
}

ORO_CREATE_COMPONENT_LIBRARY()

