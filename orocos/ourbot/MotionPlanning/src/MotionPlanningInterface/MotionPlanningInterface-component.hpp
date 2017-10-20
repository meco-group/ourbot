#ifndef OROCOS_MOTIONPLANNINGINTERFACE_COMPONENT_HPP
#define OROCOS_MOTIONPLANNINGINTERFACE_COMPONENT_HPP

#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) std::cout << x << std::endl;
#else
    #define DEBUG_PRINT(x) //std::cout << x << std::endl;
#endif

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <math.h>

using namespace RTT;
using namespace RTT::os;

typedef struct obstacle {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> checkpoints;
    std::vector<double> radii;
    std::vector<double> traj_coeffs;
    bool avoid;
} obstacle_t;

class MotionPlanningInterface : public RTT::TaskContext {

    private:
        InputPort<std::vector<double> > _target_pose_port;
        InputPort<std::vector<double> > _mp_trigger_port;
        InputPort<std::vector<double> > _obstacle_port;
        InputPort<std::vector<double> > _robobs_pose_port;
        InputPort<std::vector<double> > _robobs_velocity_port;

        OutputPort<std::vector<double> > _ref_pose_trajectory_port[3];
        OutputPort<std::vector<double> > _ref_velocity_trajectory_port[3];
        OutputPort<double> _motion_time_port;

        double _orientation_th;

        TimeService::ticks _timestamp;
        double updateOrientationTrajectory();
        std::vector<double> _mp_trigger_data;
        bool _target_set;
        bool _ready;
        bool _busy;
        bool _valid;
        bool _first_iteration;
        bool _increase;

    protected:
        virtual bool initialize() = 0;
        virtual bool config() = 0;
        virtual double getMotionTime() = 0;
        virtual bool targetReached();
        virtual bool updatePoseTrajectory();
        virtual bool updatePositionTrajectory() = 0;
        virtual int n_obstacles() { return 0; }
        virtual void initObstacles(std::vector<obstacle_t>& obstacles, int n_obs);
        virtual void fillObstacles(std::vector<obstacle_t>& obstacles);
        virtual void patchup();

        double _target_dist_tol;
        double _input_norm_tol;
        double _angle_dist_tol;
        double _max_vel_position;
        double _max_vel_orientation;
        double _max_acc_position;
        double _max_acc_orientation;

        InputPort<std::vector<double> > _est_pose_port;
        std::vector<obstacle_t> _obstacles;
        int _trajectory_length;
        int _update_length;
        double _control_rate;
        double _motionplanning_rate;
        double _horizon_time;
        double _sample_time;
        double _update_time;
        double _rotation_time;
        int _predict_shift;
        std::vector<double> _est_pose;
        std::vector<double> _target_pose;
        std::vector<std::vector<double> > _ref_pose_trajectory;
        std::vector<std::vector<double> > _ref_velocity_trajectory;

    public:
        MotionPlanningInterface(std::string const& name);
        virtual bool configureHook();
        virtual void updateHook();
        virtual void setTargetPose(const std::vector<double>& target_pose);
        virtual bool reset();
        virtual bool ready();
        virtual bool busy();
        virtual bool valid();
};
#endif

