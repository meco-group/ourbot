#ifndef OROCOS_REFERENCE_COMPONENT_HPP
#define OROCOS_REFERENCE_COMPONENT_HPP

// #define DEBUG

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>

using namespace RTT;

class Reference : public RTT::TaskContext{
  private:
    // Input & output ports
    InputPort<std::vector<double> > _ref_pose_trajectory_port[3];
    InputPort<std::vector<double> > _ref_velocity_trajectory_port[3];
    InputPort<std::vector<double> > _ref_pose_trajectory_ss_port[3];

    InputPort<std::vector<double> > _est_pose_port;
    OutputPort<std::vector<double> > _ref_pose_port;
    OutputPort<std::vector<double> > _ref_velocity_port;
    OutputPort<std::vector<double> > _mp_trigger_port;
    OutputPort<std::vector<double> > _ref_pose_trajectory_tx_port[3];

    // Define 2 containers of memory to store 2 trajectories
    std::vector<double> _ref_pose_trajectory_1[3];
    std::vector<double> _ref_velocity_trajectory_1[3];
    std::vector<double> _ref_pose_trajectory_2[3];
    std::vector<double> _ref_velocity_trajectory_2[3];

    std::vector<double> _ref_pose_trajectory_ss[3];

    // Define pointers to the memory which stores the current and next trajectory
    std::vector<double>* _cur_ref_velocity_trajectory;
    std::vector<double>* _cur_ref_pose_trajectory;
    std::vector<double>* _nxt_ref_pose_trajectory;
    std::vector<double>* _nxt_ref_velocity_trajectory;

    bool _valid_trajectories;

    // Current sample
    std::vector<double> _ref_pose_sample;
    std::vector<double> _ref_velocity_sample;

    // Estimated pose and trigger data
    std::vector<double> _est_pose;
    std::vector<double> _mp_trigger_data;

    // Which input ports are connected?
    bool _con_ref_pose_trajectory[3];
    bool _con_ref_velocity_trajectory[3];

    // From which input ports is data received?
    bool _got_ref_pose_trajectory[3];
    bool _got_ref_velocity_trajectory[3];

    bool _just_started;
    bool _first_time;
    double _orientation_homing_rate;

    double _control_sample_rate;
    double _pathupd_sample_rate;
    int _max_computation_periods;
    bool _repeat_offline_trajectory;
    std::string _trajectory_path;

    int _index1;
    int _index2;
    double _horizon_time;
    int _trajectory_length;
    int _trajectory_length_tx;
    int _tx_subsample;
    int _update_length;
    bool _new_data;
    bool _ready;
    bool _offline_trajectory;

    OperationCaller<bool(void)> mpValid;
    OperationCaller<void(void)> mpEnable;
    OperationCaller<void(void)> mpDisable;
    OperationCaller<bool(void)> mpGotTarget;
    OperationCaller<bool(void)> mpZeroOrientation;

    void reset();
    void triggerMotionPlanning();
    void writeRefSample();
    void loadTrajectories();
    void readPorts();
    void updateTxTrajectory();
    void interpolateOrientation(std::vector<double>& theta_trajectory, std::vector<double>& omega_trajectory);

  public:
    Reference(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    void writeSample();
    bool loadTrajectory();
    bool ready();
    bool setMotionPlanner(const std::string& motionplanning);
};
#endif
