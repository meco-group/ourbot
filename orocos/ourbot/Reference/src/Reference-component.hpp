#ifndef OROCOS_REFERENCE_COMPONENT_HPP
#define OROCOS_REFERENCE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class Reference : public RTT::TaskContext{
  private:
    // Input & output ports
    InputPort<std::vector<double> > _ref_pose_trajectory_port[3];
    InputPort<std::vector<double> > _ref_velocity_trajectory_port[3];
    OutputPort<std::vector<double> > _ref_pose_port;
    OutputPort<std::vector<double> > _ref_velocity_port;
    OutputPort<int> _predict_shift_port;
    OutputPort<std::vector<double> > _ref_pose_trajectory_tx_port[3];

    // Define 2 containers of memory to store 2 trajectories
    std::vector<double> _ref_pose_trajectory_1[3];
    std::vector<double> _ref_velocity_trajectory_1[3];
    std::vector<double> _ref_pose_trajectory_2[3];
    std::vector<double> _ref_velocity_trajectory_2[3];

    // Define pointers to the memory which stores the current and next trajectory
    std::vector<double>* _cur_ref_velocity_trajectory;
    std::vector<double>* _cur_ref_pose_trajectory;
    std::vector<double>* _nxt_ref_pose_trajectory;
    std::vector<double>* _nxt_ref_velocity_trajectory;

    // Current sample
    std::vector<double> _ref_pose_sample;
    std::vector<double> _ref_velocity_sample;

    // Which input ports are connected?
    bool _con_ref_pose_trajectory[3];
    bool _con_ref_velocity_trajectory[3];

    // From which input ports is data received?
    bool _got_ref_pose_trajectory[3];
    bool _got_ref_velocity_trajectory[3];

    bool _just_started;

    double _control_sample_rate;
    double _pathupd_sample_rate;

    std::string _trajectory_path;

    int _index1;
    int _index2;
    int _trajectory_length;
    int _update_length;
    bool _new_data;
    bool _offline_trajectory;

    void readPorts();

  public:
    Reference(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    void writeSample();
    bool loadTrajectory();
};
#endif
