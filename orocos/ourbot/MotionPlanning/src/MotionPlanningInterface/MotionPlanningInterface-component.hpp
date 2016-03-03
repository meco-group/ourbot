#ifndef OROCOS_MOTIONPLANNINGINTERFACE_COMPONENT_HPP
#define OROCOS_MOTIONPLANNINGINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include "MotionPlanning.hpp"

using namespace RTT;
using namespace RTT::os;

class MotionPlanningInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _est_pose_port;
    InputPort<std::vector<double> > _target_pose_port;
    InputPort<std::vector<double> > _target_velocity_port;
    OutputPort<std::vector<double> > _ref_pose_trajectory_port[3];
    OutputPort<std::vector<double> > _ref_velocity_trajectory_port[3];

    double _period;
    TimeService::ticks _timestamp;
    int _init;
    std::vector<double> _kin_limitations;

  protected:
    virtual bool trajectoryUpdate() = 0;
    virtual bool initialize() = 0;

    int _trajectory_length;
    double _control_sample_rate;
    double _pathupd_sample_rate;
    double _horizon_time;
    double _sample_time;
    double _update_time;
    std::vector<double> _est_pose;
    std::vector<double> _target_pose;
    std::vector<double> _target_velocity;
    std::vector<mp::obstacle_t> _obstacles;

    std::vector<std::vector<double> > _ref_pose_trajectory;
    std::vector<std::vector<double> > _ref_velocity_trajectory;

  public:
    MotionPlanningInterface(std::string const& name);
    void setTargetPose(std::vector<double> const&);
    void setTargetVelocity(std::vector<double> const&);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    void writeSample();
};
#endif
