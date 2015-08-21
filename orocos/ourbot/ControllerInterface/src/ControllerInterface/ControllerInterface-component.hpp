#ifndef OROCOS_CONTROLLERINTERFACE_COMPONENT_HPP
#define OROCOS_CONTROLLERINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace RTT;

class ControllerInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _ref_pose_port;
    InputPort<std::vector<double> > _ref_ffw_port;
    InputPort<std::vector<double> > _est_pose_port;

    OutputPort<std::vector<double> > _cmd_velocity_port;

    double _control_sample_rate;

    std::vector<double> _ref_pose;
    std::vector<double> _ref_ffw;
    std::vector<double> _est_pose;
    std::vector<double> _cmd_velocity;

  protected:
    virtual bool controlUpdate() = 0;
    virtual bool initialize() = 0;

    double getControlSampleRate();
    std::vector<double> getRefPose();
    std::vector<double> getRefFfw();
    std::vector<double> getEstPose();
    std::vector<double> getCmdVelocity();
    void setCmdVelocity(std::vector<double> const&);

  public:
    ControllerInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    void writeSample();
};
#endif
