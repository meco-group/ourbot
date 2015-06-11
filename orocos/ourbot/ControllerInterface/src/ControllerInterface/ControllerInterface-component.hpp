#ifndef OROCOS_CONTROLLERINTERFACE_COMPONENT_HPP
#define OROCOS_CONTROLLERINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace RTT;

class ControllerInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _ref_pose_inport;
    InputPort<std::vector<double> > _ref_ffw_inport;
    InputPort<std::vector<double> > _est_pose_inport;

    OutputPort<std::vector<double> > _set_velocity_outport;

    int _sample_rate;

    std::vector<double> _ref_pose;
    std::vector<double> _ref_ffw;
    std::vector<double> _est_pose;
    std::vector<double> _set_velocity;

  protected:
    virtual bool controlUpdate() = 0;
    virtual bool initialize() = 0;

  public:
    ControllerInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();

    double getSampleRate();
    std::vector<double> getRefPose();
    std::vector<double> getRefFfw();
    std::vector<double> getEstPose();
    std::vector<double> getSetVelocity();
    void setSetVelocity(std::vector<double> const&);
};
#endif
