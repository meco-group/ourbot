#ifndef OROCOS_DISTRIBUTEDCONTROLLERINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDCONTROLLERINTERFACE_COMPONENT_HPP

#include "../ControllerInterface/ControllerInterface-component.hpp"

class DistributedControllerInterface : public ControllerInterface{
  private:
    InputPort<std::vector<double> > _ref_relposeA_inport;
    InputPort<std::vector<double> > _ref_relposeB_inport;
    InputPort<std::vector<double> > _est_poseA_inport;
    InputPort<std::vector<double> > _est_poseB_inport;

    std::vector<double> _ref_relposeA;
    std::vector<double> _ref_relposeB;
    std::vector<double> _est_poseA;
    std::vector<double> _est_poseB;

  protected:
    virtual bool startHook();
    virtual void updateHook();

    std::vector<double> getRefRelPoseA();
    std::vector<double> getRefRelPoseB();
    std::vector<double> getEstPoseA();
    std::vector<double> getEstPoseB();

  public:
    DistributedControllerInterface(std::string const& name);
    virtual bool controlUpdate() = 0;
    virtual bool initialize() = 0;
};

#endif
