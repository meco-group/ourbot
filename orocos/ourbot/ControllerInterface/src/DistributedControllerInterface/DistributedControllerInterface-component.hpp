#ifndef OROCOS_DISTRIBUTEDCONTROLLERINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDCONTROLLERINTERFACE_COMPONENT_HPP

#include "../ControllerInterface/ControllerInterface-component.hpp"

class DistributedControllerInterface : public ControllerInterface{
  private:
    InputPort<std::vector<double> > _ref_relposeA_inport;
    InputPort<std::vector<double> > _ref_relposeB_inport;
    InputPort<std::vector<double> > _est_poseA_inport;
    InputPort<std::vector<double> > _est_poseB_inport;
    // Communication with neighbouring controllers
    InputPort<std::vector<double> > _com_inA_inport;
    InputPort<std::vector<double> > _com_inB_inport;
    OutputPort<std::vector<double> > _com_outA_outport;
    OutputPort<std::vector<double> > _com_outB_outport;

    std::vector<double> _ref_relposeA;
    std::vector<double> _ref_relposeB;
    std::vector<double> _est_poseA;
    std::vector<double> _est_poseB;
    std::vector<double> _com_inA;
    std::vector<double> _com_inB;
    std::vector<double> _com_outA;
    std::vector<double> _com_outB;

    // Size of communication signals
    int _com_size;

  protected:
    virtual bool controlUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getRefRelPoseA();
    std::vector<double> getRefRelPoseB();
    std::vector<double> getEstPoseA();
    std::vector<double> getEstPoseB();
    std::vector<double> getComInA();
    std::vector<double> getComInB();
    void setComOutA(std::vector<double> const&);
    void setComOutB(std::vector<double> const&);

  public:
    DistributedControllerInterface(std::string const& name, int com_size);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};

#endif
