#ifndef OROCOS_DISTRIBUTEDCONTROLLERINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDCONTROLLERINTERFACE_COMPONENT_HPP

#include "../ControllerInterface/ControllerInterface-component.hpp"

class DistributedControllerInterface : public ControllerInterface{
  private:
    InputPort<std::vector<double> > _ref_relposeL_port;
    InputPort<std::vector<double> > _ref_relposeR_port;
    InputPort<std::vector<double> > _est_poseL_port;
    InputPort<std::vector<double> > _est_poseR_port;
    // Communication with neighbouring controllers
    InputPort<std::vector<double> > _com_inL_port;
    InputPort<std::vector<double> > _com_inR_port;
    OutputPort<std::vector<double> > _com_outL_port;
    OutputPort<std::vector<double> > _com_outR_port;

    std::vector<double> _ref_relposeL;
    std::vector<double> _ref_relposeR;
    std::vector<double> _est_poseL;
    std::vector<double> _est_poseR;
    std::vector<double> _com_inL;
    std::vector<double> _com_inR;
    std::vector<double> _com_outL;
    std::vector<double> _com_outR;

    // Size of communication signals
    int _com_size;

  protected:
    virtual bool controlUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getRefRelPoseL();
    std::vector<double> getRefRelPoseR();
    std::vector<double> getEstPoseL();
    std::vector<double> getEstPoseR();
    std::vector<double> getComInL();
    std::vector<double> getComInR();
    void setComOutL(std::vector<double> const&);
    void setComOutR(std::vector<double> const&);

  public:
    DistributedControllerInterface(std::string const& name, int com_size);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    void writeSample();
};

#endif
