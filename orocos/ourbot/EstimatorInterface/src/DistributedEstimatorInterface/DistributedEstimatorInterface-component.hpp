#ifndef OROCOS_DISTRIBUTEDESTIMATORINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDESTIMATORINTERFACE_COMPONENT_HPP

#include "../EstimatorInterface/EstimatorInterface-component.hpp"

class DistributedEstimatorInterface : public EstimatorInterface{
  private:
    // Communication with neighbouring controllers
    InputPort<std::vector<double> > _com_inA_inport;
    InputPort<std::vector<double> > _com_inB_inport;

    OutputPort<std::vector<double> > _est_poseA_outport;
    OutputPort<std::vector<double> > _est_poseB_outport;

    std::vector<double> _com_inA;
    std::vector<double> _com_inB;
    std::vector<double> _est_poseA;
    std::vector<double> _est_poseB;

    int _com_size;

  protected:
    virtual bool estimateUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getComInA();
    std::vector<double> getComInB();
    void setEstPoseA(std::vector<double> const&);
    void setEstPoseB(std::vector<double> const&);

  public:
    DistributedEstimatorInterface(std::string const& name, int com_size);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};

#endif
