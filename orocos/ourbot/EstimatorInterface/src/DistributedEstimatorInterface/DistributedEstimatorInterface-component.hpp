#ifndef OROCOS_DISTRIBUTEDESTIMATORINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDESTIMATORINTERFACE_COMPONENT_HPP

#include "../EstimatorInterface/EstimatorInterface-component.hpp"

class DistributedEstimatorInterface : public EstimatorInterface{
  private:
    // Communication with neighbouring controllers
    InputPort<std::vector<double> > _com_inL_port;
    InputPort<std::vector<double> > _com_inR_port;

    OutputPort<std::vector<double> > _est_poseL_port;
    OutputPort<std::vector<double> > _est_poseR_port;

    std::vector<double> _com_inL;
    std::vector<double> _com_inR;
    std::vector<double> _est_poseL;
    std::vector<double> _est_poseR;

    int _com_size;

  protected:
    virtual bool estimateUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getComInL();
    std::vector<double> getComInR();
    void setEstPoseL(std::vector<double> const&);
    void setEstPoseR(std::vector<double> const&);

  public:
    DistributedEstimatorInterface(std::string const& name, int com_size);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    void writeSample();
};

#endif
