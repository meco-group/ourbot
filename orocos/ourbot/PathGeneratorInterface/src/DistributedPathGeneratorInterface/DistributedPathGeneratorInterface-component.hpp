#ifndef OROCOS_DISTRIBUTEDPATHGENERATORINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDPATHGENERATORINTERFACE_COMPONENT_HPP

#include "../PathGeneratorInterface/PathGeneratorInterface-component.hpp"

class DistributedPathGeneratorInterface : public PathGeneratorInterface{
  private:
    InputPort<std::vector<double> > _est_poseA_inport;
    InputPort<std::vector<double> > _est_poseB_inport;

    OutputPort<std::vector<double> > _ref_relposeA_outport[3];
    OutputPort<std::vector<double> > _ref_relposeB_outport[3];

    InputPort<std::vector<double> > _ref_inA_inport[3];
    InputPort<std::vector<double> > _ref_inB_inport[3];

    std::vector<double> _est_poseA;
    std::vector<double> _est_poseB;
    std::vector<double> _ref_relposeA[3];
    std::vector<double> _ref_relposeB[3];
    std::vector<double> _ref_inA[3];
    std::vector<double> _ref_inB[3];

  protected:
    virtual bool pathUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getEstPoseA();
    std::vector<double> getEstPoseB();

    std::vector<double> getRefInA_x();
    std::vector<double> getRefInA_y();
    std::vector<double> getRefInA_t();

    std::vector<double> getRefInB_x();
    std::vector<double> getRefInB_y();
    std::vector<double> getRefInB_t();

    void setRefRelPoseA(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);
    void setRefRelPoseB(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);

  public:
    DistributedPathGeneratorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};

#endif
