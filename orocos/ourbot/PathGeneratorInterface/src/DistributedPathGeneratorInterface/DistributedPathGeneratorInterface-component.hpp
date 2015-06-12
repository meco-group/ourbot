#ifndef OROCOS_DISTRIBUTEDPATHGENERATORINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDPATHGENERATORINTERFACE_COMPONENT_HPP

#include "../PathGeneratorInterface/PathGeneratorInterface-component.hpp"

class DistributedPathGeneratorInterface : public PathGeneratorInterface{
  private:
    InputPort<std::vector<double> > _est_poseA_inport;
    InputPort<std::vector<double> > _est_poseB_inport;
    InputPort<std::vector<double> > _ref_inA_path_inport[3];
    InputPort<std::vector<double> > _ref_inB_path_inport[3];

    OutputPort<std::vector<double> > _ref_relposeA_path_outport[3];
    OutputPort<std::vector<double> > _ref_relposeB_path_outport[3];

    std::vector<double> _est_poseA;
    std::vector<double> _est_poseB;
    std::vector<double> _ref_inA_path[3];
    std::vector<double> _ref_inB_path[3];
    std::vector<double> _ref_relposeA_path[3];
    std::vector<double> _ref_relposeB_path[3];

  protected:
    virtual bool pathUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getEstPoseA();
    std::vector<double> getEstPoseB();

    std::vector<double> getRefInAPath_x();
    std::vector<double> getRefInAPath_y();
    std::vector<double> getRefInAPath_t();

    std::vector<double> getRefInBPath_x();
    std::vector<double> getRefInBPath_y();
    std::vector<double> getRefInBPath_t();

    void setRefRelPoseAPath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);
    void setRefRelPoseBPath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);

  public:
    DistributedPathGeneratorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};

#endif
