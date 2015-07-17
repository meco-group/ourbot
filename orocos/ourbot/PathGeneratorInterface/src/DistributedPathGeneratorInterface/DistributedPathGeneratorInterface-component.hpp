#ifndef OROCOS_DISTRIBUTEDPATHGENERATORINTERFACE_COMPONENT_HPP
#define OROCOS_DISTRIBsUTEDPATHGENERATORINTERFACE_COMPONENT_HPP

#include "../PathGeneratorInterface/PathGeneratorInterface-component.hpp"

class DistributedPathGeneratorInterface : public PathGeneratorInterface{
  private:
    InputPort<std::vector<double> > _est_poseL_port;
    InputPort<std::vector<double> > _est_poseR_port;
    InputPort<std::vector<double> > _ref_inL_path_port[3];
    InputPort<std::vector<double> > _ref_inR_path_port[3];

    OutputPort<std::vector<double> > _ref_relposeL_path_port[3];
    OutputPort<std::vector<double> > _ref_relposeR_path_port[3];

    std::vector<double> _est_poseL;
    std::vector<double> _est_poseR;
    std::vector<double> _ref_inL_path[3];
    std::vector<double> _ref_inR_path[3];
    std::vector<double> _ref_relposeL_path[3];
    std::vector<double> _ref_relposeR_path[3];

  protected:
    virtual bool pathUpdate() = 0;
    virtual bool initialize() = 0;

    std::vector<double> getEstPoseL();
    std::vector<double> getEstPoseR();

    std::vector<double> getRefInLPath_x();
    std::vector<double> getRefInLPath_y();
    std::vector<double> getRefInLPath_t();

    std::vector<double> getRefInRPath_x();
    std::vector<double> getRefInRPath_y();
    std::vector<double> getRefInRPath_t();

    void setRefRelPoseLPath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);
    void setRefRelPoseRPath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);

  public:
    DistributedPathGeneratorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};

#endif
