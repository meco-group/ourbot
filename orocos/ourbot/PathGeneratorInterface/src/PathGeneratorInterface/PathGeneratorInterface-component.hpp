#ifndef OROCOS_PATHGENERATORINTERFACE_COMPONENT_HPP
#define OROCOS_PATHGENERATORINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class PathGeneratorInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _est_pose_inport;

    OutputPort<std::vector<double> > _ref_pose_path_outport[3];
    OutputPort<std::vector<double> > _ref_ffw_path_outport[3];

    std::vector<double> _est_pose;
    std::vector<double> _ref_pose_path[3];
    std::vector<double> _ref_ffw_path[3];

    int _control_sample_rate;
    int _pathUpd_sample_rate;
    std::vector<double> _kin_limitations;

  protected:
    virtual bool pathUpdate() = 0;
    virtual bool initialize() = 0;

    int _path_length;

    int getPathLength();
    int getControlSampleRate();
    int getPathUpdSampleRate();
    std::vector<double> getKinLimitations();
    std::vector<double> getEstPose();

    void setRefPosePath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);
    void setRefFfwPath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);

  public:
    PathGeneratorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};
#endif
