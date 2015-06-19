#ifndef OROCOS_PATHGENERATORINTERFACE_COMPONENT_HPP
#define OROCOS_PATHGENERATORINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace RTT;
using namespace RTT::os;

class PathGeneratorInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _est_pose_port;               // estimated pose wrt initial frame (starts on 0.)
    InputPort<std::vector<double> > _est_global_offset_port;      // estimated offset of initial frame wrt world frame
    InputPort<std::vector<double> > _map_obstacles_port;          // obstacles wrt to world frame
    InputPort<std::vector<double> > _target_abspose_port;         // target wrt world frame
    InputPort<std::vector<double> > _target_relpose_port;         // target wrt initial frame

    OutputPort<std::vector<double> > _ref_pose_path_port[3];
    OutputPort<std::vector<double> > _ref_ffw_path_port[3];
    OutputPort<std::string> _fsm_event_port;

    std::vector<double> _est_pose;
    std::vector<double> _est_global_offset;
    std::vector<double> _target_abspose;
    std::vector<double> _target_relpose;
    std::vector<double> _map_obstacles;

    std::vector<double> _ref_pose_path[3];
    std::vector<double> _ref_ffw_path[3];

    double _period;
    TimeService::ticks _timestamp;
    int _init;
    double _control_sample_rate;
    double _pathupd_sample_rate;
    std::vector<double> _kin_limitations;
    int _obs_data_length;

    bool _target_abs;   // target is defined as abs of relative?

  protected:
    virtual bool pathUpdate() = 0;
    virtual bool initialize() = 0;

    int _path_length;

    int getPathLength();
    double getControlSampleRate();
    double getPathUpdSampleRate();
    int getObsDataLength();
    std::vector<double> getKinLimitations();
    std::vector<double> getEstPose();
    std::vector<double> getTargetPose();
    std::vector<double> getObstacleData();

    void setRefPosePath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);
    void setRefFfwPath(std::vector<double> const&, std::vector<double> const&, std::vector<double> const&);

  public:
    PathGeneratorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
};
#endif
