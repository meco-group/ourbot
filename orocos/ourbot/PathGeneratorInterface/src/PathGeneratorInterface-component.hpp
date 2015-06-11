#ifndef OROCOS_PATHGENERATORINTERFACE_COMPONENT_HPP
#define OROCOS_PATHGENERATORINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>

class PathGeneratorInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _ref_pose_outport[3];
    InputPort<std::vector<double> > _ref_ffw_outport[3];
    InputPort<std::vector<double> > _ref_relposeA_outport[3];
    InputPort<std::vector<double> > _ref_relposeB_outport[3];

  public:
    PathGeneratorInterface(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
