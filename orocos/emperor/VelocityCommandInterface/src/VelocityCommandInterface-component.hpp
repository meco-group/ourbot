#ifndef OROCOS_VELOCITYCOMMANDINTERFACE_COMPONENT_HPP
#define OROCOS_VELOCITYCOMMANDINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>

class VelocityCommandInterface : public RTT::TaskContext{
  public:
    VelocityCommandInterface(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
