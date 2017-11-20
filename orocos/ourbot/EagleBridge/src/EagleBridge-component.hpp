#ifndef OROCOS_EAGLEBRIDGE_COMPONENT_HPP
#define OROCOS_EAGLEBRIDGE_COMPONENT_HPP

#include <rtt/RTT.hpp>

class EagleBridge : public RTT::TaskContext{
  public:
    EagleBridge(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
