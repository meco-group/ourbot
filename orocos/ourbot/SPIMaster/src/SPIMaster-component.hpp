#ifndef OROCOS_SPIMASTER_COMPONENT_HPP
#define OROCOS_SPIMASTER_COMPONENT_HPP

#include <rtt/RTT.hpp>

class SPIMaster : public RTT::TaskContext{
  public:
    SPIMaster(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
