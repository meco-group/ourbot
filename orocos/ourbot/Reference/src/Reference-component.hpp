#ifndef OROCOS_REFERENCE_COMPONENT_HPP
#define OROCOS_REFERENCE_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Reference : public RTT::TaskContext{
  public:
    Reference(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
