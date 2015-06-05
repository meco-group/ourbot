#ifndef OROCOS_TEST_COMPONENT_HPP
#define OROCOS_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Test : public RTT::TaskContext{
  public:
    Test(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
