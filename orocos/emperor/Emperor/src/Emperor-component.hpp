#ifndef OROCOS_EMPEROR_COMPONENT_HPP
#define OROCOS_EMPEROR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
using namespace RTT;

class Emperor : public RTT::TaskContext{
    OutputPort<std::string>_fsm_event_outport;
    InputPort<std::string>_current_state_evport;

  public:
    Emperor(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void setStateRun();
    void setStateIdle();
    void setStateIdentify();
};
#endif
