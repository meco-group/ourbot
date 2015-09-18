#ifndef OROCOS_GAMEPAD_COMPONENT_HPP
#define OROCOS_GAMEPAD_COMPONENT_HPP

#include <rtt/RTT.hpp>

class GamePad : public RTT::TaskContext{
  public:
    GamePad(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
