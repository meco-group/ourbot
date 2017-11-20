#ifndef OROCOS_CAMERA_COMPONENT_HPP
#define OROCOS_CAMERA_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Camera : public RTT::TaskContext{
  public:
    Camera(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
