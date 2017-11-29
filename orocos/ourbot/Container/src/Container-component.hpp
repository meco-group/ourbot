#ifndef OROCOS_CONTAINER_COMPONENT_HPP
#define OROCOS_CONTAINER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

typedef std::vector<RTT::TaskContext*> Components;
typedef std::vector<RTT::base::PortInterface*> Ports;

class Container : public RTT::TaskContext {
private:
    Components _components;

public:
    Container(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    bool addComponent( const std::string& component, const std::string& component_name = "");
    Components getComponents();
};
#endif
