#ifndef OROCOS_CONFIGURATOR_COMPONENT_HPP
#define OROCOS_CONFIGURATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Configurator : public RTT::TaskContext{
  public:
    Configurator(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    int getPathLength();
    int getControlSampleRate();
    int getPathUpdSampleRate();
    std::vector<double> getKinLimitations();
    int getNrofIR();
    int getLidarDataLength();
    int getObsDataLength();

};
#endif
