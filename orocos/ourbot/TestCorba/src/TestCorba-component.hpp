#ifndef OROCOS_TESTCORBA_COMPONENT_HPP
#define OROCOS_TESTCORBA_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace RTT;
using namespace RTT::os;

class TestCorba : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _channel1_port;
    int _datasize;
    std::vector<double> _readdata;
    double _period;
    TimeService::ticks _timestamp;
    int _init;

  public:
    TestCorba(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
