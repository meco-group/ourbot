#ifndef OROCOS_TESTCORBA_COMPONENT_HPP
#define OROCOS_TESTCORBA_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class TestCorba : public RTT::TaskContext{
  private:
    OutputPort<std::vector<double> > _channel1_port;
    int _datasize;
    double _cnt;
    std::vector<double> _writevalue;

  public:
    TestCorba(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
