#ifndef OROCOS_DUMMY_COMPONENT_HPP
#define OROCOS_DUMMY_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace RTT;

class Dummy : public RTT::TaskContext{
  private:
    InputPort<double> _dummy_in_port;
    InputPort<std::vector<double> > _dummy_vec_in_port;
    OutputPort<double> _dummy_out_port;
    OutputPort<std::vector<double> > _dummy_vec_out_port;

    double _dummy_in;
    double _dummy_out;
    std::vector<double> _dummy_vec_in;
    std::vector<double> _dummy_vec_out;

  public:

    Dummy(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
