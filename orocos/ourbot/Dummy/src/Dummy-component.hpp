#ifndef OROCOS_DUMMY_COMPONENT_HPP
#define OROCOS_DUMMY_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace RTT;

class Dummy : public RTT::TaskContext{
  private:
    InputPort<double> _dummy_in_port;
    InputPort<std::vector<double> > _dummy_vec_in_port1;
    InputPort<std::vector<double> > _dummy_vec_in_port2;
    InputPort<std::vector<double> > _dummy_vec_in_port3;
    InputPort<std::vector<double> > _dummy_vec_in_port4;
    InputPort<std::vector<double> > _dummy_vec_in_port5;
    InputPort<std::vector<double> > _dummy_vec_in_port6;
    InputPort<std::vector<double> > _dummy_vec_in_port7;
    InputPort<std::vector<double> > _dummy_vec_in_port8;
    InputPort<std::vector<double> > _dummy_vec_in_port9;
    InputPort<std::vector<double> > _dummy_vec_in_port10;
    InputPort<std::vector<double> > _dummy_vec_in_port11;
    InputPort<std::vector<double> > _dummy_vec_in_port12;
    InputPort<std::vector<double> > _dummy_vec_in_port13;
    InputPort<std::vector<double> > _dummy_vec_in_port14;
    InputPort<std::vector<double> > _dummy_vec_in_port15;
    InputPort<std::vector<double> > _dummy_vec_in_port16;
    InputPort<std::vector<double> > _dummy_vec_in_port17;
    InputPort<std::vector<double> > _dummy_vec_in_port18;
    InputPort<std::vector<double> > _dummy_vec_in_port19;
    InputPort<std::vector<double> > _dummy_vec_in_port20;
    OutputPort<double> _dummy_out_port;
    OutputPort<std::vector<double> > _dummy_vec_out_port1;
    OutputPort<std::vector<double> > _dummy_vec_out_port2;
    OutputPort<std::vector<double> > _dummy_vec_out_port3;
    OutputPort<std::vector<double> > _dummy_vec_out_port4;
    OutputPort<std::vector<double> > _dummy_vec_out_port5;
    OutputPort<std::vector<double> > _dummy_vec_out_port6;
    OutputPort<std::vector<double> > _dummy_vec_out_port7;
    OutputPort<std::vector<double> > _dummy_vec_out_port8;
    OutputPort<std::vector<double> > _dummy_vec_out_port9;
    OutputPort<std::vector<double> > _dummy_vec_out_port10;
    OutputPort<std::vector<double> > _dummy_vec_out_port11;
    OutputPort<std::vector<double> > _dummy_vec_out_port12;
    OutputPort<std::vector<double> > _dummy_vec_out_port13;
    OutputPort<std::vector<double> > _dummy_vec_out_port14;
    OutputPort<std::vector<double> > _dummy_vec_out_port15;
    OutputPort<std::vector<double> > _dummy_vec_out_port16;
    OutputPort<std::vector<double> > _dummy_vec_out_port17;
    OutputPort<std::vector<double> > _dummy_vec_out_port18;
    OutputPort<std::vector<double> > _dummy_vec_out_port19;
    OutputPort<std::vector<double> > _dummy_vec_out_port20;

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
