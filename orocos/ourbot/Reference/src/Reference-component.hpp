#ifndef OROCOS_REFERENCE_COMPONENT_HPP
#define OROCOS_REFERENCE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class Reference : public RTT::TaskContext{
  private:
    // Input & output ports
    InputPort<std::vector<double> > _ref_pose_inport[3];
    InputPort<std::vector<double> > _ref_ffw_inport[3];
    InputPort<std::vector<double> > _ref_relposeA_inport[3];
    InputPort<std::vector<double> > _ref_relposeB_inport[3];

    OutputPort<std::vector<double> > _ref_pose_outport;
    OutputPort<std::vector<double> > _ref_ffw_outport;
    OutputPort<std::vector<double> > _ref_relposeA_outport;
    OutputPort<std::vector<double> > _ref_relposeB_outport;

    // Define 2 containers of memory to store 2 paths
    std::vector<double> _ref_pose_1[3];
    std::vector<double> _ref_ffw_1[3];
    std::vector<double> _ref_relposeA_1[3];
    std::vector<double> _ref_relposeB_1[3];

    std::vector<double> _ref_pose_2[3];
    std::vector<double> _ref_ffw_2[3];
    std::vector<double> _ref_relposeA_2[3];
    std::vector<double> _ref_relposeB_2[3];

    // Define pointers to the memory which stores the current and next path
    std::vector<double>* _cur_ref_ffw;
    std::vector<double>* _cur_ref_pose;
    std::vector<double>* _cur_ref_relposeA;
    std::vector<double>* _cur_ref_relposeB;

    std::vector<double>* _nxt_ref_pose;
    std::vector<double>* _nxt_ref_ffw;
    std::vector<double>* _nxt_ref_relposeA;
    std::vector<double>* _nxt_ref_relposeB;

    // Current sample
    std::vector<double> _ref_pose_sample;
    std::vector<double> _ref_ffw_sample;
    std::vector<double> _ref_relposeA_sample;
    std::vector<double> _ref_relposeB_sample;

    // Which ports are connected?
    bool _con_ref_pose[3];
    bool _con_ref_ffw[3];
    bool _con_ref_relposeA[3];
    bool _con_ref_relposeB[3];

    // From which ports is data received?
    bool _got_ref_pose[3];
    bool _got_ref_ffw[3];
    bool _got_ref_relposeA[3];
    bool _got_ref_relposeB[3];

    bool _new_data;
    int _path_length;
    int _index;

    void readPorts();

  public:
    Reference(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
};
#endif
