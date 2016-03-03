#ifndef OROCOS_DISTRIBUTEDREFERENCE_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDREFERENCE_COMPONENT_HPP

#include "../Reference/Reference-component.hpp"

class DistributedReference : public Reference{
  private:
    InputPort<std::vector<double> > _ref_relposeL_trajectory_port[3];
    InputPort<std::vector<double> > _ref_relposeR_trajectory_port[3];

    OutputPort<std::vector<double> > _ref_relposeL_port;
    OutputPort<std::vector<double> > _ref_relposeR_port;

    // Define 2 containers of memory to store 2 trajectories
    std::vector<double> _ref_relposeL_trajectory_1[3];
    std::vector<double> _ref_relposeR_trajectory_1[3];
    std::vector<double> _ref_relposeL_trajectory_2[3];
    std::vector<double> _ref_relposeR_trajectory_2[3];

    // Define pointers to the memory which stores the current and next trajectory
    std::vector<double>* _cur_ref_relposeL_trajectory;
    std::vector<double>* _cur_ref_relposeR_trajectory;
    std::vector<double>* _nxt_ref_relposeL_trajectory;
    std::vector<double>* _nxt_ref_relposeR_trajectory;

    // Current sample
    std::vector<double> _ref_relposeL_sample;
    std::vector<double> _ref_relposeR_sample;

    // Which input ports are connected?
    bool _con_ref_relposeL_trajectory[3];
    bool _con_ref_relposeR_trajectory[3];

    // From which input ports is data received?
    bool _got_ref_relposeL_trajectory[3];
    bool _got_ref_relposeR_trajectory[3];

    void readPorts();

  public:
    DistributedReference(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    void writeSample();
};

#endif
