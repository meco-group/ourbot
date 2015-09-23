#ifndef OROCOS_REFERENCE_COMPONENT_HPP
#define OROCOS_REFERENCE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class Reference : public RTT::TaskContext{
  private:
    // Input & output ports
    InputPort<std::vector<double> > _ref_pose_path_port[3];
    InputPort<std::vector<double> > _ref_ffw_path_port[3];
    OutputPort<std::vector<double> > _ref_pose_port;
    OutputPort<std::vector<double> > _ref_ffw_port;

    // Define 2 containers of memory to store 2 paths
    std::vector<double> _ref_pose_path_1[3];
    std::vector<double> _ref_ffw_path_1[3];
    std::vector<double> _ref_pose_path_2[3];
    std::vector<double> _ref_ffw_path_2[3];

    // Define pointers to the memory which stores the current and next path
    std::vector<double>* _cur_ref_ffw_path;
    std::vector<double>* _cur_ref_pose_path;
    std::vector<double>* _nxt_ref_pose_path;
    std::vector<double>* _nxt_ref_ffw_path;

    // Current sample
    std::vector<double> _ref_pose_sample;
    std::vector<double> _ref_ffw_sample;

    // Which input ports are connected?
    bool _con_ref_pose_path[3];
    bool _con_ref_ffw_path[3];

    // From which input ports is data received?
    bool _got_ref_pose_path[3];
    bool _got_ref_ffw_path[3];

    double _control_sample_rate;
    double _pathupd_sample_rate;

  protected:
    int _index;
    int _path_length;
    bool _new_data;
    void readPorts();

  public:
    Reference(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    void writeSample();
};
#endif
