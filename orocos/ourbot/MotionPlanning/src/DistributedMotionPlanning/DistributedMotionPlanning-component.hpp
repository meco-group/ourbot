#ifndef OROCOS_DISTRIBUTEDMOTIONPLANNING_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDMOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include "Holonomic_p2pf.hpp"
#include "FormationPoint2Point_p2pf.hpp"

using namespace RTT;
using namespace RTT::os;

typedef struct _code_t {
  uint32_t index;
  bool valid;
  bool target_reached;
} __attribute__((packed)) code_t;

class DistributedMotionPlanning : public MotionPlanningInterface{
  private:
    OutputPort<std::vector<double> > _x_var_port;
    OutputPort<std::vector<double> > _zl_ij_var_port[2];
    OutputPort<std::vector<double> > _negotiate_out_port;

    InputPort<std::vector<double> > _x_j_var_port[2];
    InputPort<std::vector<double> > _zl_ji_var_port[2];
    InputPort<std::vector<double> > _negotiate_in_port;

    int _n_st;
    int _n_in;
    omgf::FormationPoint2Point* _problem;

    std::vector<std::vector<double> > _ref_pose;
    std::vector<std::vector<double> > _ref_velocity;

    std::vector<double> _x_var;
    std::vector<std::vector<double> > _x_j_var;
    std::vector<std::vector<double> > _z_ji_var;
    std::vector<std::vector<double> > _l_ji_var;
    std::vector<std::vector<double> > _z_ij_var;
    std::vector<std::vector<double> > _l_ij_var;

    std::vector<double> _x_p_var;
    std::vector<std::vector<double> > _x_j_p_var;
    std::vector<std::vector<double> > _zl_ji_p_var;
    std::vector<std::vector<double> > _zl_ij_p_var;

    std::vector<bool> _target_reached_nghb;
    bool _target_reached;

    int _n_shared;
    int _n_nghb;
    std::vector<std::string> _neighbors;

    std::vector<double> _configuration_x;
    std::vector<double> _configuration_y;

    std::vector<int> _nghb_index;
    std::vector<double> _residuals;
    std::vector<double> _rel_pos_c;
    int _init_iter;
    double _rho;

    #ifdef DEBUG
    TimeService::ticks _timestamp;
    #endif

    bool admmIteration(bool initial);
    void getObstacles(std::vector<omgf::obstacle_t>& obstacles);
    double encode(int index, bool valid);
    code_t decode(double number);
    void emptyPorts();
    bool watchDog(bool initial, TimeService::ticks t0);

  public:
    DistributedMotionPlanning(std::string const& name);

    bool trajectoryUpdate();
    bool initialize();
    bool config();
    bool targetReached();
    void writeSample();
    void setRelPoseC(std::vector<double>);
    std::vector<double> setConfiguration(int number_of_robots);
};

#endif
