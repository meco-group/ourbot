#ifndef OROCOS_DISTRIBUTEDMOTIONPLANNING_COMPONENT_HPP
#define OROCOS_DISTRIBUTEDMOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include "Toolbox/src/Holonomic.hpp"
#include "Toolbox/src/FormationPoint2Point.hpp"

using namespace RTT;
using namespace RTT::os;

class DistributedMotionPlanning : public MotionPlanningInterface{
  private:
    OutputPort<std::vector<double> > _x_var_port;
    OutputPort<std::vector<double> > _zl_ij_var_port[2];

    InputPort<std::vector<double> > _x_j_var_port[2];
    InputPort<std::vector<double> > _zl_ji_var_port[2];

    omg::FormationPoint2Point* _problem;
    int _cnt = 0;
    const int _cnt_max = 5;
    std::vector<double> _state0;
    std::vector<double> _stateT;
    std::vector<omg::obstacle_t> _obstacles;

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

    int _n_shared;
    int _n_nghb;
    std::vector<int> _nghb_index;
    std::vector<double> _residuals;
    std::vector<double> _rel_pos_c;
    int _init_iter;
    double _rho;

    #ifdef DEBUG
    TimeService::ticks _timestamp;
    #endif

    bool admmIteration();

  public:
    DistributedMotionPlanning(std::string const& name);

    bool trajectoryUpdate();
    bool initialize();
    bool config();
    void writeSample();
    void setRelPoseC(std::vector<double>);
};

#endif
