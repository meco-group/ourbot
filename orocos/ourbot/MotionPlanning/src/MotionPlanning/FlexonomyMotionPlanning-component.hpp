#ifndef OROCOS_FLEXONOMYMOTIONPLANNING_COMPONENT_HPP
#define OROCOS_FLEXONOMYMOTIONPLANNING_COMPONENT_HPP

#include "MotionPlanning-component.hpp"

class FlexonomyMotionPlanning : public MotionPlanning{
  private:

    InputPort<std::vector<double> > _obstacle_trajectory_port;
    OutputPort<std::vector<double> > _host_obstacle_trajectory_port;

    std::vector<double> _robotarm_pos;
    std::vector<double> _robotarm_checkpts;
    std::vector<double> _robotarm_radii;
    bool _robotarm_avoid;

    double _eps_position;
    double _eps_velocity;
    double _horizon_time;
    double _vmax;

    void getObstacles(std::vector<omg::obstacle_t>& obstacles);

  public:
    FlexonomyMotionPlanning(std::string const& name);
    void initHostObstTraj();
    void writeHostObstTraj(int option);
    double getMotionTime();
};

#endif
