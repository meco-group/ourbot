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
    std::vector<double> _obstacle_radius;
    std::vector<double> _obstacle_checkpts;
    bool _robotarm_avoid;

    std::vector<double> _old_estimated_pose;

    double _eps_position;
    double _eps_velocity;
    double _horizon_time;
    double _vmax;

    omg::obstacle_t guest_obstacle;

    void getObstacles(std::vector<omg::obstacle_t>& obstacles);
    bool config();

  public:
    FlexonomyMotionPlanning(std::string const& name);
    void writeHostObstTraj(int option);
    double getMotionTime();
};

#endif
