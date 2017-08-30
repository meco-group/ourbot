#ifndef OROCOS_FLEXONOMYMOTIONPLANNING_COMPONENT_HPP
#define OROCOS_FLEXONOMYMOTIONPLANNING_COMPONENT_HPP

#include "MotionPlanning-component.hpp"

class FlexonomyMotionPlanning : public MotionPlanning{
  private:

    InputPort<std::vector<double> > _obstacle_trajectory_port;
    OutputPort<std::vector<double> > _host_obstacle_trajectory_port;

    std::vector<double> _robotarm_pose;
    std::vector<double> _robotarm_size;
    std::vector<double> _neighbor_size;

    std::vector<double> _old_estimated_pose;

    double _vmax;

    void getObstacles(std::vector<omg::obstacle_t>& obstacles);
    bool config();

  public:
    FlexonomyMotionPlanning(std::string const& name);
    void writeHostObstTraj(int option);
    double getMotionTime();
};

#endif
