#ifndef OROCOS_MOTIONPLANNING_COMPONENT_HPP
#define OROCOS_MOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>
#include "Holonomic_p2p.hpp"
#include "Point2Point_p2p.hpp"

class MotionPlanning : public MotionPlanningInterface{
  private:
    int _n_st;
    int _n_in;
    omg::Point2Point* _p2p;

    std::vector<std::vector<double> > _ref_pose;
    std::vector<std::vector<double> > _ref_velocity;


  public:
    MotionPlanning(std::string const& name);

    bool trajectoryUpdate();
    bool initialize();
    virtual void getObstacles(std::vector<omg::obstacle_t>& obstacles);
    virtual bool config();
};

#endif
