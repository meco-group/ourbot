#ifndef OROCOS_MOTIONPLANNING_COMPONENT_HPP
#define OROCOS_MOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>

class MotionPlanning : public MotionPlanningInterface{
  private:
   mp::MotionPlanning _mp;
   int _cnt = 0;
   const int _cnt_max = 5;

  public:
    MotionPlanning(std::string const& name);

    bool trajectoryUpdate();
    bool initialize();
};

#endif
