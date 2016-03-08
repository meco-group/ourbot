#ifndef OROCOS_MOTIONPLANNING_COMPONENT_HPP
#define OROCOS_MOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>

class MotionPlanning : public MotionPlanningInterface{
  private:
   omg::MotionPlanning* _p2p;
   int _cnt = 0;
   const int _cnt_max = 5;
   std::vector<double> _state0;
   std::vector<double> _stateT;

   std::vector<std::vector<double> > _ref_pose;
   std::vector<std::vector<double> > _ref_velocity;


  public:
    MotionPlanning(std::string const& name);

    bool trajectoryUpdate();
    bool initialize();
    bool config();
};

#endif
