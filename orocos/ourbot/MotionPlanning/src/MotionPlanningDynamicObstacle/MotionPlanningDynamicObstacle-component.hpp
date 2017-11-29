#ifndef OROCOS_MOTIONPLANNINGDYNAMICOBSTACLE_COMPONENT_HPP
#define OROCOS_MOTIONPLANNINGDYNAMICOBSTACLE_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include "../MotionPlanning/MotionPlanning-component.hpp"
#include <vector>
#include "Holonomic_p2pdynobst.hpp"
#include "Point2Point_p2pdynobst.hpp"

class MotionPlanningDynamicObstacle : public MotionPlanning {

    protected:
        virtual bool config();
        virtual bool updatePositionTrajectory();

    public:
        MotionPlanningDynamicObstacle(std::string const& name);
};

#endif
