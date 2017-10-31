#ifndef OROCOS_FLEXONOMYMOTIONPLANNING_COMPONENT_HPP
#define OROCOS_FLEXONOMYMOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include "../MotionPlanning/MotionPlanning-component.hpp"
#include <vector>
#include "Holonomic_p2pflex.hpp"
#include "Point2Point_p2pflex.hpp"

class FlexonomyMotionPlanning : public MotionPlanning {

    protected:
        virtual bool config();
        virtual bool updatePositionTrajectory();

    public:
        FlexonomyMotionPlanning(std::string const& name);
};

#endif
