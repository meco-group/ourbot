#ifndef OROCOS_MOTIONPLANNING_COMPONENT_HPP
#define OROCOS_MOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>
#include "Holonomic_p2p.hpp"
#include "Point2Point_p2p.hpp"

class MotionPlanning : public MotionPlanningInterface {

    private:
        int _n_st;
        int _n_in;
        bool _ideal_prediction;
        std::vector<std::vector<double> > _ref_velocity;
        std::vector<std::vector<double> > _ref_pose;

    protected:
        omg::Point2Point* _p2p;
        int n_obstacles() { return _p2p->n_obs;}
        virtual bool initialize();
        virtual bool config();
        virtual bool updatePositionTrajectory();
        virtual void patchup();
        virtual double getMotionTime();


    public:
        MotionPlanning(std::string const& name);
};

#endif
