#ifndef OROCOS_MOTIONPLANNING_COMPONENT_HPP
#define OROCOS_MOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include <vector>
#include "Problem.hpp"
#include "Holonomic_p2p.hpp"
#include "Point2Point_p2p.hpp"

class MotionPlanning : public MotionPlanningInterface {

    private:
        std::vector<std::vector<double> > _ref_velocity;
        std::vector<std::vector<double> > _ref_pose;

    protected:
        ProblemInterface* _problem;
        bool _ideal_prediction;
        int _n_st;
        int _n_in;
        virtual int n_obstacles() { return _problem->n_obstacles(); }
        virtual bool initialize();
        virtual bool config();
        virtual bool updatePositionTrajectory();
        virtual void patchup();
        virtual double getMotionTime();
        void save(const std::vector<std::vector<double> >& ref_pose, const std::vector<std::vector<double> >& ref_velocity);

    public:
        MotionPlanning(std::string const& name);
};

#endif
