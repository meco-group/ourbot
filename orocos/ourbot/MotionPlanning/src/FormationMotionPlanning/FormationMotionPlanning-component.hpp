#ifndef OROCOS_FORMATIONMOTIONPLANNING_COMPONENT_HPP
#define OROCOS_FORMATIONMOTIONPLANNING_COMPONENT_HPP

#include "../MotionPlanningInterface/MotionPlanningInterface-component.hpp"
#include "../MotionPlanning/MotionPlanning-component.hpp"
#include <vector>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include "Holonomic_p2pf.hpp"
#include "FormationPoint2Point_p2pf.hpp"

using namespace RTT;
using namespace RTT::os;

typedef struct _code_t {
    uint32_t index;
    bool valid;
    bool target_reached;
} __attribute__((packed)) code_t;

typedef struct _robot_t {
    std::string name;
    std::vector<double> pose;
} robot_t;

class FormationMotionPlanning : public MotionPlanning {

    private:
        OutputPort<std::vector<double> > _x_var_port[2];
        OutputPort<std::vector<double> > _zl_ij_var_port[2];
        InputPort<std::vector<double> > _x_j_var_port[2];
        InputPort<std::vector<double> > _zl_ji_var_port[2];

        std::vector<std::vector<double> > _ref_velocity;
        std::vector<std::vector<double> > _ref_pose;

        std::vector<double> _x_var;
        std::vector<std::vector<double> > _x_j_var;
        std::vector<std::vector<double> > _z_ji_var;
        std::vector<std::vector<double> > _l_ji_var;
        std::vector<std::vector<double> > _z_ij_var;
        std::vector<std::vector<double> > _l_ij_var;

        std::vector<double> _x_p_var;
        std::vector<std::vector<double> > _x_j_p_var;
        std::vector<std::vector<double> > _zl_ji_p_var;
        std::vector<std::vector<double> > _zl_ij_p_var;

        std::vector<bool> _target_reached_nghb;
        bool _target_reached;

        int _n_shared;

        std::string _host;
        std::vector<double> _configuration;
        int _init_admm_iter;
        double _rho;

        std::vector<robot_t> _robots;
        std::vector<robot_t> _neighbors;
        std::vector<double> _residuals;
        std::vector<double> _rel_pos_c;

        #ifdef DEBUG
        TimeService::ticks _timestamp;
        #endif

        bool admmIteration(bool initial);
        double encode(int index, bool valid);
        code_t decode(double number);
        void emptyPorts();
        bool watchDog(bool initial, TimeService::ticks t0);

    protected:
        virtual bool initialize();
        virtual bool config();
        virtual bool updatePositionTrajectory();
        virtual bool targetReached();

    public:
        FormationMotionPlanning(std::string const& name);
        void resetNeighbors();
        void addNeighbor(const std::string& name, const std::vector<double>& pose);
        std::vector<std::string> getNeighbors();
        bool connectWithNeighbors();
        virtual void setTargetPose(const std::vector<double>& target_pose);
        std::vector<double> buildFormation();

};

#endif
