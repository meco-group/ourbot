#ifndef OROCOS_REFERENCE_COMPONENT_HPP
#define OROCOS_REFERENCE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class Reference : public RTT::TaskContext{
    private:
        InputPort<std::vector<double> > _ref_pose_trajectory_port[3];
        InputPort<std::vector<double> > _ref_velocity_trajectory_port[3];

        OutputPort<std::vector<double> > _ref_pose_port;
        OutputPort<std::vector<double> > _ref_velocity_port;
        OutputPort<std::vector<double> > _ref_position_trajectory_port[2];

        std::vector<double> _ref_pose_trajectory[3];
        std::vector<double> _ref_velocity_trajectory[3];
        std::vector<double> _ref_position_trajectory[2];

        bool _con_ref_pose_trajectory[3];
        bool _con_ref_velocity_trajectory[3];

        std::vector<double> _ref_pose;
        std::vector<double> _ref_velocity;

        bool _repeat_trajectory;
        int _n_samples_plot;
        int _index;
        bool _ready;
        std::vector<double> _pose_offset;

        void updatePositionTrajectory();

    public:
        Reference(std::string const& name);
        bool configureHook();
        bool startHook();
        void updateHook();

        void reset();
        void setPoseOffset(const std::vector<double>& pose_offset);
        bool receiveTrajectory(int index);
        bool loadTrajectory(const std::string& path);

};

#endif
