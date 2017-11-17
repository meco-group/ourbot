#ifndef OROCOS_EAGLEBRIDGE_COMPONENT_HPP
#define OROCOS_EAGLEBRIDGE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <eagle.h>

using namespace RTT;

class EagleBridge : public RTT::TaskContext{

    private:
        // ports
        OutputPort<std::vector<double> > _detected_pose_port;

        // properties
        std::string _host;
        std::string _communication_group;
        std::vector<std::string> _eagles;
        int _verbose;
        std::vector<int> _robot_ids;

        unsigned int _id;
        std::vector<double> _detected_pose;
        eagle::Communicator* _communicator;
        eagle::Message _message;
        std::vector<eagle::Obstacle*> _obstacles;
        std::map<std::string, std::vector<eagle::Obstacle*>> _obst_per_peer;

        void init_robot();
        void receive_detected();
        void print(int verbose);

    public:
        EagleBridge(std::string const& name);
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();

        std::vector<double> getCircularObstacles();
        std::vector<double> getRectangularObstacles();
};
#endif
