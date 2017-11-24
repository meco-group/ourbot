#ifndef OROCOS_EAGLEBRIDGE_COMPONENT_HPP
#define OROCOS_EAGLEBRIDGE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <eagle.h>
#include <vector>
#include <stdint.h>

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
        std::vector<double> _robot_sizes;

        unsigned int _id;
        std::vector<double> _detected_pose;
        eagle::Collector* _collector;
        eagle::Communicator* _communicator;
        eagle::Message _message;
        std::vector<eagle::Robot*> _robots;
        std::vector<eagle::Obstacle*> _obstacles;
        std::vector<uint32_t> _robot_timestamps;
        std::vector<uint32_t> _obstacle_timestamps;
        uint32_t _prev_timestamp;

        void init_robots();
        void receive_detected();
        void print(int verbose);
        void sort_obstacles(std::vector<eagle::Obstacle*>& obstacles);

    public:
        EagleBridge(std::string const& name);
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();

        std::vector<double> getCircularObstacles(int n);
        std::vector<double> getRectangularObstacles(int n);
        std::vector<double> getRobotPose(int id);
        int getRobotTimestamp(int id);
};
#endif
