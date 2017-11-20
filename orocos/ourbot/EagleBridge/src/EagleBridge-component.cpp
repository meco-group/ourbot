#include "EagleBridge-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

EagleBridge::EagleBridge(std::string const& name) : TaskContext(name, PreOperational), _detected_pose(4) {
    // ports
    ports()->addPort("detected_pose_port", _detected_pose_port).doc("Detected pose with timestamp");
    // properties
    addProperty("host", _host).doc("Name of this host");
    addProperty("communication_group", _communication_group).doc("Zyre group to communicate eagle messages");
    addProperty("eagles", _eagles).doc("Name of eagle nodes to listen to");
    addProperty("verbose", _verbose).doc("Verbose flag");
    addProperty("robot_ids", _robot_ids).doc("Id's of robots to detect");
    // operations
    addOperation("getCircularObstacles", &EagleBridge::getCircularObstacles, this);
    addOperation("getRectangularObstacles", &EagleBridge::getRectangularObstacles, this);
}

bool EagleBridge::configureHook() {
    // show example data sample to ports to make data flow real-time
    _detected_pose_port.setDataSample(_detected_pose);
    std::string node_name = "eagle_bridge_";
    node_name.append(_host);
    _communicator = new eagle::Communicator(node_name, "wlan0", 5670);
    _communicator->verbose(0);
    _communicator->start(0);
    _communicator->join(_communication_group);
    init_robot();
    return true;
}

bool EagleBridge::startHook() {
    return true;
}

void EagleBridge::updateHook() {
    receive_detected();
    print(_verbose);
}

void EagleBridge::stopHook() {
    _communicator->stop();
}

std::vector<double> EagleBridge::getCircularObstacles(int n) {
    std::vector<double> ret(0);
    int cnt = 0;
    for (uint k=0; k<_obstacles.size(); k++) {
        if (cnt == n) {
            break;
        }
        if (dynamic_cast<eagle::CircleObstacle*>(_obstacles[k]) != nullptr) {
            ret.push_back(((eagle::CircleObstacle*)_obstacles[k])->center().x);
            ret.push_back(((eagle::CircleObstacle*)_obstacles[k])->center().y);
            ret.push_back(((eagle::CircleObstacle*)_obstacles[k])->radius());
            cnt++;
        }
    }
    return ret;
}

std::vector<double> EagleBridge::getRectangularObstacles(int n) {
    std::vector<double> ret(0);
    int cnt = 0;
    for (uint k=0; k<_obstacles.size(); k++) {
        if (cnt == n) {
            break;
        }
        if (dynamic_cast<eagle::RectangleObstacle*>(_obstacles[k]) != nullptr) {
            ret.push_back(((eagle::RectangleObstacle*)_obstacles[k])->center().x);
            ret.push_back(((eagle::RectangleObstacle*)_obstacles[k])->center().y);
            ret.push_back(((eagle::RectangleObstacle*)_obstacles[k])->angle());
            ret.push_back(((eagle::RectangleObstacle*)_obstacles[k])->width());
            ret.push_back(((eagle::RectangleObstacle*)_obstacles[k])->height());
        }
        cnt++;
    }
    return ret;
}

void EagleBridge::init_robot() {
    std::vector<std::string> hosts = {"dave", "krist", "kurt"};
    for (uint k=0; k<hosts.size(); k++) {
        if (_host == hosts[k]) {
            _id = _robot_ids[k];
            break;
        }
    }
}

void EagleBridge::receive_detected() {
    if (_communicator->receive()) {
        std::map<std::string, eagle::marker_t> marker_msgs;
        std::map<std::string, unsigned long> marker_times;
        std::map<std::string, std::vector<eagle::Obstacle*> > obstacle_msgs;
        // handle received messages
        while (_communicator->pop_message(_message)) {
            if (std::find(_eagles.begin(), _eagles.end(), _message.peer()) != _eagles.end()) {
                while (_message.available()) {
                    // read header
                    eagle::header_t header;
                    _message.read(&header);
                    switch (header.id) {
                        case eagle::MARKER: {
                            eagle::marker_t marker;
                            _message.read(&marker);
                            if (marker.id == _id) {
                                marker_msgs[_message.peer()] = marker;
                                marker_times[_message.peer()] = header.time;
                            }
                            break;
                        }
                        case eagle::OBSTACLE: {
                            eagle::obstacle_t obst;
                            _message.read(&obst);
                            obstacle_msgs[_message.peer()].push_back(eagle::Obstacle::deserialize(obst));
                            break;
                        }
                        default: {
                            _message.dump_frame();
                            break;
                        }
                    }
                }
            }
        }
        // merge detected robot information and send it
        int n_msg = marker_msgs.size();
        std::fill(_detected_pose.begin(), _detected_pose.end(), 0);
        if (n_msg > 0) {
            for (std::map<std::string, eagle::marker_t>::iterator mrk=marker_msgs.begin(); mrk!=marker_msgs.end(); ++mrk) {
                _detected_pose[0] += (1./n_msg)*mrk->second.x;
                _detected_pose[1] += (1./n_msg)*mrk->second.y;
                _detected_pose[2] += (1./n_msg)*mrk->second.yaw;
                _detected_pose[3] += (1./n_msg)*static_cast<double>(marker_times[mrk->first]);
            }
            _detected_pose_port.write(_detected_pose);
        }
        // merge detected obstacle information and save it
        _obstacles.clear();
        for (std::map<std::string, std::vector<eagle::Obstacle*>>::iterator obs=obstacle_msgs.begin(); obs!=obstacle_msgs.end(); ++obs) {
            if (obs->second.size() > 0) {
                _obst_per_peer[obs->first] = obs->second;
            }
            for (uint k=0; k<_obst_per_peer[obs->first].size(); k++) {
                _obstacles.push_back(_obst_per_peer[obs->first][k]);
            }
        }
        // sort obstacles
        sort_obstacles(_obstacles);
    }
}

void EagleBridge::print(int verbose) {
    if (verbose >= 1) {
        std::cout << "* Detected robot pose: [";
        std::cout << _detected_pose[0] << "," << _detected_pose[1] << "," << _detected_pose[2] << "]";
        std::cout << " at time " << _detected_pose[3] << "." <<std::endl;
        std::cout << "* Obstacles detected: " << std::endl;
        for (uint k=0; k<_obstacles.size(); k++) {
            std::cout << "\t- " << _obstacles[k]->to_string() << std::endl;
        }
    }
}

void EagleBridge::sort_obstacles(std::vector<eagle::Obstacle*>& obstacles) {
    uint j;
    eagle::Obstacle* obstacle;
    // insertion sort
    for (uint i=1; i<obstacles.size(); i++) {
        j = i;
        while (j > 0 && obstacles[j-1]->area() < obstacles[j]->area()) {
            // swap obstacles
            obstacle = obstacles[j];
            obstacles[j] = obstacles[j-1];
            obstacles[j-1] = obstacle;
        }
        j--;
    }
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(EagleBridge)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(EagleBridge)
