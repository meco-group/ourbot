#include "EagleBridge-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <chrono>

EagleBridge::EagleBridge(std::string const& name) : TaskContext(name, PreOperational), _detected_pose(4) {
    // ports
    ports()->addPort("detected_pose_port", _detected_pose_port).doc("Detected pose with timestamp");
    // properties
    addProperty("host", _host).doc("Name of this host");
    addProperty("communication_group", _communication_group).doc("Zyre group to communicate eagle messages");
    addProperty("eagles", _eagles).doc("Name of eagle nodes to listen to");
    addProperty("verbose", _verbose).doc("Verbose flag");
    addProperty("robot_ids", _robot_ids).doc("Id's of robots to detect");
    addProperty("robot_sizes", _robot_sizes).doc("Sizes of robots to detect");
    // operations
    addOperation("getCircularObstacles", &EagleBridge::getCircularObstacles, this);
    addOperation("getRectangularObstacles", &EagleBridge::getRectangularObstacles, this);
}

bool EagleBridge::configureHook() {
    // show example data sample to ports to make data flow real-time
    _detected_pose_port.setDataSample(_detected_pose);
    std::string node_name = "eagle_bridge_";
    node_name.append(_host);
    _collector = new eagle::Collector();
    _communicator = new eagle::Communicator(node_name, "wlan0", 5670);
    _communicator->verbose(0);
    _communicator->start(0);
    _communicator->join(_communication_group);
    init_robots();
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

std::vector<double> EagleBridge::getRobotPose(int id) {
    std::vector<double> pose(0);
    for (uint k=0; k<_robots.size(); k++) {
        if (_robots[k]->id() == id) {
            pose.push_back(_robots[k]->translation().x);
            pose.push_back(_robots[k]->translation().y);
            pose.push_back(_robots[k]->rotation().z);
        }
    }
    return pose;
}


void EagleBridge::init_robots() {
    _robots.resize(_robot_ids.size());
    for (uint k=0; k<_robots.size(); k++) {
        _robots[k] = new eagle::Robot(_robot_ids[k], _robot_sizes[2*k], _robot_sizes[2*k+1]);
    }
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
        // handle received messages
        while (_communicator->pop_message(_message)) {
            if (std::find(_eagles.begin(), _eagles.end(), _message.peer()) != _eagles.end()) {
                std::vector<eagle::marker_t> robot_msgs;
                std::vector<unsigned long> robot_msg_times;
                std::vector<eagle::obstacle_t> obstacle_msgs;
                std::vector<unsigned long> obstacle_msg_times;
                bool detect_msg = false;
                if (_message.empty()) {
                    // nothing detected
                    detect_msg = true;
                }
                while (_message.available()) {
                    // read header
                    eagle::header_t header;
                    _message.read(&header);
                    switch (header.id) {
                        case eagle::MARKER: {
                            detect_msg = true;
                            eagle::marker_t marker;
                            _message.read(&marker);
                            robot_msgs.push_back(marker);
                            robot_msg_times.push_back(header.time);
                            break;
                        }
                        case eagle::OBSTACLE: {
                            detect_msg = true;
                            eagle::obstacle_t obst;
                            _message.read(&obst);
                            obstacle_msgs.push_back(obst);
                            obstacle_msg_times.push_back(header.time);
                            break;
                        }
                        default: {
                            _message.dump_frame();
                            break;
                        }
                    }
                }
                if (detect_msg) {
                    _collector->add(_message.peer(), robot_msgs, robot_msg_times);
                    _collector->add(_message.peer(), obstacle_msgs, obstacle_msg_times);
                }
            }
        }
        _collector->robots(_robots, _robot_timestamps);
        _collector->obstacles(_obstacles, _obstacle_timestamps);
        for (uint k=0; k<_robots.size(); k++) {
            if (_robots[k]->id() == _id && _robots[k]->detected()) {
                cv::Point3f t = _robots[k]->translation();
                cv::Point3f r = _robots[k]->rotation();
                _detected_pose[0] = t.x;
                _detected_pose[1] = t.y;
                _detected_pose[2] = r.z;
                _detected_pose[3] = _robot_timestamps[k]*double(std::chrono::milliseconds::period::num) / std::chrono::milliseconds::period::den;
                _detected_pose_port.write(_detected_pose);
            }
        }
        // sort obstacles
        sort_obstacles(_obstacles);
    }
}

void EagleBridge::merge_obstacles(std::vector<eagle::Obstacle*>& obstacles) {
    std::vector<eagle::Obstacle*> similars;
    std::vector<eagle::Obstacle*> new_obstacles;
    for (uint i=0; i<obstacles.size(); i++) {
        similars.clear();
        similars.push_back(obstacles[i]);
        for (uint j=i+1; j<obstacles.size(); j++) {
            double dst = cv::norm(obstacles[i]->center() - obstacles[j]->center());
            if (dst < 0.1) {
                if (dynamic_cast<eagle::RectangleObstacle*>(obstacles[i]) != nullptr && dynamic_cast<eagle::RectangleObstacle*>(obstacles[j]) != nullptr) {
                    eagle::RectangleObstacle* obst_i = dynamic_cast<eagle::RectangleObstacle*>(obstacles[i]);
                    eagle::RectangleObstacle* obst_j = dynamic_cast<eagle::RectangleObstacle*>(obstacles[j]);
                    // same size detected as width?
                    if (fabs(obst_i->width() - obst_j->width())/fabs(obst_i->width()) < 0.01 && fabs(obst_i->height() - obst_j->height())/fabs(obst_i->height()) < 0.01) {
                        similars.push_back(obstacles[j]);
                    } else {
                        similars.push_back(new eagle::RectangleObstacle(cv::RotatedRect(obst_j->center(), cv::Size(obst_j->height(), obst_j->width()), 90 - obst_j->angle()*180/M_PI ))); // no idea if this is correct.
                    }
                }
                if (dynamic_cast<eagle::CircleObstacle*>(obstacles[i]) != nullptr && dynamic_cast<eagle::CircleObstacle*>(obstacles[j]) != nullptr) {
                    similars.push_back(obstacles[j]);
                }
            }
        }
        int n_sim = similars.size();
        if (n_sim > 1) {
            if (dynamic_cast<eagle::RectangleObstacle*>(obstacles[i]) != nullptr) {
                double x = 0, y = 0, w = 0, h = 0, t = 0;
                for (uint k=0; k<similars.size(); k++) {
                    x += (1/n_sim)*similars[k]->center().x;
                    y += (1/n_sim)*similars[k]->center().y;
                    w += (1/n_sim)*dynamic_cast<eagle::RectangleObstacle*>(similars[k])->width();
                    h += (1/n_sim)*dynamic_cast<eagle::RectangleObstacle*>(similars[k])->height();
                    t += (1/n_sim)*dynamic_cast<eagle::RectangleObstacle*>(similars[k])->angle();
                }
                new_obstacles.push_back(new eagle::RectangleObstacle(cv::RotatedRect(cv::Point2f(x, y), cv::Size(w, h), t*180/M_PI)));
            }
            if (dynamic_cast<eagle::CircleObstacle*>(obstacles[i]) != nullptr) {
                double x = 0, y = 0, r = 0;
                for (uint k=0; k<similars.size(); k++) {
                    x += (1/n_sim)*similars[k]->center().x;
                    y += (1/n_sim)*similars[k]->center().y;
                    r += (1/n_sim)*dynamic_cast<eagle::CircleObstacle*>(similars[k])->radius();
                }
                new_obstacles.push_back(new eagle::CircleObstacle(cv::Point2f(x, y), r));
            }
        } else {
            new_obstacles.push_back(obstacles[i]);
        }
    }
    obstacles = new_obstacles;
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
