#include "Camera-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

void mouseCallBack(int event, int x, int y, int flags, void* data){
    Camera* cam = (Camera*)data;
    if (event == cv::EVENT_LBUTTONDOWN){
        cam->set_mouse_click(x, y);
    }
}

Camera::Camera(std::string const& name) : TaskContext(name){
    // ports
    ports()->addPort("target_out_port", _target_out_port).doc("Target output port");
    ports()->addPort("target_in_port", _target_in_port).doc("Target input port");
    // properties
    addProperty("local_detecting", _local_detecting).doc("Are we detecting on this device?");
    addProperty("capture_bg_at_start", _capture_bg_at_start).doc("Capture background at start");
    addProperty("node_name", _node_name).doc("Name of zyre node for Camera component");
    addProperty("communication_group", _communication_group).doc("Zyre group to communicate eagle messages");
    addProperty("eagle_config_path", _eagle_config_path).doc("Path to configure project eagle components");
    addProperty("image_size", _image_size).doc("Image size [w x h] in m");
    addProperty("pixels_per_meter", _pixels_per_meter).doc("Number of pixels displayed in a meter");
    addProperty("image_stream", _image_stream).doc("Setting up image stream with remote eagles?");
    addProperty("frame_position", _frame_position).doc("Position of frame in displayed image in m");
    addProperty("save_movie", _save_movie).doc("Save movie?");
    addProperty("eagles", _eagles).doc("Name of eagle nodes to listen to");
    addProperty("verbose", _verbose).doc("Verbose flag");
    addProperty("draw", _draw).doc("Amount of drawing");
    addProperty("robot_ids", _robot_ids).doc("Id's of robots to detect");
    addProperty("robot_sizes", _robot_sizes).doc("Sizes of robots to detect");
    addProperty("robot_marker_translation", _robot_marker_translation).doc("Translation of marker wrt to robot frame");
    addProperty("robot_colors", _robot_colors).doc("Colors of robots to detect");
    // operations
    addOperation("capture_background", &Camera::capture_background, this);
}

bool Camera::configureHook(){

    _camera = eagle::getCamera(_eagle_config_path);
    _detector = new eagle::Detector(_eagle_config_path);
    _communicator = new eagle::Communicator(_node_name, _eagle_config_path);
    _communicator->verbose(0);
    _communicator->start(100);
    _communicator->join(_communication_group);
    init_robots();
    if (_local_detecting) {
        _projection = _detector->projection();
    } else {
        _image = cv::Mat(cv::Size2f(_image_size[0]*_pixels_per_meter, _image_size[1]*_pixels_per_meter), CV_8UC3);
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << _pixels_per_meter, 0, _frame_position[0]*_pixels_per_meter, 0, _pixels_per_meter, _frame_position[1]*_pixels_per_meter, 0, 0, 1);
        cv::Mat distortion_vector = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
        cv::Mat T = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 1., 0, 0, 0, 1);
        _projection = new eagle::Projection(camera_matrix, distortion_vector, _image, T);
        _collector = new eagle::Collector();
    }
    return true;
}

bool Camera::startHook(){
    if (_local_detecting) {
        _camera->start();
        _camera->read(_image);
    } else {
        stream(_image_stream);
        _image = cv::Mat(cv::Size2f(_image_size[0]*_pixels_per_meter, _image_size[1]*_pixels_per_meter), CV_8UC3);
    }
    if (!load_background()) {
        return false;
    }
    if (_save_movie){
        _movie.open("movie.avi", CV_FOURCC('M','J','P','G'), 1./getPeriod(), _image.size(), true);
    }
    cvStartWindowThread();
    cv::namedWindow("Image", 1);
    cv::setMouseCallback("Image", mouseCallBack, (void*)this);
    return true;
}

void Camera::updateHook(){
    if (_local_detecting) {
        // read new camera image
        if (!_camera->read(_image)) {
            log(Error) << "Could not read camera!" << endlog();
            error();
        }
        _capture_time = eagle::timestamp();
        // detect robots & obstacles
        _detector->search(_image, _robots, _obstacles);
        // transmit results
        transmit_detected();
    } else {
        if (!_image_stream) {
            _image = cv::Mat(cv::Size2f(_image_size[0]*_pixels_per_meter, _image_size[1]*_pixels_per_meter), CV_8UC3);
            _image.setTo(cv::Scalar(184, 195, 209));
        }
        receive_detected();
    }
    print(_verbose);
    // draw results
    if (_local_detecting || !_image_stream) {
        _image = draw();
    }
    cv::imshow("Image", _image);
    cv::waitKey(25);
    if (_save_movie) {
        _movie.write(_image);
    }
}

void Camera::cleanupHook() {
    cvDestroyAllWindows();
    _camera->stop();
    _communicator->stop();
}

bool Camera::load_background() {
    if (_capture_bg_at_start) {
        capture_background();
    } else if (_local_detecting) {
        cv::Mat background = cv::imread("background.png", CV_LOAD_IMAGE_COLOR);
        if (!background.data) {
            log(Error) << "Could not open background!" << endlog();
            return false;
        }
    }
    return true;
}

bool Camera::capture_background() {
    if (_local_detecting) {
        for (int i=0; i<10; i++) {
            if (!_camera->read(_image)) {
                log(Error) << "Could not read camera!" << endlog();
                return false;
            }
            cv::waitKey(1); // give camera time to reload
        }
        cv::Mat background = cv::Mat(_image.size(), CV_32FC3, cv::Scalar(0, 0, 0));
        for (int i=0; i<50; i++) {
            if (!_camera->read(_image)) {
                log(Error) << "Could not read camera!" << endlog();
                return false;
            }
            cv::accumulate(_image, background);
        }
        background /= 50;
        background.convertTo(background, CV_8UC3);
        cv::imwrite("background.png", background);
        _detector->set_background(background);
        std::cout << "done." << std::endl;
    } else {
        if (!shout_command(eagle::BACKGROUND)) {
            std::cout << "Sending background capture not succesful." << std::endl;
            return false;
        }
        std::cout << "Background capture command sent." << std::endl;
    }
    return true;
}

void Camera::init_robots() {
    _robots.resize(_robot_ids.size());
    _robot_est_pose_port.resize(_robot_ids.size());
    _robot_ref_x_port.resize(_robot_ids.size());
    _robot_ref_y_port.resize(_robot_ids.size());
    for (uint k=0; k<_robots.size(); k++) {
        cv::Scalar color(_robot_colors[3*k], _robot_colors[3*k+1], _robot_colors[3*k+2]);
        _robots[k] = new eagle::Robot(_robot_ids[k], _robot_sizes[2*k], _robot_sizes[2*k+1],
            cv::Point3f(_robot_marker_translation[2*k], _robot_marker_translation[2*k+1], 0), color);
        // create ports
        _robot_est_pose_port[k] = new InputPort<std::vector<double> >();
        ports()->addPort("robot" + std::to_string(k) + "_est_pose_port", *_robot_est_pose_port[k]).doc("Estimated pose of robot " + std::to_string(k));
        _robot_ref_x_port[k] = new InputPort<std::vector<double> >();
        ports()->addPort("robot" + std::to_string(k) + "_ref_x_port", *_robot_ref_x_port[k]).doc("Reference x trajectory of robot " + std::to_string(k));
        _robot_ref_y_port[k] = new InputPort<std::vector<double> >();
        ports()->addPort("robot" + std::to_string(k) + "_ref_y_port", *_robot_ref_y_port[k]).doc("Reference y trajectory of robot " + std::to_string(k));
    }
}

bool Camera::stream(bool enable) {
    if (enable) {
        if (!shout_command(eagle::IMAGE_STREAM_ON)) {
            std::cout << "Sending image_stream_on command not succesful." << std::endl;
            return false;
        }
    }
    // } else {
    //     if (!shout_command(eagle::IMAGE_STREAM_OFF)) {
    //         std::cout << "Sending image_stream_off command not succesful." << std::endl;
    //         return false;
    //     }
    // }
    return true;
}

bool Camera::shout_command(eagle::cmd_t cmd) {
    eagle::header_t header = {eagle::CMD, 0}; //set time
    return _communicator->shout(&header, &cmd, sizeof(header), sizeof(cmd), _communication_group);
}

void Camera::receive_detected() {
    if (_communicator->receive()) {
        // handle received messages
        while (_communicator->pop_message(_message)) {
            if (std::find(_eagles.begin(), _eagles.end(), _message.peer()) != _eagles.end()) {
                std::vector<eagle::marker_t> robot_msgs;
                std::vector<uint32_t> robot_msg_times;
                std::vector<eagle::obstacle_t> obstacle_msgs;
                std::vector<uint32_t> obstacle_msg_times;
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
                        case eagle::IMAGE: {
                            if (_image_stream) {
                                size_t size = _message.framesize();
                                uchar buffer[size];
                                _message.read(buffer);
                                cv::Mat raw = cv::Mat(1, size, CV_8UC1, buffer);
                                _image = cv::imdecode(raw, 1);
                            } else {
                                _message.dump_frame();
                                break;
                            }
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
        _collector->get(_robots, _obstacles, _robot_timestamps, _obstacle_timestamps);
    }
}

void Camera::transmit_detected() {
    // pack robots
    size_t n_robots = 0;
    eagle::header_t mheaders[_robots.size()];
    eagle::marker_t markers[_robots.size()];
    for (uint k=0; k<_robots.size(); k++) {
        if (_robots[k]->detected()) {
            mheaders[n_robots].id = eagle::MARKER;
            mheaders[n_robots].time = _capture_time;
            markers[n_robots] = _robots[k]->serialize();
            n_robots++;
        }
    }
    // pack obstacles
    eagle::header_t oheaders[_obstacles.size()];
    eagle::obstacle_t obstacles[_obstacles.size()];
    for (uint k=0; k<_obstacles.size(); k++) {
        oheaders[k].id = eagle::OBSTACLE;
        oheaders[k].time = _capture_time;
        obstacles[k] = _obstacles[k]->serialize();
    }
    // make data and size vector
    std::vector<size_t> sizes = std::vector<size_t>(2*n_robots + 2*_obstacles.size(), 0);
    std::vector<const void*> data = std::vector< const void* >(sizes.size());
    int i=0;
    for (uint k=0; k<n_robots; k++) {
        sizes[i] = sizeof(eagle::header_t);
        data[i++] = (const void*)(&mheaders[k]);
        sizes[i] = sizeof(eagle::marker_t);
        data[i++] = (const void*)(&markers[k]);
    }
    for (uint k=0; k<_obstacles.size(); k++) {
        sizes[i] = sizeof(eagle::header_t);
        data[i++] = (const void*)(&oheaders[k]);
        sizes[i] = sizeof(eagle::obstacle_t);
        data[i++] = (const void*)(&obstacles[k]);
    }
    // send everything
    _communicator->shout(data, sizes, _communication_group);
}

void Camera::print(int verbose) {
    if (verbose >= 1) {
        std::cout << "* Robots detected: " << std::endl;
        for (uint k=0; k<_robots.size(); k++) {
            if (_robots[k]->detected()) {
                std::cout << "\t- " << _robots[k]->to_string() << std::endl;
            }
        }
        std::cout << "* Obstacles detected: " << std::endl;
        for (uint k=0; k<_obstacles.size(); k++) {
            std::cout << "\t- " << _obstacles[k]->to_string() << std::endl;
        }
    }
}

cv::Mat Camera::draw() {
    cv::Mat img;
    _projection->remap(_image, img);
    double robot_height = 0.07;
    cv::Scalar gray(77, 76, 75);
    if (_draw >= 2) {
        // coordinate system
        cv::Point2f O = _projection->project_to_image(cv::Point3f(0, 0, 0));
        cv::Point2f Ex = _projection->project_to_image(cv::Point3f(1, 0, 0));
        cv::Point2f Ey = _projection->project_to_image(cv::Point3f(0, 1, 0));
        cv::circle(img, O, 5, gray, -2);
        cv::arrowedLine(img, O, Ex, gray, 2); //Ex vector
        cv::arrowedLine(img, O, Ey, gray, 2); //Ey vector
        cv::putText(img, "x", _projection->project_to_image(cv::Point3f(1.1, 0, 0)), cv::FONT_HERSHEY_SIMPLEX, 1, gray, 2);
        cv::putText(img, "y", _projection->project_to_image(cv::Point3f(0, 1.1, 0)), cv::FONT_HERSHEY_SIMPLEX, 1, gray, 2);
    }
    if (_draw >= 3) {
        // grid
        cv::Point2f o1;
        for (int k=0; k<5; k++) {
            o1 = _projection->project_to_image(cv::Point3f(k, 0, 0));
            cv::line(img, cv::Point2f(o1.x, 0), cv::Point2f(o1.x, img.size().height), gray, 1);
            if (k != 0) {
                o1 = _projection->project_to_image(cv::Point3f(-k, 0, 0));
                cv::line(img, cv::Point2f(o1.x, 0), cv::Point2f(o1.x, img.size().height), gray, 1);
            }
            o1 = _projection->project_to_image(cv::Point3f(0, k, 0));
            cv::line(img, cv::Point2f(0, o1.y), cv::Point2f(img.size().width, o1.y), gray, 1);
            if (k != 0) {
                o1 = _projection->project_to_image(cv::Point3f(0, -k, 0));
                cv::line(img, cv::Point2f(0, o1.y), cv::Point2f(img.size().width, o1.y), gray, 1);
            }
        }
    }
    if (_draw >= 4) {
        // obstacles
        for (uint k=0; k<_obstacles.size(); k++) {
            _obstacles[k]->draw(img, *_projection);
        }
    }
    if (_draw >= 1) {
        // robots
        for (uint k=0; k<_robots.size(); k++) {
            if (_robots[k]->detected()) {
                _robots[k]->draw(img, *_projection);
                // estimated pose
                std::vector<double> pose({-100, -100, -100});
                _robot_est_pose_port[k]->read(pose);
                cv::Point2f center = _projection->project_to_image(cv::Point3f(pose[0], pose[1], robot_height));
                cv::Point2f heading = _projection->project_to_image(cv::Point3f(pose[0]+0.1*cos(pose[2]), pose[1]+0.1*sin(pose[2]), robot_height));
                cv::circle(img, center, 5, _robots[k]->color(), -2);
                cv::line(img, center, heading, _robots[k]->color(), 2);
                // reference trajectory
                std::vector<double> ref_x, ref_y;
                cv::Scalar color_w = mix_with_white(_robots[k]->color(), 50);
                _robot_ref_x_port[k]->read(ref_x);
                _robot_ref_y_port[k]->read(ref_y);
                std::vector<cv::Point2f> points;
                for (uint i=0; i<ref_x.size(); i++) {
                    if (ref_x[i] != 0 && ref_y[i] != 0) {
                        points.push_back(_projection->project_to_image(cv::Point3f(ref_x[i], ref_y[i], robot_height)));
                    }
                }
                if (points.size() > 0) {
                    for (uint i=0; i<points.size()-1; i++) {
                        cv::line(img, points[i], points[i+1], color_w, 2);
                    }
                }
            }
        }
    }
    if (_draw >= 1) {
        // target
        std::vector<double> target_pose;
        _target_in_port.read(target_pose);
        if (target_pose.size() >= 2) {
            cv::Point2f target = _projection->project_to_image(cv::Point3f(target_pose[0], target_pose[1], robot_height));
            cv::circle(img, target, 30, gray, 2);
            cv::circle(img, target, 10, gray, -1.5);
        }
    }
    return img;
}

void Camera::set_mouse_click(int x, int y) {
    cv::Point2f point_px(x, y);
    cv::Mat ground_plane = (cv::Mat_<float>(1, 4) << 0, 0, 1, 0);
    cv::Point2f point = eagle::dropz(_projection->project_to_plane(point_px, ground_plane));
    _target_out_port.write(std::vector<double>({point.x, point.y}));
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Camera)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Camera)
