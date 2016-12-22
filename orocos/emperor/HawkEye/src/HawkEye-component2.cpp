#include "HawkEye-component.hpp"
#include <unistd.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace RTT;
using namespace RTT::os;

HawkEye::HawkEye(std::string const& name) : TaskContext(name, PreOperational), _target_pose(3)
{
  // ports
  ports()->addPort("obstacle_port", _obstacle_port).doc("Detected obstacles");
  ports()->addPort("target_pose_port", _target_pose_port).doc("Target pose");

  // properties
  addProperty("robot_names", _robot_names).doc("Name of robots to detect");
  addProperty("top_markers", _top_markers_path).doc("Path of top marker for each robot");
  addProperty("bottom_marker", _bottom_marker_path).doc("Path of bottom marker used by each robot");
  addProperty("top_marker_table", _top_marker_table).doc("Table mapping top marker index on robot index");
  addProperty("marker_locations", _marker_locations).doc("Locations of markers in robot frame [x1_r1, y1_r1, x2_r1, y2_r1, x3_r1, y3_r1, x1_r2, ...] (m)");
  addProperty("robot_sizes", _robot_sizes).doc("Width and height of outer rectangle of each robot [w_r1, h_r1, w_r2, ...] (m)");
  addProperty("robot_colors", _robot_colors).doc("RGB colors of each robot [r_r1, g_r1, b_r1, r_r2, ...]");
  addProperty("video_port_name", _video_port_name).doc("Path to video driver");
  addProperty("brightness", _brightness).doc("Camera brightness");
  addProperty("exposure", _exposure).doc("Camera exposure");
  addProperty("iso", _iso).doc("Camera iso");
  addProperty("camera_cfs", _camera_cfs).doc("Coefficients of camera matrix (fx, cx, fy, cy)");
  addProperty("distortion_cfs", _distortion_cfs).doc("Coefficients of distortion vector (k1, k2, p1, p2, k3)");
  addProperty("max_detectable_obstacles", _max_detectable_obstacles).doc("Maximum number of detectable obstacles");
  addProperty("image_path", _image_path).doc("Path where images are stored");
  addProperty("gui_resolution", _gui_resolution).doc("Width and height of gui window");
  addProperty("threshold_bgst", _threshold_bgst).doc("Threshold for background subtraction [0-255]");
  addProperty("threshold_match", _threshold_match).doc("Threshold for template matching [0-1]");
  addProperty("min_robot_area", _min_robot_area).doc("Minimum robot area to detect (m2)");
  addProperty("min_obstacle_area", _min_obstacle_area).doc("Minimum obstacle area to detect (m2)");
  addProperty("pixelspermeter", _pixelspermeter).doc("Pixels per meter");
  addProperty("capture_bg_at_start", _capture_bg_at_start).doc("Capture background at start");
  addProperty("number_of_bg_samples", _number_of_bg_samples).doc("Number of samples taken to determine the background");
  addProperty("show_prev_frame", _show_prev_frame).doc("Show previous frame (conform with the current received estimation)");
  addProperty("capture_time_mod", _capture_time_mod).doc("At what fracture of total frame capture procedure does the capture happens");
  addProperty("save_video", _save_video).doc("Save the resulting video");
  addProperty("hawkeye_sample_rate", _hawkeye_sample_rate).doc("Sample rate of hawkeye");

  addPropert("marker_params", _marker_params).doc("Parameters that define marker");

  // operations
  addOperation("captureBackground", &HawkEye::captureBackground, this);

}


void HawkEye::saveMarkerParams(){
  _mp["nbit_w"] = _marker_params[0];
  _mp["nbit_h"] = _marker_params[1];
}

bool HawkEye::configureHook(){
  // create robots and ports
  createRobots();
  // show example data sample to output ports to make data flow real-time
  std::vector<double> example_markers(7, 0.0);
  std::vector<double> example_obstacles(5*_max_detectable_obstacles, 0.0);
  std::vector<double> example_pose(3, 0.0);
  _obstacle_port.setDataSample(example_obstacles);
  _target_pose_port.setDataSample(example_pose);
  for (uint k=0; k<_robot_markers_port.size(); k++){
    _robot_markers_port[k]->setDataSample(example_markers);
  }
  // create gui, camera and robots
  _gui = new Gui(_gui_resolution, _save_video, _hawkeye_sample_rate);
  _gui->setPixelsPerMeter(_pixelspermeter);
  // _camera = new Camera(_video_port_name, {1920, 1080}, _brightness, _exposure, _iso, _camera_cfs, _distortion_cfs, _capture_time_mod);
  _camera = new Camera(_video_port_name, {1280, 720}, _brightness, _exposure, _iso, _camera_cfs, _distortion_cfs, _capture_time_mod);
  _detector = new Detector(_marker_params);
  // init blob detector
  initDetector();
  return true;
}

bool HawkEye::startHook(){
  // start gui
  if (!_gui->start()){
    return false;
  }
  // start camera
  if (!_camera->start(false)){
    return false;
  }
  // get background
  cv::Mat background
  if (!getBackground(background)){
    return false;
  }
  // start detector
  if (!_detector->start(background)){
    return false;
  }
  // put something useful in _prev_frame
  if (!_camera->capture(_prev_frame, _capture_time)){
    log(Error) << "Could not capture frame!" << endlog();
  }
  return true;
}

void HawkEye::updateHook(){
  #ifdef DEBUG
  TimeService::ticks _timestamp = TimeService::Instance()->getTicks();
  #endif
  // capture frame
  if (!_camera->capture(_frame, _capture_time)){
    log(Error) << "Could not capture frame!" << endlog();
  }
  // reset previous stuff
  reset();
  // process frame
  _detector->update(_frame, _robots, );
  // write results
  writeResults();
  // draw
  if (_show_prev_frame){
    drawResults(_prev_frame);

  } else {
    drawResults(_frame);
  }
  // save frame
  _frame.copyTo(_prev_frame);
  // set target pose
  setTargetPose();
  #ifdef DEBUG
  Seconds time_elapsed = TimeService::Instance()->secondsSince(_timestamp);
  #endif
  DEBUG_PRINT("update took " << time_elapsed << "s")
}

void HawkEye::stopHook(){
  _gui->stop();
  _camera->stop();
}

void HawkEye::createRobots(){
  int n_robots = int(_mp["nbit_w"]*_mp["nbit_h"]);
  _robot_markers_port.resize(n_robots);
  _robot_est_pose_port.resize(n_robots);
  _robot_ref_x_port.resize(n_robots);
  _robot_ref_y_port.resize(n_robots);
  for (uint k=0; k<n_robots.size(); k++){
    _robot_markers_port[k] = new OutputPort<std::vector<double> >();
    ports()->addPort("robot" + k + "_pose_port", *_robot_markers_port[k]).doc("Detected markers of " + _robot_names[k]);
    _robot_est_pose_port[k] = new InputPort<std::vector<double> >();
    ports()->addPort("robot" + k + "_est_pose_port", *_robot_est_pose_port[k]).doc("Estimated pose of " +  _robot_names[k]);
    _robot_ref_x_port[k] = new InputPort<std::vector<double> >();
    ports()->addPort("robot" + k + "_ref_x_port", *_robot_ref_x_port[k]).doc("Reference x trajectory of " + _robot_names[k]);
    _robot_ref_y_port[k] = new InputPort<std::vector<double> >();
    ports()->addPort("robot" + k + "_ref_y_port", *_robot_ref_y_port[k]).doc("Reference y trajectory of " + _robot_names[k]);
  }
  _robots.resize(n_robots);
  for (uint k=0; k<_robots.size(); k++){
    std::vector<cv::Mat> top_markers;
    for (uint i=0; i<_top_marker_table.size(); i++){
      if (_top_marker_table[i] == k){
        top_markers.push_back(_top_markers[i]);
      }
    }
    std::vector<double> marker_loc(6);
    for (int i=0; i<6; i++){
      marker_loc[i] = _marker_locations[6*k+i];
    }
    _robots[k] = new Robot(_robot_names[k], top_markers, marker_loc, _pixelspermeter*_robot_sizes[2*k], _pixelspermeter*_robot_sizes[2*k+1]);
  }
}

bool HawkEye::getBackground(cv::Mat& background){
  if (!_capture_bg_at_start){
    if (!loadBackground(background)){
      log(Error) << "Could not load background!" << endlog();
      return false;
    }
  } else {
    if (!captureBackground(background)){
      log(Error) << "Could not capture background!" << endlog();
      return false;
    }
  }
  return true;
}

bool HawkEye::loadBackground(cv::Mat& background){
  background = cv::imread(_image_path + "background.png", CV_LOAD_IMAGE_COLOR);
  if (!_background.data){
    log(Error) << "Could not open " << _image_path << "background.png" << endlog();
    return false;
  }
  return true;
}

bool HawkEye::captureBackground(cv::Mat& background){
  std::cout << "Capturing background ... " << std::endl;
  if (!_camera->capture(_frame, _capture_time)){
    log(Error) << "Could not capture frame!" << endlog();
    return false;
  }
  background = cv::Mat(_frame.size(), CV_32FC3, cv::Scalar(0,0,0)); // black
  for (int i=0; i<_number_of_bg_samples; i++){
    if (!_camera->capture(_frame, _capture_time)){
      log(Error) << "Could not capture frame!" << endlog();
      return false;
    }
    cv::accumulate(_frame, background);
  }
  background /= _number_of_bg_samples;
  background.convertTo(background, CV_8UC3);
  cv::imwrite(_image_path + "background.png", background);
  std::cout << "done." << std::endl;
  return true;
}

void HawkEye::reset(){
  for (uint k=0; k<_robots.size(); k++){
    _robots[k]->reset();
  }
  _obstacles.clear();
}

void HawkEye::writeResults(){
  // robots
  std::vector<double> marker_vector(7);
  for (uint k=0; k<_robots.size(); k++){
    if (_robots[k]->detected()){
      _robots[k]->getMarkers(marker_vector, _pixelspermeter, _frame.size().height);
      marker_vector[6] = _capture_time;
      _robot_markers_port[k]->write(marker_vector);
    }
  }
  // obstacles
  _obstacle_port.write(_obstacles);
}

void HawkEye::drawResults(cv::Mat& frame){
  for (uint k=0; k<_robots.size(); k++){
    if (_robots[k]->detected()){
      std::vector<double> pose(3, 0.0);
      _robot_est_pose_port[k]->read(pose);
      _robots[k]->setPose(pose);
      std::vector<double> ref_x, ref_y;
      if (_robot_ref_x_port[k]->read(ref_x) == RTT::NewData && _robot_ref_y_port[k]->read(ref_y) == RTT::NewData){
        _robots[k]->setRef(ref_x, ref_y);
      }
    }
  }
  _gui->draw(frame, _obstacles, _robots, _robot_colors);
}

void HawkEye::setTargetPose(){
  std::vector<double> target_pose(3);
  _gui->getClickPose(target_pose);
  bool send = false;
  for (int i=0; i<3; i++){
    if (target_pose[i] != _target_pose[i]){
      send = true;
      break;
    }
  }
  if (send){
    for (int i=0; i<3; i++){
      if (target_pose[i] < 0.){
        send = false;
        break;
      }
    }
  }
  if (send){
    _target_pose_port.write(target_pose);
    _target_pose = target_pose;
  }
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 & ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HawkEye)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(HawkEye)
