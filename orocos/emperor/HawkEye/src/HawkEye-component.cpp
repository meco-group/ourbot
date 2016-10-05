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

  // operations
  addOperation("captureBackground", &HawkEye::captureBackground, this);

}

bool HawkEye::configureHook(){
  // load marker templates
  if (!loadMarkers()){
      return false;
  }
  // add ports
  _robot_markers_port.resize(_robot_names.size());
  _robot_est_pose_port.resize(_robot_names.size());
  for (uint k=0; k<_robot_names.size(); k++){
    _robot_markers_port[k] = new OutputPort<std::vector<double> >();
    ports()->addPort(_robot_names[k]+"_pose_port", *_robot_markers_port[k]).doc("Detected markers of " + _robot_names[k]);
    _robot_est_pose_port[k] = new InputPort<std::vector<double> >();
    ports()->addPort(_robot_names[k]+"_est_pose_port", *_robot_est_pose_port[k]).doc("Estimated pose of " +  _robot_names[k]);
  }
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
  _gui = new Gui(_gui_resolution);
  _gui->setPixelsPerMeter(_pixelspermeter);
  _camera = new Camera(_video_port_name, {1920, 1080}, _brightness, _exposure, _iso, _camera_cfs, _distortion_cfs, _capture_time_mod);
  createRobots();
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
  if (!getBackground()){
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
  processFrame(_frame);
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

bool HawkEye::loadMarkers(){
  _top_markers.resize(_top_markers_path.size());
  for (uint k=0; k<_top_markers_path.size(); k++){
      _top_markers[k] = cv::imread(_image_path + "markers/" + _top_markers_path[k], CV_LOAD_IMAGE_GRAYSCALE);
      if (!_top_markers[k].data){
          log(Error) << "Could not open top marker " << _top_markers_path[k] << "!" << endlog();
          return false;
      }
  }
  _bottom_marker = cv::imread(_image_path + "markers/" + _bottom_marker_path, CV_LOAD_IMAGE_GRAYSCALE);
  if (!_bottom_marker.data){
      log(Error) << "Could not open bottom marker " << _bottom_marker_path << "!" << endlog();
      return false;
  }
  return true;
}

void HawkEye::createRobots(){
  _robots.resize(_robot_names.size());
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

bool HawkEye::getBackground(){
  if (!_capture_bg_at_start){
    if (!loadBackground()){
      log(Error) << "Could not load background!" << endlog();
      return false;
    }
  } else {
    if (!captureBackground()){
      log(Error) << "Could not capture background!" << endlog();
      return false;
    }
  }
  return true;
}

bool HawkEye::loadBackground(){
  _background = cv::imread(_image_path + "background.png", CV_LOAD_IMAGE_COLOR);
  if (!_background.data){
    log(Error) << "Could not open " << _image_path << "background.png" << endlog();
    return false;
  }
  return true;
}

bool HawkEye::captureBackground(){
  std::cout << "Capturing background ... " << std::endl;
  if (!_camera->capture(_frame, _capture_time)){
    log(Error) << "Could not capture frame!" << endlog();
    return false;
  }
  _background = cv::Mat(_frame.size(), CV_32FC3, cv::Scalar(0,0,0)); // black
  for (int i=0; i<_number_of_bg_samples; i++){
    if (!_camera->capture(_frame, _capture_time)){
      log(Error) << "Could not capture frame!" << endlog();
      return false;
    }
    cv::accumulate(_frame, _background);
  }
  _background /= _number_of_bg_samples;
  _background.convertTo(_background, CV_8UC3);
  cv::imwrite(_image_path + "background.png", _background);
  std::cout << "done." << std::endl;
  return true;
}

void HawkEye::processFrame(const cv::Mat& frame){
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  // background subtraction
  backgroundSubtraction(frame, contours, hierarchy);
  if (hierarchy.empty()){
    return; // no contours were found
  }
  detectRobots(frame, contours);
  subtractRobots(contours, hierarchy);
  if (hierarchy.empty()){
    return; // no contours except from robots were found
  }
  detectObstacles(frame, contours);
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

void HawkEye::backgroundSubtraction(const cv::Mat& frame, std::vector<std::vector<cv::Point> >& contours,
  std::vector<cv::Vec4i>& hierarchy){
  cv::Mat mask_copy;
  cv::absdiff(_background, frame, _mask);
  cv::cvtColor(_mask, _mask, CV_RGB2GRAY);
  cv::threshold(_mask, _mask, _threshold_bgst, 255, cv::THRESH_BINARY);
  _mask.copyTo(mask_copy);
  cv::findContours(mask_copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

void HawkEye::detectRobots(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours){
  std::vector<cv::Point> contour;
  cv::Rect roi_rectangle; //region-of-interest rectangle
  std::vector<int> roi_location(2);
  cv::Mat roi;
  std::vector<int> marker_loc_rob(6);
  for (uint i=0; i<contours.size(); i++){
    cv::convexHull(contours[i], contour);
    if (cv::contourArea(contour) > _min_robot_area*pow(_pixelspermeter, 2)){
      std::vector<int> marker_locations;
      std::vector<int> robot_indices;
      roi_rectangle = cv::boundingRect(contour);
      roi_location[0] = roi_rectangle.x;
      roi_location[1] = roi_rectangle.y;
      frame(roi_rectangle).copyTo(roi);
      if (matchMarkers(roi, roi_location, marker_locations, robot_indices)){
        for (uint k=0; k<robot_indices.size(); k++){
          DEBUG_PRINT("Detected " << _robots[robot_indices[k]]->getName() << "!")
          for (uint j=0; j<6; j++){
            marker_loc_rob[j] = marker_locations[6*k+j];
          }
          _robots[robot_indices[k]]->setMarkers(marker_loc_rob);
        }
      }
    }
  }
}

void HawkEye::subtractRobots(std::vector<std::vector<cv::Point> >& contours,
  std::vector<cv::Vec4i>& hierarchy){
  cv::Point2f robot_vertices[4];
  std::vector<cv::Point> robot_contour(4);
  std::vector<std::vector<cv::Point> > robot_contour_vector;
  for (uint k=0; k<_robots.size(); k++){
    if (_robots[k]->detected()){
      _robots[k]->getBox().points(robot_vertices);
      for (int i=0; i<4; i++){
        robot_contour[i] = robot_vertices[i];
      }
      robot_contour_vector.clear();
      robot_contour_vector.push_back(robot_contour);
      // remove robot box from mask
      cv::drawContours(_mask, robot_contour_vector, 0, cv::Scalar(0,0,0), -1);
    }
  }
  // find new contours
  cv::findContours(_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

void HawkEye::detectObstacles(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours){
  std::vector<cv::Point> contour;
  std::vector<cv::RotatedRect> rectangles;
  std::vector<std::vector<double> > circles;
  cv::Point2f center;
  float radius;
  for (uint i=0; i<contours.size(); i++){
    cv::convexHull(contours[i], contour);
    if (cv::contourArea(contour) > _min_obstacle_area*pow(_pixelspermeter, 2)){
      rectangles.push_back(cv::minAreaRect(contour));
      cv::minEnclosingCircle(contour, center, radius);
      std::vector<double> circle = {center.x, center.y, radius};
      circles.push_back(circle);
    }
  }
  // filter obstacles
  filterObstacles(rectangles, circles);
}

void HawkEye::filterObstacles(const std::vector<cv::RotatedRect>& rectangles, const std::vector<std::vector<double> >& circles){
  bool add;
  cv::Point2f vertices[4];
  std::vector<cv::Point> other_contour(4);
  std::vector<std::vector<double> > obstacles;
  std::vector<double> areas;
  for (uint i=0; i<rectangles.size(); i++){
    add = true;
    for (uint k=0; k<_robots.size(); k++){
      if (_robots[k]->detected()){
        _robots[k]->getBox().points(vertices);
        for (int l=0; l<4; l++){
          other_contour[l] = vertices[l];
        }
        if (cv::pointPolygonTest(other_contour, rectangles[i].center, false) > 0){
          add = false; // obstacle is within robot
        }
      }
    }
    for (uint k=0; k<rectangles.size(); k++){
      if (k!=i){
        rectangles[k].points(vertices);
        for (int l=0; l<4; l++){
          other_contour[l] = vertices[l];
        }
        if (cv::pointPolygonTest(other_contour, rectangles[i].center, false) > 0){
          add = false; // obstacle is within another obstacle
        }
      }
    }
    if (add){
      addObstacle(rectangles[i], circles[i], obstacles, areas);
    }
  }

  // sort obstacles on area size
  sortObstacles(obstacles, areas);

  // keep largest obstacles
  if (obstacles.size() > _max_detectable_obstacles){
    log(Warning) << "More obstacles detected than allowed!" << endlog();
    for (int k=0; k<_max_detectable_obstacles; k++){
      for (int i=0; i<5; i++){
        _obstacles.push_back(obstacles[k][i]);
      }
    }
  } else {
    for (int k=0; k<obstacles.size(); k++){
      for (int i=0; i<5; i++){
        _obstacles.push_back(obstacles[k][i]);
      }
    }
    for (int k=obstacles.size(); k<_max_detectable_obstacles; k++){
      for (int i=0; i<5; i++){
        _obstacles.push_back(-100);
      }
    }
  }
}

void HawkEye::addObstacle(const cv::RotatedRect& rectangle, const std::vector<double>& circle,
  std::vector<std::vector<double>>& obstacles, std::vector<double>& areas){
  double circle_area = M_PI*circle[2]*circle[2];
  double rectangle_area = rectangle.size.width * rectangle.size.height;
  std::vector<double> obstacle(5);
  if (rectangle_area < circle_area){
    obstacle[0] = rectangle.center.x*1.0/_pixelspermeter;
    obstacle[1] = (_frame.size().height-rectangle.center.y)*1.0/_pixelspermeter;
    obstacle[2] = -rectangle.angle;
    obstacle[3] = rectangle.size.width*1.0/_pixelspermeter;
    obstacle[4] = rectangle.size.height*1.0/_pixelspermeter;
    areas.push_back(rectangle_area);
  } else {
    obstacle[0] = circle[0]*1.0/_pixelspermeter;
    obstacle[1] = (_frame.size().height-circle[1])*1.0/_pixelspermeter;
    obstacle[2] = circle[2]*1.0/_pixelspermeter;
    obstacle[3] = -100;
    obstacle[4] = -100;
    areas.push_back(circle_area);
  }
  obstacles.push_back(obstacle);
}

void HawkEye::sortObstacles(std::vector<std::vector<double> >& obstacles, std::vector<double>& areas){
  int n = areas.size();
  std::vector<int> indices(n);
  for (int i=0; i<n; i++){
    indices[i] = i;
  }
  int j, index;
  double area;
  // basic insertion sort
  for (int i=1; i<n; i++){
    j = i;
    while (j > 0 && areas[j-1] < areas[j]){
      // swap area
      area = areas[j];
      areas[j] = areas[j-1];
      areas[j-1] = area;
      // swap index
      index = indices[j];
      indices[j] = indices[j-1];
      indices[j-1] = index;
      j--;
    }
  }
  std::vector<std::vector<double> > obstacles_out(n);
  for (int i=0; i<n; i++){
    obstacles_out[i] = obstacles[indices[i]];
  }
  obstacles = obstacles_out;
}

bool HawkEye::matchMarkers(cv::Mat& roi, const std::vector<int>& roi_location, std::vector<int>& marker_locations, std::vector<int>& robot_indices){
  cv::cvtColor(roi, roi, CV_RGB2GRAY);
  std::vector<int> marker_loc;
  std::vector<double> marker_scores;
  // search bottom markers
  matchTemplates(roi, _bottom_marker, _threshold_match, marker_loc, marker_scores);
  if (marker_loc.size() < 4){
    return false; // no bottom markers detected
  }
  int number_of_robots = marker_loc.size()/4; // number of robots in roi
  robot_indices.resize(number_of_robots);
  marker_locations.resize(6*number_of_robots, 0);
  std::vector<int> bottom_marker_locations(4*number_of_robots, 0);
  for (int i=0; i<marker_loc.size(); i++){
    bottom_marker_locations[i] = marker_loc[i] + roi_location[i%2];
    if (i%2 == 0){
      // blank-out detected markers
      blank(roi, cv::Point(marker_loc[i], marker_loc[i+1]), _bottom_marker.size().width, _bottom_marker.size().height);
    }
  }
  // search top markers
  std::vector<double> max_scores(number_of_robots, 0.0);
  std::vector<cv::Mat> top_markers;
  std::vector<int> top_marker_locations(2*number_of_robots, 0);
  double min_max_score = 0.0;
  int min_max_score_index = 0;
  for (uint k=0; k<_robots.size(); k++){
    _robots[k]->getTopMarkers(top_markers);
    for (uint i=0; i<top_markers.size(); i++){
      matchTemplates(roi, top_markers[i], _threshold_match, marker_loc, marker_scores);
      if (marker_loc.size() == 2 && marker_scores[0] > min_max_score){
        top_marker_locations[2*min_max_score_index] = marker_loc[0] + roi_location[0];
        top_marker_locations[2*min_max_score_index+1] = marker_loc[1] + roi_location[1];
        max_scores[min_max_score_index] = marker_scores[0];
        robot_indices[min_max_score_index] = k;
        // find new minum of the max_scores
        min_max_score = 2.0;
        for (int j=0; j<number_of_robots; j++){
          if (max_scores[j] < min_max_score){
            min_max_score = max_scores[j];
            min_max_score_index = j;
          }
        }
      }
    }
  }
  bool top_markers_detected = false;
  for (int j=0; j<number_of_robots; j++){
    if (max_scores[j] > 0.0){
      top_markers_detected = true;
      break;
    }
  }
  if (!top_markers_detected){
    return false;
  }
  // combine bottom markers with correct robot
  double distance, ref_distance;
  int k;
  for (uint i=0; i<number_of_robots; i++){
    ref_distance = _pixelspermeter*_robots[robot_indices[i]]->getBottomTopDistance();
    k=0;
    for (uint j=0; j<bottom_marker_locations.size(); j+=2){
      distance = sqrt(pow(top_marker_locations[2*i]-bottom_marker_locations[j], 2) + pow(top_marker_locations[2*i+1]-bottom_marker_locations[j+1], 2));
      if (distance < 1.5*ref_distance && distance > 0.6*ref_distance){
        marker_locations[6*i+k] = bottom_marker_locations[j];
        marker_locations[6*i+k+1] = bottom_marker_locations[j+1];
        k+=2;
        if(k>=4){
          break;
        }
      }
    }
    if(k<4){
      return false;
    }
    marker_locations[6*i+4] = top_marker_locations[2*i];
    marker_locations[6*i+5] = top_marker_locations[2*i+1];
    // ensure fixed order of bottom markers (based on sign cross-product)
    int x1 = marker_locations[6*i+4];
    int y1 = marker_locations[6*i+5];
    int x2 = marker_locations[6*i+0];
    int y2 = marker_locations[6*i+1];
    int x3 = marker_locations[6*i+2];
    int y3 = marker_locations[6*i+3];
    if ((x2-x1) * (y3-y1) - (y2-y1) * (x3-x1) <= 0){
      marker_locations[6*i+0] = x3;
      marker_locations[6*i+1] = y3;
      marker_locations[6*i+2] = x2;
      marker_locations[6*i+3] = y2;
    }
  }
  return true;
}

void HawkEye::matchTemplates(cv::Mat& frame, const cv::Mat& marker, double threshold, std::vector<int>& marker_locations, std::vector<double>& marker_scores){
  // see http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/template_matching/template_matching.html
  marker_locations.clear();
  marker_scores.clear();
  cv::Mat result;
  int result_cols = frame.cols - marker.cols + 1;
  int result_rows = frame.rows - marker.rows + 1;
  result.create(result_rows, result_cols, CV_32FC1);
  double min_val, max_val;
  cv::Point min_loc, max_loc;
  if (marker.size().height < frame.size().height && marker.size().width < frame.size().width){
    cv::matchTemplate(frame, marker, result, cv::TM_CCOEFF_NORMED);
    while(true){
      cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);
      if (max_val >= threshold){
        // marker found
        marker_locations.push_back(max_loc.x + marker.size().width/2); // location in center of marker
        marker_locations.push_back(max_loc.y + marker.size().height/2);
        marker_scores.push_back(max_val);
        // hide marker for next loop
        blank(result, max_loc, marker.size().width, marker.size().height);
        cv::floodFill(result, max_loc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.0));
      } else {
        break;
      }
    }
  }
}

void HawkEye::blank(cv::Mat& frame, const cv::Point& location, int width, int height){
  cv::rectangle(frame, cv::Point(location.x-width/2, location.y-height/2), cv::Point(location.x+width/2, location.y+height/2), cv::Scalar(255,255,255), -1);
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
