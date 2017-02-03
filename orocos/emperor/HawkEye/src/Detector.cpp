#include "Detector.hpp"
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace RTT;
using namespace RTT::os;


using namespace RTT;

Detector::Detector(const std::vector<double>& marker_params, int threshold_bgst,
  double threshold_kp, double threshold_top, double min_robot_area, double min_obstacle_area, int max_detectable_obstacles,
  int pixelspermeter) : _threshold_bgst(threshold_bgst), _threshold_kp(threshold_kp), _threshold_top(threshold_top),
_min_robot_area(min_robot_area), _min_obstacle_area(min_obstacle_area),
_max_detectable_obstacles(max_detectable_obstacles), _pixelspermeter(pixelspermeter){
    initDetector();
    _nbit_x = int(marker_params[0]);
    _nbit_y = int(marker_params[1]);
    _triangle_ratio = marker_params[2];
    _qr_rel_pos = marker_params[3];
    _qr_rel_width = marker_params[4];
}

void Detector::initDetector(){
  cv::SimpleBlobDetector::Params par;
  par.minThreshold = 10;
  par.maxThreshold = 200;
  par.filterByArea = true;
  par.minArea = 80;
  par.filterByCircularity = true;
  par.minCircularity = 0.85;
  _blob_detector = new cv::SimpleBlobDetector(par);
}

bool Detector::start(const cv::Mat& background){
  _background = background;
  _frame_height = background.size().height;
  return true;
}

void Detector::update(const cv::Mat& frame, std::vector<Robot*>& robots, std::vector<double>& obstacle_data){
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  // background subtraction
  backgroundSubtraction(frame, contours, hierarchy);
  if (hierarchy.empty()){
    return; // no contours were found
  }
  detectRobots(frame, contours, robots);
  subtractRobots(contours, hierarchy, robots);
  if (hierarchy.empty()){
    return; // only robot contours were found
  }
  detectObstacles(frame, contours, obstacle_data, robots);
}

void Detector::backgroundSubtraction(const cv::Mat& frame, std::vector<std::vector<cv::Point> >& contours,
  std::vector<cv::Vec4i>& hierarchy){
  cv::Mat mask_copy;
  cv::absdiff(_background, frame, _mask);
  cv::cvtColor(_mask, _mask, CV_RGB2GRAY);
  cv::threshold(_mask, _mask, _threshold_bgst, 255, cv::THRESH_BINARY);
  _mask.copyTo(mask_copy);
  cv::findContours(mask_copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

void Detector::detectRobots(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours, std::vector<Robot*>& robots){
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
      if (findRobots(roi, roi_location, marker_locations, robot_indices)){
        for (uint k=0; k<robot_indices.size(); k++){
          DEBUG_PRINT("Detected robot " << robot_indices[k] << "!")
          for (uint j=0; j<6; j++){
            marker_loc_rob[j] = marker_locations[6*k+j];
          }
          robots[robot_indices[k]]->setMarkers(marker_loc_rob);
        }
      }
    }
  }
}

bool Detector::findRobots(cv::Mat& roi, const std::vector<int>& roi_location, std::vector<int>& marker_locations, std::vector<int>& robot_indices){
  // detect blobs
  cv::cvtColor(roi, roi, CV_RGB2GRAY);
  std::vector<cv::KeyPoint> keypoints;
  cv::resize(roi, roi, cv::Size(), 2, 2);
  _blob_detector->detect(roi, keypoints);
  std::vector<cv::Point2f> points(3);
  int code;
  robot_indices.resize(0);
  std::vector<std::vector<double> > markers_per_robot;
  if (keypoints.size() < 3){
    return false;
  }
  std::vector<double> robot_markers(6);
  if (keypoints.size() > 3){
    std::vector<bool> selector(keypoints.size());
    std::fill(selector.begin(), selector.begin() + 3, true);
    do {
      // get a combination of 3 points
      int k = 0;
      for (uint i=0; i < keypoints.size(); i++){
        if (selector[i]){
          points[k] = keypoints[i].pt;
          k++;
        }
      }
      // check if combination is a valid robot
      if ((code = isRobot(roi, points, robot_markers)) >= 0){
        robot_indices.push_back(code);
        markers_per_robot.push_back(robot_markers);
      }
    } while (std::prev_permutation(selector.begin(), selector.end()));
  } else if (keypoints.size() == 3){
    for (uint k=0; k<3; k++){
      points[k] = keypoints[k].pt;
    }
    if ((code = isRobot(roi, points, robot_markers)) >= 0){
      robot_indices.push_back(code);
      markers_per_robot.push_back(robot_markers);
    }
  }
  if (robot_indices.size() > 0){
    marker_locations.resize(6*robot_indices.size(), 0);
    for (uint i=0; i<robot_indices.size(); i++){
      for (int j=0; j<6; j++){
        marker_locations[6*i+j] = 0.5*markers_per_robot[i][j] + roi_location[j%2];
      }
    }
    return true;
  }
  return false;
}

int Detector::isRobot(cv::Mat& roi, const std::vector<cv::Point2f>& points, std::vector<double>& robot_markers){
  robot_markers.resize(6);
  int top_index = getTopIndex(points);
  if (top_index < 0){
    return -1;
  }
  cv::Point2f top = points[top_index];
  std::vector<cv::Point2f> bottom(2);
  int k=0;
  for (int l=0; l<3; l++){
    if (l != top_index){
      bottom[k] = points[l];
      k++;
    }
  }
  cv::Point2f midbottom = 0.5*(bottom[0]+bottom[1]);
  // check if valid triangle
  double width = cv::norm(bottom[0] - bottom[1]);
  double height = cv::norm(top - midbottom);
  if (fabs(height/width - _triangle_ratio)/_triangle_ratio > _threshold_kp){
    return -1;
  }
  // decode QR tag
  cv::Point2f midpoint = midbottom + _qr_rel_pos*(top - midbottom);
  double step_size = 0.25*height*_qr_rel_width;
  cv::Point2f step_x = 0.25*_qr_rel_width*(top-midbottom);
  cv::Point2f step_y = cv::Point2f(-step_x.y, step_x.x);
  cv::Mat roi_mask;
  roi.copyTo(roi_mask);
  // cv::GaussianBlur(roi_mask, roi_mask, cv::Size(5, 5), 1, 1);
  cv::threshold(roi_mask, roi_mask, 100, 255, cv::THRESH_BINARY);
  cv::Scalar color;
  cv::Point2f point;
  unsigned int code = 0;
  unsigned int bit_selector = 1;
  for (int k=0; k<2; k++){
    for (int l=0; l<2; l++){
      point = midpoint - pow(-1,k)*step_x - pow(-1,l)*step_y;
      color = roi_mask.at<uchar>(point);
      if (color.val[0] == 0){
        code |= (bit_selector << (k+2*l));
      }
    }
  }
  // std::cout << "code : " << code << std::endl;
  // std::cout << "code : " <<  std::bitset<8>(code) << std::endl;
  // fixed order of markers (left - right - top)
  double x1 = top.x;
  double y1 = top.y;
  double x2 = bottom[0].x;
  double y2 = bottom[0].y;
  double x3 = bottom[1].x;
  double y3 = bottom[1].y;

  if (((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)) >= 0){
    robot_markers[0] = x2;
    robot_markers[1] = y2;
    robot_markers[2] = x3;
    robot_markers[3] = y3;
  } else {
    robot_markers[0] = x3;
    robot_markers[1] = y3;
    robot_markers[2] = x2;
    robot_markers[3] = y2;
  }
  robot_markers[4] = top.x;
  robot_markers[5] = top.y;
  return code;
}

int Detector::getTopIndex(const std::vector<cv::Point2f>& points){
  std::vector<double> dist(3);
  dist[0] = cv::norm(points[0] - points[1]);
  dist[1] = cv::norm(points[1] - points[2]);
  dist[2] = cv::norm(points[2] - points[0]);
  int ind_l = 0;
  int ind_s = 0;
  for (int k=0; k<3; k++){
      for (int l=0; l<3; l++){
          if (k != l){
              if ((fabs(dist[k]-dist[l])/dist[l]) < _threshold_top){
                  ind_l = (k > l) ? k : l;
                  ind_s = (k < l) ? k : l;
                  return (ind_l-ind_s>1) ? ind_s : ind_l;
              }
          }
      }
  }
  return -1;
}

void Detector::subtractRobots(std::vector<std::vector<cv::Point> >& contours,
  std::vector<cv::Vec4i>& hierarchy, std::vector<Robot*>& robots){
  cv::Point2f robot_vertices[4];
  std::vector<cv::Point> robot_contour(4);
  std::vector<std::vector<cv::Point> > robot_contour_vector;
  for (uint k=0; k<robots.size(); k++){
    if (robots[k]->detected()){
      robots[k]->getBox().points(robot_vertices);
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

void Detector::detectObstacles(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours, std::vector<double>& obstacle_data, const std::vector<Robot*>& robots){
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
  filterObstacles(rectangles, circles, obstacle_data, robots);
}

void Detector::filterObstacles(const std::vector<cv::RotatedRect>& rectangles, const std::vector<std::vector<double> >& circles, std::vector<double>& obstacle_data, const std::vector<Robot*>& robots){
  bool add;
  cv::Point2f vertices[4];
  std::vector<cv::Point> other_contour(4);
  std::vector<std::vector<double> > obstacles;
  std::vector<double> areas;
  for (uint i=0; i<rectangles.size(); i++){
    add = true;
    for (uint k=0; k<robots.size(); k++){
      if (robots[k]->detected()){
        robots[k]->getBox().points(vertices);
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
        obstacle_data.push_back(obstacles[k][i]);
      }
    }
  } else {
    for (uint k=0; k<obstacles.size(); k++){
      for (int i=0; i<5; i++){
        obstacle_data.push_back(obstacles[k][i]);
      }
    }
    for (uint k=obstacles.size(); k<_max_detectable_obstacles; k++){
      for (int i=0; i<5; i++){
        obstacle_data.push_back(-100);
      }
    }
  }
}

void Detector::addObstacle(const cv::RotatedRect& rectangle, const std::vector<double>& circle,
  std::vector<std::vector<double>>& obstacles, std::vector<double>& areas){
  double circle_area = M_PI*circle[2]*circle[2];
  double rectangle_area = rectangle.size.width * rectangle.size.height;
  std::vector<double> obstacle(5);
  if (rectangle_area < circle_area){
    obstacle[0] = rectangle.center.x*1.0/_pixelspermeter;
    obstacle[1] = (_frame_height-rectangle.center.y)*1.0/_pixelspermeter;
    obstacle[2] = -rectangle.angle;
    obstacle[3] = rectangle.size.width*1.0/_pixelspermeter;
    obstacle[4] = rectangle.size.height*1.0/_pixelspermeter;
    areas.push_back(rectangle_area);
  } else {
    obstacle[0] = circle[0]*1.0/_pixelspermeter;
    obstacle[1] = (_frame_height-circle[1])*1.0/_pixelspermeter;
    obstacle[2] = circle[2]*1.0/_pixelspermeter;
    obstacle[3] = -100;
    obstacle[4] = -100;
    areas.push_back(circle_area);
  }
  obstacles.push_back(obstacle);
}

void Detector::sortObstacles(std::vector<std::vector<double> >& obstacles, std::vector<double>& areas){
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
