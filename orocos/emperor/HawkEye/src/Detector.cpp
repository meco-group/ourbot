#include "Detector.hpp"

using namespace RTT;

Detector::Detector(const std::vector<double>& marker_params) : _marker_params(marker_params){
    initDetector();
}

void Detector::initDetector(){
  cv::SimpleBlobDetector::Params par;
  par.minThreshold = 10;
  par.maxThreshold = 200;
  par.filterByArea = true;
  par.minArea = 50;
  par.filterByCircularity = true;
  par.minCircularity = 0.7;
  _blob_detector = new cv::SimpleBlobDetector(par);
}

void HawkEye::start(const cv::Mat& background){
  _background = background;
}

void HawkEye::update(const cv::Mat& frame, std::vector<Robot*>& robots, std::vector<double>& obstacle_data){
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

void HawkEye::detectRobots2(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours, std::vector<Robot*>& robots){
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

      if (findMarkers(roi, roi_location, marker_locations, robot_indices)){
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

void HawkEye::subtractRobots(std::vector<std::vector<cv::Point> >& contours,
  std::vector<cv::Vec4i>& hierarchy, std::vector<Robot*>& robots){
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

void HawkEye::detectObstacles(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours, std::vector<double>& obstacle_data, const std::vector<Robot*>& robots){
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

void HawkEye::filterObstacles(const std::vector<cv::RotatedRect>& rectangles, const std::vector<std::vector<double> >& circles, std::vector<double>& obstacle_data, const std::vector<Robot*>& robots){
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
  if (obstacles.size() > _max_detectableobstacle_data){
    log(Warning) << "More obstacles detected than allowed!" << endlog();
    for (int k=0; k<_max_detectable_obstacles; k++){
      for (int i=0; i<5; i++){
        obstacle_data.push_back(obstacles[k][i]);
      }
    }
  } else {
    for (int k=0; k<obstacles.size(); k++){
      for (int i=0; i<5; i++){
        obstacle_data.push_back(obstacles[k][i]);
      }
    }
    for (int k=obstacles.size(); k<_max_detectable_obstacle; k++){
      for (int i=0; i<5; i++){
        obstacle_data.push_back(-100);
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


bool HawkEye::findMarkers(cv::Mat& roi, const std::vector<int>& roi_location, std::vector<int>& marker_locations, std::vector<int>& robot_indices){
  // detect blobs
  cv::cvtColor(roi, roi, CV_RGB2GRAY);
  std::vector<cv::KeyPoint> keypoints;
  _detector->detect(roi, keypoints);
  std::vector<cv::Point2f> points(3);
  int code;
  robot_indices.resize(0);
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
      for (int i=0; i < keypoints.size(); i++){
        if (selector[i]){
          points[k] = keypoints[i].pt;
          k++;
        }
      }
      // check if combination is a valid robot
      if ((code = isRobot(roi, points, robot_markers)) >= 0){
        robot_indices.push_back(code);
      }
    } while (std::prev_permutation(selector.begin(), selector.end()));
  } else if (keypoints.size() == 3){
    if ((code = isRobot(roi, points, robot_markers)) >= 0){
      robot_indices.push_back(code);
    }
  }
  if (robot_indices.size() > 0){
    return true;
  }
  return false;
}

int HawkEye::getTopIndex(const std::vector<cv::Point2f>& points){
  double dist_threshold = 2*1e-2;
  std::vector<double> dist(3);
  dist[0] = cv::norm(points[0] - points[1]);
  dist[1] = cv::norm(points[1] - points[2]);
  dist[2] = cv::norm(points[2] - points[0]);
  int ind_l = 0;
  int ind_s = 0;
  for (int k=0; k<3; k++){
      for (int l=0; l<3; l++){
          if (k != l){
              if ((abs(dist[k]-dist[l])/dist[l]) < dist_threshold){
                  ind_l = (k > l) ? k : l;
                  ind_s = (k < l) ? k : l;
                  return (ind_l-ind_s>1) ? ind_s : ind_l;
              }
          }
      }
  }
  return -1;
}

int HawkEye::isRobot(cv::Mat& roi, const std::vector<cv::Point2f>& points, std::vector<double>& robot_markers){
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
  double dist_ratio_threshold = 1e-2;
  double dist_ratio = 0.833333;
  double triangle_ratio = 0.5;
  double width = cv::norm(bottom[0] - bottom[1]);
  double height = cv::norm(top - midbottom);
  if (abs(height/width - triangle_ratio)/triangle_ratio > dist_ratio_threshold){
    return -1;
  }
  // decode QR tag
  cv::Point2f midpoint = 0.5*(midbottom + top);
  double step = (1.5/15.)*width;
  cv::Mat roi_mask;
  cv::GaussianBlur(roi, roi_mask, cv::Size(5, 5), 1, 1);
  cv::threshold(roi, roi, 100, 255, cv::THRESH_BINARY);
  cv::Scalar color;
  cv::Point2f point;
  unsigned int code = 0;
  unsigned int bit_selector = 1;
  for (int k=0; k<2; k++){
    for (int l=0; l<2; l++){
      point = cv::Point2f(midpoint.x - pow(-1,k)*step, midpoint.y - pow(-1,k)*step);
      color = roi_mask.at<uchar>(point);
      if (color.val[0] == 0){
        code |= (bit_selector << 2*k+l);
      }
    }
  }
  robot_markers[0] = top[0];
  robot_markers[1] = top[1];
  // order ...
  robot_markers[2] = bottom[]
  std::cout << "code is: " << std::bitset<8>(code) << std::endl;
  return code;
}

bool HawkEye::matchMarkers(cv::Mat& roi, const std::vector<int>& roi_location, std::vector<int>& marker_locations, std::vector<int>& robot_indices){
  cv::cvtColor(roi, roi, CV_RGB2GRAY);
  std::vector<int> marker_loc;
  std::vector<double> marker_scores;
  // search bottom markers
  matchTemplates(roi, _bottom_marker, _threshold_match, marker_loc, marker_scores);
  if (marker_loc.size() < 2){
    return false; // no bottom markers detected
  }
  std::vector<int> bottom_marker_locations(marker_loc.size(), 0);
  for (int i=0; i<marker_loc.size(); i++){
    bottom_marker_locations[i] = marker_loc[i] + roi_location[i%2];
    if (i%2 == 0){
      // blank-out detected markers
      blank(roi, cv::Point(marker_loc[i], marker_loc[i+1]), _bottom_marker.size().width, _bottom_marker.size().height);
    }
  }
  // search top markers
  std::vector<cv::Mat> top_markers;
  std::vector<int> top_marker_locations;
  double max_score;
  int number_of_robots = 0;
  for (uint k=0; k<_robots.size(); k++){
    _robots[k]->getTopMarkers(top_markers);
    max_score = 0.0;
    for (uint i=0; i<top_markers.size(); i++){
      matchTemplates(roi, top_markers[i], _threshold_match, marker_loc, marker_scores);
      if (marker_loc.size() == 2 && marker_scores[0] > max_score){
        top_marker_locations.push_back(marker_loc[0] + roi_location[0]);
        top_marker_locations.push_back(marker_loc[1] + roi_location[1]);
        max_score = marker_scores[0];
        robot_indices.push_back(k);
      }
    }
    if (max_score > 0.0){
      number_of_robots++;
    }
  }
  if (number_of_robots == 0){
    return false;
  }
  marker_locations.resize(6*number_of_robots, 0);
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
    if (k<4){
      return false; // we did not find 2 matching bottom markers
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
