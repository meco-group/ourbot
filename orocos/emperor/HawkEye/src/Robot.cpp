#include "Robot.hpp"

Robot::Robot(int width, int height, double z_position, double z_position_cam, std::vector<double>& local_marker_locations_m):
_width(width), _height(height), _z_position(z_position), _z_position_cam(z_position_cam), _local_marker_locations_m(local_marker_locations_m){
    reset();
}

void Robot::reset(){
    _detected = false;
}

bool Robot::detected(){
    return _detected;
}

void Robot::setMarkers(const std::vector<int>& marker_locations_px){
    _global_marker_locations_px = marker_locations_px;
    _detected = true;
    createBox();
}

void Robot::createBox(){
    double x1 = _global_marker_locations_px[4]; // top
    double y1 = _global_marker_locations_px[5];
    double x2 = _global_marker_locations_px[0]; // bottom
    double y2 = _global_marker_locations_px[1];
    double x3 = _global_marker_locations_px[2];
    double y3 = _global_marker_locations_px[3];

    double lineside = (x3-x2)*(y1-y2) - (y3-y2)*(x1-x2);
    double orientation = atan2((y3-y2),(x3-x2));
    if (lineside > 0){
      orientation += M_PI/2.;
    } else {
      orientation -= M_PI/2.; // always this?
    }
    double x_bottom_px = 0.5*(x2+x3);
    double y_bottom_px = 0.5*(y2+y3);

    double x_bottom_m = 0.5*(_local_marker_locations_m[0]+_local_marker_locations_m[2]);
    double y_bottom_m = 0.5*(_local_marker_locations_m[1]+_local_marker_locations_m[3]);

    double dist_top_bottom_px = sqrt(pow(x1-x_bottom_px, 2) + pow(y1-y_bottom_px, 2));
    double dist_top_bottom_m = sqrt(pow(_local_marker_locations_m[4]-x_bottom_m, 2) + pow(_local_marker_locations_m[5]-y_bottom_m, 2));

    double scale = dist_top_bottom_px/dist_top_bottom_m;

    double local_marker_center_x = scale*(_local_marker_locations_m[0]+_local_marker_locations_m[2]+_local_marker_locations_m[4])/3.;
    double local_marker_center_y = -scale*(_local_marker_locations_m[1]+_local_marker_locations_m[3]+_local_marker_locations_m[5])/3.;

    double center_x = (x1+x2+x3)/3. - (local_marker_center_x*cos(orientation) - local_marker_center_y*sin(orientation));
    double center_y = (y1+y2+y3)/3. - (local_marker_center_x*sin(orientation) + local_marker_center_y*cos(orientation));

    std::cout << "orientation: " << orientation << std::endl;

    orientation += M_PI/2;
    if (orientation < 0){
      orientation += 2*M_PI;
    }
    _box = cv::RotatedRect(cv::Point2f(center_x, center_y), cv::Size2f(_width, _height), (180./M_PI)*orientation);
}

cv::RotatedRect Robot::getBox(){
    return _box;
}

int Robot::getWidth(){
    return _width;
}

int Robot::getHeight(){
    return _height;
}

void Robot::getMarkers(std::vector<double>& marker_vector, int pixelspermeter, int height, int width){
  std::vector<double> position(2);
  for (int i=0; i<3; i++){
    position[0] = _global_marker_locations_px[2*i];
    position[1] = _global_marker_locations_px[2*i+1];
    position = transform(position, pixelspermeter, height, width);
    marker_vector[2*i] = position[0];
    marker_vector[2*i+1] = position[1];
  }
}

void Robot::setPose(const std::vector<double>& pose){
  _pose = pose;
}

void Robot::setRef(const std::vector<double>& ref_x, const std::vector<double>& ref_y){
  _ref_x = ref_x;
  _ref_y = ref_y;
}

void Robot::draw(cv::Mat& frame, const cv::Scalar& color, int pixelspermeter, int draw_amount){
  // markers
  cv::Point2f center;
  for (int i=0; i<2; i++){
    cv::circle(frame, cv::Point2f(_global_marker_locations_px[2*i], _global_marker_locations_px[2*i+1]), 3, color, -1);
  }
  cv::circle(frame, cv::Point2f(_global_marker_locations_px[4], _global_marker_locations_px[5]), 3, cv::Scalar(0,0,255), -1);
  // pose
  std::vector<double> point1(2), point2(2);
  double orientation = _pose[2];
  point1[0] = _pose[0];
  point1[1] = _pose[1];
  point2[0] = point1[0] + 0.1*cos(orientation);
  point2[1] = point1[1] + 0.1*sin(orientation);
  point1 = invtransform(point1, pixelspermeter, frame.size().height, frame.size().width);
  point2 = invtransform(point2, pixelspermeter, frame.size().height, frame.size().width);
  cv::circle(frame, cv::Point2f(point1[0], point1[1]), 5, color, -2);
  cv::line(frame, cv::Point2f(point1[0], point1[1]), cv::Point2f(point2[0], point2[1]), color, 2);
  // reference
  cv::Scalar color_w;
  mixWithWhite(color, color_w, 50);
  std::vector<std::vector<double> > points;
  std::vector<double> point(2);
  for (uint i=0; i<_ref_x.size(); i++){
    point[0] = _ref_x[i];
    point[1] = _ref_y[i];
    if (point[0] != 0 && point[1] != 0){
      point = invtransform(point, pixelspermeter, frame.size().height, frame.size().width);
      points.push_back(point);
    }
  }
  if (points.size() > 0){
    for (uint i=0; i<points.size()-1; i++){
      if (!((points[i][0] == 0 && points[i][1] == 0) || (points[i+1][0] == 0 && points[i+1][1] == 0))){
        cv::line(frame, cv::Point2f(points[i][0], points[i][1]), cv::Point2f(points[i+1][0], points[i+1][1]), color_w, 2);
      }
    }
  }
  if (draw_amount >= 4) {
    // box
    cv::Point2f vertices[4];
    _box.points(vertices);
    for (int i=0; i<4; i++){
      cv::line(frame, vertices[i], vertices[(i+1)%4], color, 2);
    }
  }
}

void Robot::mixWithWhite(const cv::Scalar& color, cv::Scalar& color_w, double perc_white){
  color_w[0] = int(((100. - perc_white)*color[0] + perc_white*255.)/100.);
  color_w[1] = int(((100. - perc_white)*color[1] + perc_white*255.)/100.);
  color_w[2] = int(((100. - perc_white)*color[2] + perc_white*255.)/100.);
}

std::vector<double> Robot::transform(const std::vector<double>& point, int pixelspermeter, int height, int width){
  std::vector<double> point_tf(2);
  double alpha = (_z_position_cam-_z_position)/_z_position_cam;
  double pixelspermeter_z = pixelspermeter/alpha;
  point_tf[0] = (1.0/pixelspermeter_z)*point[0] + 0.5*(1 - alpha)*(1.0*width)/pixelspermeter;
  point_tf[1] = (1.0/pixelspermeter)*height - ((1.0/pixelspermeter_z)*point[1] + 0.5*(1 - alpha)*((1.0/pixelspermeter)*height));
  return point_tf;
}

std::vector<double> Robot::invtransform(const std::vector<double>& point, int pixelspermeter, int height, int width){
  std::vector<double> point_tf(2);
  double alpha = (_z_position_cam-_z_position)/_z_position_cam;
  double pixelspermeter_z = pixelspermeter/alpha;
  point_tf[0] = pixelspermeter_z*(point[0] - 0.5*(1 - alpha)*(1.0/pixelspermeter)*width);
  point_tf[1] = -pixelspermeter_z*(point[1] + height/pixelspermeter + 0.5*(1 - alpha)*(1.0/pixelspermeter)*height);
  return point_tf;
}
