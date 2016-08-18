#include "Scanmatcher-component.hpp"

// In order to use this component, you need the C(anonical) Scan Matcher lib:
// sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
// sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
// sudo apt-get update
// sudo apt-get install ros-indigo-csm

#undef min
#undef max
#undef FALSE
#undef TRUE

Scanmatcher::Scanmatcher(std::string const& name) : TaskContext(name),
_enc_pose(3), _estimated_position_change(3), _covariance(9), _rows(3), _columns(3){

	// Some defaults when no properties are available
	_lidar_data_length = 400;
	_max_sense_range = 5;
	_sensenumber = 128;

	// InputPorts for the Scanmatcher component
	ports()->addPort( "cor_lidar_distance_port",_cor_lidar_distance_port).doc("Input port for corrected lidar node positions. Holds a vector of 100 distances [m]");
  ports()->addPort( "cor_lidar_angle_port",_cor_lidar_angle_port).doc("Input port for corrected lidar node positions. Holds a vector of 100 angles [rad]");
  ports()->addPort("cal_enc_pose_port",_cal_enc_pose_port).doc("Input port for calibrated encoder values. (x,y,orientation) in [m , m ,rad]");
  ports()->addEventPort("trigger_scanmatcher_port", _trigger_scanmatcher_port).doc("Input port which triggers scanmatcher");
  //@@@Michiel: add extra output port with pose at start scan
  ports()->addPort("scanstart_pose_port", _scanstart_pose_port).doc("Pose at start of a scan");

  // OutputPorts
 	ports()->addPort( "scanmatch_pose_port", _scanmatch_pose_port).doc("Estimated position-change of scanmatcher in encoder-form [x,y,orientation]");
	ports()->addPort( "scanmatch_covariance_port", _scanmatch_covariance_port).doc("covariance scanmatcher [x,y,orientation]");

  // Properties
  addProperty("max_sense_range", _max_sense_range).doc("Maximum distance the lidar can measure");
  addProperty("max_angular_correction_deg",_max_angular_correction_deg).doc("Maximum correction for angles [deg]");
  addProperty("max_linear_correction",_max_linear_correction ).doc("Maximum translation between scans [m]");
  addProperty("use_corr_tricks", _use_corr_tricks ).doc("Use smart tricks for finding correspondences.");
  addProperty("max_iterations",_max_iterations ).doc("Maximum iterations to do");
  addProperty("epsilon_xy",_epsilon_xy ).doc("A threshold for stopping [m]");
  addProperty("epsilon_theta",_epsilon_theta ).doc("A threshold for stopping (rad)");
  addProperty("sigma",_sigma ).doc("Noise in the scan [m]");
  addProperty("max_correspondence_dist",_max_correspondence_dist ).doc("Dubious parameter [m]");
  addProperty("orientation_neighbourhood",_orientation_neighbourhood ).doc("Number of neighbour rays used to estimate the orientation");
  addProperty("outliers_maxPerc",_outliers_maxPerc ).doc("Maximum percentage of outliers to remove");
  addProperty("outliers_adaptive_order",_outliers_adaptive_order ).doc("Outlier adaptive order");
  addProperty("outliers_adaptive_mult",_outliers_adaptive_mult).doc("Outliers adaptive mult");
  addProperty("use_point_to_line_distance",_use_point_to_line_distance).doc("1 to use PLICP, 0 for ICP");
  addProperty("use_odometry_guess",_use_odometry_guess).doc("1 to make guess with odometry information, 0 pure scanmatch");
  addProperty("scans",_scans).doc("Number of reference scans to keep as reference to match new scan");
  addProperty("do_compute_covariance",_do_compute_covariance).doc("Calculate covariance matrix Scan matcher");
  addProperty("lidar_data_length", _lidar_data_length).doc("Length of the lidar data");
  addProperty("precision", _precision).doc("Precision used to compute artificial lidar");

  std::cout << "Scanmatcher constructed !" <<std::endl;
}

bool Scanmatcher::configureHook(){
	_available_lidar_distances.resize(_lidar_data_length);
	_available_lidar_angles.resize(_lidar_data_length);
  _prev_lidar_distances.resize((_lidar_data_length - _sensenumber)*_scans);
  _prev_lidar_angles.resize((_lidar_data_length - _sensenumber)*_scans);
	_lidar_distances.resize(_lidar_data_length- _sensenumber);
	_lidar_angles.resize(_lidar_data_length- _sensenumber);

  _start_pose.resize(3);

	std::fill(_prev_lidar_angles.begin(), _prev_lidar_angles.end(), 0);
	std::fill(_prev_lidar_distances.begin(), _prev_lidar_distances.end(), 0);

  _scanmatch_pose_port.setDataSample(_estimated_position_change);
  std::vector<double> example(9, 0.0);
  _scanmatch_covariance_port.setDataSample(example);

  // Settings of scanmatcher
  ls.min_reading = 0.0;
  ls.max_reading = _max_sense_range;
  ls.max_angular_correction_deg = _max_angular_correction_deg;//60
  ls.max_linear_correction = _max_linear_correction;//0.5
  ls.use_corr_tricks = _use_corr_tricks;//1
  ls.max_iterations = _max_iterations;
  ls.epsilon_xy = _epsilon_xy;//0.001
  ls.epsilon_theta = _epsilon_theta;//0.0001
  ls.max_correspondence_dist = _max_correspondence_dist;//0.3
  ls.restart = 0;//1
  ls.restart_threshold_mean_error = 0.1;//0.1
  ls.restart_dt = 1.0;//1.0
  ls.restart_dtheta = 0.1;//0.1
  ls.outliers_maxPerc = _outliers_maxPerc;//0.85
  ls.outliers_adaptive_order = _outliers_adaptive_order;//0.7
  ls.outliers_adaptive_mult = _outliers_adaptive_mult;//2.0
  ls.outliers_remove_doubles = 1;//1
  ls.clustering_threshold = 0.10;//0.15
  ls.orientation_neighbourhood = _orientation_neighbourhood;//10
  ls.do_alpha_test = 0;
  ls.do_alpha_test_thresholdDeg = 20.0;
  ls.do_visibility_test = 0;//0
  ls.use_point_to_line_distance = _use_point_to_line_distance;//1
  ls.use_ml_weights = 0;
  ls.use_sigma_weights = 0;
  ls.do_compute_covariance = _do_compute_covariance;
  ls.debug_verify_tricks = 0;
  ls.laser[0] = 0;
  ls.laser[1] = 0;
  ls.laser[2] = 0;
  ls.sigma = _sigma;//0.01

  _lastX = 0;
  _lastY = 0;
  _lastTheta = 0;

  loadEnvironment();
  

  // TODO laten werken
  //_environment_circles = std::vector<std::vector<double> _circle(3);> _cirlces;
  //_environment_polygons = std::vector<std::vector<std::vector<double> _point(2);> _polygon;> _polygons;

  std::cout << "Scanmatcher configured !" <<std::endl;
  return true;
}

bool Scanmatcher::startHook(){
  std::cout << "Scanmatcher started !" <<std::endl;
  return true;
}

void Scanmatcher::updateHook(){

  std::cout << "Scanmatcher updateHook !" <<std::endl;

  artificialLidar();

  std::cout << "Scanmatcher artificialLidar passed !" <<std::endl;
  
  /*
  int rays = _lidar_data_length - _sensenumber;
	sm_result result;

	// local scope array to put in ls-struct
	ls.laser_sens = ld_alloc_new(rays);
	ls.laser_ref = ld_alloc_new(rays*_scans);

  if(_use_odometry_guess == 1){
    _cal_enc_pose_port.read(_enc_pose);
    ls.first_guess[0] = _enc_pose[0] - _lastX;
    ls.first_guess[1] = _enc_pose[1] - _lastY;
    ls.first_guess[2] = _enc_pose[2] - _lastTheta;
  }
  else{
    ls.first_guess[0] = 0;
    ls.first_guess[1] = 0;
    ls.first_guess[2] = 0;
  }
  _lastX = _enc_pose[0];
  _lastY = _enc_pose[1];
  _lastTheta = _enc_pose[2];

  // Reading data from ports
  _cor_lidar_distance_port.read(_available_lidar_distances);
  _cor_lidar_angle_port.read(_available_lidar_angles);

  //TODO
  _scanstart_pose_port.read(_start_pose);

  // Copying all laser beams that are not used by Slam to use them for positioning with the scanmatcher
  int index = 0;
  int index2 = 1;
  for(int i = _sensenumber; i < _lidar_data_length; i++){
    if (index2 >= _lidar_data_length){break;}
    _lidar_distances[index] = _available_lidar_distances[index2];
    _lidar_angles[index] = _available_lidar_angles[index2];
    index++;
    if(index2 <= 2 && index2 % 2 == 0){index2++;}
    index2++;
    if(index2 > 3 && index2 % 3 == 0){index2++;}
  }

  if(_lidar_distances[1] != 0 && _lidar_distances[2] != 0 && _lidar_distances[3] != 0){
    for(int i=0; i< rays; i++){
      if (_lidar_angles[i] > 2*M_PI){
        _lidar_angles[i] = _lidar_angles[i] - 2*M_PI;
      }
	    else if (_lidar_angles[i] < 0){
        _lidar_angles[i] = _lidar_angles[i] + 2*M_PI;
      }
    }

    // give distances and angles to sm-algorithm
    double angles[rays];
    double distances[rays];
    for(int i= 0; i< rays; i++){
      distances[i] = _lidar_distances[i];
      angles[i] = _lidar_angles[i];
    }
    quickSort(angles, distances, 0, rays-1);
    for(int i=0; i< rays; i++){
      _lidar_distances[i] = distances[i];
      _lidar_angles[i] = angles[i];
    }
    for(int i=0; i< rays; i++){
      ls.laser_sens->readings[i] = _lidar_distances[i];
      ls.laser_sens->theta[i] = _lidar_angles[i];
    	if(_lidar_distances[i] > _max_sense_range){
         	ls.laser_sens->valid[i] = 0;
    	}
    	else
      {
        ls.laser_sens->valid[i] = 1;
      }
    }
    // give ref distances and angles to sm-algorithm
  	double anglesref[rays*_scans];
    double distancesref[rays*_scans];
    for(int i= 0; i<rays*_scans; i++){
        distancesref[i] = _prev_lidar_distances[i];
        anglesref[i] = _prev_lidar_angles[i];
    }
    quickSort(anglesref, distancesref, 0, _scans*rays-1);
    for(int i= 0; i< rays*_scans; i++){
     	ls.laser_ref->readings[i] = distancesref[i];
      ls.laser_ref->theta[i] = anglesref[i];
      if(distancesref[i] > _max_sense_range){
        ls.laser_ref->valid[i] = 0;
      }
      else
      {
        ls.laser_ref->valid[i] = 1;
      }
    }

    // information dependent settings
    ls.laser_sens->min_theta = _lidar_angles[0];
    ls.laser_sens->max_theta = _lidar_angles[rays-1];
    ls.laser_ref->min_theta = anglesref[0];
    ls.laser_ref->max_theta = anglesref[rays*_scans-1];

    // reference pose is (0,0,0)
    ls.laser_ref->true_pose[0] = 0;
    ls.laser_ref->true_pose[1] = 0;
    ls.laser_ref->true_pose[2] = 0;
    ls.laser_ref->odometry[0] = 0;
    ls.laser_ref->odometry[1] = 0;
    ls.laser_ref->odometry[2] = 0;

    // Solve scan match problem
    sm_icp(&ls, &result);

    if(result.valid == 1){
      _estimated_position_change[0] = result.x[0];
      _estimated_position_change[1] = result.x[1];
      _estimated_position_change[2] = result.x[2];

      _rows = result.cov_x_m->size1;
      _columns = result.cov_x_m->size2;

    	for(int i=0; i<_rows; i++){
    		for(int j=0;j<_columns;j++){
    		  _covariance[i*_rows+j] = gsl_matrix_get(result.cov_x_m, i, j);
    		}
    	}
    } else{
    	_estimated_position_change[0] = ls.first_guess[0];
    	_estimated_position_change[1] = ls.first_guess[1];
    	_estimated_position_change[2] = ls.first_guess[2];
    	for(int i=0; i<_rows; i++){
    		for(int j=0;j<_columns;j++){
    		  _covariance[i*_rows+j] = 0.1;
    		}
    	}
    }

    // set data on ports
    _scanmatch_covariance_port.write(_covariance);
    _scanmatch_pose_port.write(_estimated_position_change);

  	correctInformation(_prev_lidar_distances, _prev_lidar_angles, _estimated_position_change);

  	for(int i=rays*(_scans-1); i< rays*_scans; i++){
      _prev_lidar_distances[i] = _lidar_distances[i-rays*(_scans-1)];
      _prev_lidar_angles[i] = _lidar_angles[i-rays*(_scans-1)];
    }
    ld_free(ls.laser_ref);
    ld_free(ls.laser_sens);
  }
  else {
    fprintf(stderr, "Scanmatcher got all zeros!");
  }
  */
}

void Scanmatcher::stopHook() {
  std::cout << "Scanmatcher executes stopping !" <<std::endl;
}

void Scanmatcher::cleanupHook() {
  std::cout << "Scanmatcher cleaning up !" <<std::endl;
}

void Scanmatcher::quickSort(double angles[], double distances[] ,int low, int high) {
  int start, end, p;
  double k;

  if (low < high){
    p = (int)((low+high)/2);
    double tmp;
    double tmp2;
    tmp = angles[low];
    tmp2= distances[low];
    angles[low] = angles[p];
    distances[low] = distances[p];
    angles[p] = tmp;
    distances[p]= tmp2;
    k = angles[low];
    start = (low+1);
    end = high;

    while (start <= end){
      while ((start <= high) && (angles[start] <= k)){
         start++;
      }
      while ((end >= low) && (angles[end] > k)){
         end--;
      }
      if (start < end){
        double tmpp;
        double tmpp2;
        tmpp = angles[start];
        tmpp2 = distances[start];
        angles[start] = angles[end];
        distances[start] = distances[end];
        angles[end] = tmpp;
        distances[end] = tmpp2;
      }
    }
    double tmppp;
    double tmppp2;
    tmppp = angles[low];
    tmppp2 = distances[low];
    angles[low] = angles[end];
    distances[low] = distances[end];
    angles[end] = tmppp;
    distances[end] = tmppp2;
    quickSort(angles, distances, low, (end-1));
    quickSort(angles, distances, (end+1),high);
  }
}

void Scanmatcher::correctInformation(std::vector<double> laser_scan_distance, std::vector<double> laser_scan_angle ,std::vector<double> delta_odo_scan)
{
  double rotAngle, movAngle;
  double x_cart;
  double y_cart;
  // double theta_cart;
  double x_old, y_old, x_new, y_new;

  // In de eerste for loop worden cartesiaanse coordinaten berekend in het het robotassenstelsel van op de eerste positie

  // De hoeveelheid van beweging in één richting volgens het assenstelsel van het punt i
  movAngle = delta_odo_scan[2]/2;

  // De cartesiaanse coordinaten in het robotassenstelsel van de eerste positie, theta_cart is de hoek tussen het assenstelsel
  // op positie i en de allereerste positie. De veranderingen in beweging moeten dus geprojecteerd (geroteerd) worden t.o.v.
  // het cartesiaans assenstelsel en opgeteld worden bij de vorige positie.
  x_cart    = delta_odo_scan[0]*cos(movAngle) - delta_odo_scan[1]*sin(movAngle);
  y_cart    = delta_odo_scan[1]*cos(movAngle) + delta_odo_scan[0]*sin(movAngle);
  rotAngle = -delta_odo_scan[2];

  // In de tweede for loop worden de laserstralen eerst geroteerd
  for(int i = (_lidar_data_length - _sensenumber); i < (_lidar_data_length - _sensenumber)*_scans; i++){

  // eerst wordt geroteerd zodat de coördinaten t.o.v. een assenstelsel zijn op het punt
  // van de tussenpositie van de robot, maar wel met dezelfde oriëntatie als de allereerste positie

    x_old = laser_scan_distance[i]*cos(laser_scan_angle[i]) - x_cart;
    y_old = laser_scan_distance[i]*sin(laser_scan_angle[i]) - y_cart;

    // Nu worden de laserstralen opnieuw geroteerd zodat hun coördinaten zoals deze opgemeten zouden worden kunnen berekend worden

    x_new = x_old*cos(rotAngle) - y_old*sin(rotAngle);
    y_new = x_old*sin(rotAngle) + y_old*cos(rotAngle);

    // Ten laatste worden de x, y van de laserstraal nog omgezet naar een angle en een distance

    _prev_lidar_angles[i-(_lidar_data_length - _sensenumber)]  = atan2(y_new,x_new);

   	if (_prev_lidar_angles[i-(_lidar_data_length - _sensenumber)] > 2*M_PI)
      	    _prev_lidar_angles[i-(_lidar_data_length - _sensenumber)] = _prev_lidar_angles[i-(_lidar_data_length - _sensenumber)] - 2*M_PI;

   	else if (_prev_lidar_angles[i-(_lidar_data_length - _sensenumber)] < 0)
      	    _prev_lidar_angles[i-(_lidar_data_length - _sensenumber)] = _prev_lidar_angles[i-(_lidar_data_length - _sensenumber)] + 2*M_PI;

    _prev_lidar_distances[i-(_lidar_data_length - _sensenumber)]  = sqrt(x_new*x_new + y_new*y_new);

  }
}

void Scanmatcher::artificialLidar(){
  // @@@TODO wel of geen rekening houden met beperkingen LIDAR?
  // eventueel cte aatal metingen -> angles wordt const vector.
  // welke hoek is theta
  std::vector<double> _laser_distances(_lidar_data_length,_max_sense_range);
  std::vector<double> _laser_angles;

  _laser_angles.reserve(_lidar_data_length);
  double angle;
  double intersect_distance;

  for(int i = 0; i < _lidar_data_length; i++){
    angle = 2*M_PI*i/_lidar_data_length + _start_pose[2];
    _laser_angles[i] = angle;
  
    for (unsigned int c = 0; c < _environment_circles.size(); c++){

      intersect_distance = getIntersectDistanceCircle(_environment_circles[c], angle);

    	if (intersect_distance < _laser_distances[i]){
    		_laser_distances[i] = intersect_distance;
    	}
    }

    for (unsigned int p = 0; p < _environment_polygons.size(); p++){
      Polygon polygon = _environment_polygons[p];
      int polygonSize = polygon.getSize();
      for (int w = 0; w < polygonSize; w++){

        intersect_distance = getIntersectDistanceLine( polygon.getXVertex(w),  
        	polygon.getYVertex(w), polygon.getXVertex((w+1)%polygonSize), 
          polygon.getYVertex((w+1)%polygonSize), angle);

        if (intersect_distance < _laser_distances[i]){
          _laser_distances[i] = intersect_distance;
        }
      }
    }

    std::cout << "scan :" << i << "; angle :" << angle*180/3.141592 << "; distance :" << _laser_distances[i] <<std::endl;
  }
};

double Scanmatcher::getIntersectDistanceLine(double const& x_a, double const& y_a, 
										 double const& x_b, double const& y_b, double const& angle){
	double determinant;
	double tang;
	double x_estimate;
	double y_estimate;
	double x_intersect;
	double y_intersect;
	bool intersection_found;

	x_estimate = _start_pose[0];
	y_estimate = _start_pose[1];

	tang = tan ( angle );
	determinant = (x_a - x_b) * tang - (y_a - y_b);

	if (std::abs(determinant) < _precision){
		intersection_found = false;
	} else{
		
    x_intersect = ( ( y_b - y_estimate + tang * x_estimate ) * ( x_a - x_b ) - 
					   ( y_a - y_b ) * x_b ) / determinant;
		y_intersect = y_estimate + tang * ( x_intersect - x_estimate );

    // intersection in boundaries
		if ( inRange(x_a, x_b, x_intersect) && inRange (y_a, y_b, y_intersect)){
      intersection_found = correctIntersection(x_estimate, y_estimate, x_intersect, y_intersect, angle);
		} else {
			intersection_found = false;
    }
	}

	if ( intersection_found ) {
		return sqrt ( (x_intersect - x_estimate) * (x_intersect - x_estimate) +
					  (y_intersect - y_estimate) * (y_intersect - y_estimate) );
	} else {
		return _max_sense_range;
	}

}

double Scanmatcher::getIntersectDistanceCircle(Circle circle, double const& angle){

	double x_estimate;
	double y_estimate;
	double x_intersect;
  double y_intersect;
  double x_centre;
  double y_centre;
  double radius;
  double discriminant;
  double distance;
  double A;
  double B;
  double C;
  double tangens;
  double intersect_distance;

  x_estimate = _start_pose[0];
  y_estimate = _start_pose[1];
  tangens = tan(angle);
  intersect_distance = _max_sense_range;
  x_centre = circle.getX();
  y_centre = circle.getY();
  radius = circle.getRadius();

  A = tangens*tangens + 1.;
  B = -2.*(tangens*(tangens*x_estimate +y_estimate + y_centre) + x_centre);
  C = (y_centre*y_centre + x_centre*x_centre - radius*radius + 
          (tangens*x_estimate+y_estimate)*(-2*y_centre* + (-tangens*x_estimate - y_estimate)));

  discriminant = B*B - 4*A*C;

  if (greaterThan(0., discriminant)){
    for (int i=0; i<2; i++){
      x_intersect = (-B + pow(-1,i) * sqrt(discriminant))/(2.*A);
      y_intersect = tangens*(x_intersect - x_estimate) - y_estimate;
      if (correctIntersection(x_estimate, y_estimate, x_intersect, y_intersect, angle)){
        distance = sqrt((x_intersect - x_estimate) * (x_intersect - x_estimate) +
              (y_intersect - y_estimate) * (y_intersect - y_estimate));
        if (distance < intersect_distance)
          intersect_distance = distance;
      } 
    }
  }
  return intersect_distance;
}

bool Scanmatcher::inRange(double const& boundarie_1, double const& boundarie_2, 
  double const& value){

  return (((greaterThan(boundarie_1, value)) && (smallerThan(boundarie_2, value))) ||
          ((smallerThan(boundarie_1, value)) && (greaterThan(boundarie_2, value))));
}

bool Scanmatcher::greaterThan(double const& boundarie, double const& value){
  return (value > boundarie - _precision);
}

bool Scanmatcher::smallerThan(double const& boundarie, double const& value){
  return (value < boundarie + _precision);
}

bool Scanmatcher::correctIntersection(double const& x, double const& y, 
  double const& x_intersect, double const& y_intersect, double const& angle){

  bool result;

  if ((inRange(0., M_PI/2., angle)) && (greaterThan(x, x_intersect)) &&
         (greaterThan(y, y_intersect)))
    result = true;
  else if ((inRange(M_PI/2., M_PI, angle)) && (smallerThan(x, x_intersect)) &&
         (greaterThan(y, y_intersect)))
    result = true;
  else if ((inRange(M_PI, M_PI*1.5, angle)) && (smallerThan(x, x_intersect)) &&
         (smallerThan(y, y_intersect)))
    result = true;
  else if ((inRange(M_PI*1.5, M_PI*2., angle)) && (greaterThan(x, x_intersect)) &&
         (smallerThan(y, y_intersect)))
    result = true;
  else
    result = false;

  return result;  
}

void Scanmatcher::loadEnvironment(){
  //std::cout << "loading environment..." << std::endl;
  rapidxml::xml_document<> doc;

  // Read the xml file into a vector
  // TODO generate path automatically
  std::ifstream myfile("/home/michiel/ourbot/orocos/ourbot/Scanmatcher/src/environment.xml");
  std::vector<char> buffer((std::istreambuf_iterator<char>(myfile)), std::istreambuf_iterator<char>());
  buffer.push_back('\0');
  // Parse the buffer using the xml file parsing library into doc
  doc.parse<0>(&buffer[0]);
  // Find our root node
  rapidxml::xml_node<> * root_node = doc.first_node("obstacles");

  // Iterate over Circles
  for (rapidxml::xml_node<> * circle_node = root_node->first_node("circle"); 
          circle_node; circle_node = circle_node->next_sibling("circle")){
    //std::cout << "circle found:" << std::endl;

    double x_centre = atof(circle_node->first_node("x")->value());
    double y_centre = atof(circle_node->first_node("y")->value());
    double radius = atof(circle_node->first_node("radius")->value());
    
    Circle circle = Circle(x_centre,y_centre,radius);
    //std::cout << "  x: " << circle.getX() << ", y: " << circle.getY() << ", r: "<< circle.getRadius() << std::endl;
    _environment_circles.push_back(circle);
  }
  // Iterate over Polygons
  for (rapidxml::xml_node<> * polygon_node = root_node->first_node("polygon"); 
          polygon_node; polygon_node = polygon_node->next_sibling("polygon")){
    //std::cout << "polygon found: " << std::endl;
    Polygon polygon = Polygon();
    int i = 0;
    for (rapidxml::xml_node<> * vertex_node = polygon_node->first_node("vertex");
          vertex_node; vertex_node = vertex_node->next_sibling("vertex")){

      double x_vertex = atof(vertex_node->first_node("x")->value());
      double y_vertex = atof(vertex_node->first_node("y")->value());

      polygon.addVertex(x_vertex, y_vertex);
      //std::cout << "  node" << i << ": x:" << polygon.getXVertex(i) << ", y: " << polygon.getYVertex(i) << std::endl;
      i++;
    }
    _environment_polygons.push_back(polygon);
  }

  std::cout << "Environment loaded!" << std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Scanmatcher)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Scanmatcher)
