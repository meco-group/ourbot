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


//Michiel Honoursproject
//Voornaamste aanpassingen: omgeving inlezen, scanstart_pose gebruiken om artificiele kaart
//    op te bouwen, scanmatch resultaat wordt ongeldig verklaard indien positie buiten kaart is of
//    de sprong te groot is.
//Configurations: Scanmatcher: precision       1e-3
//                system: control_sample_rate  100.
//                        pathupd_sample_rate  2.
//                        io_sample_rate       200.
//                        reporter_sample_rate 10.
//                teensy: velocity_controller_P 0.25
//                        velocity_controller_I 2.
//                        velocity_controller_D 0.025

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
  //Michiel event port to trigger scanmatcher
  ports()->addEventPort("scanstart_pose_port", _scanstart_pose_port).doc("Pose at start of a scan");

  // OutputPorts
 	ports()->addPort( "scanmatch_pose_port", _scanmatch_pose_port).doc("Estimated position-change of scanmatcher in encoder-form [x,y,orientation]");
	ports()->addPort( "scanmatch_covariance_port", _scanmatch_covariance_port).doc("covariance scanmatcher [x,y,orientation]");
  //Michiel
  ports()->addPort( "artificial_lidar_distances_port", _artificial_lidar_distances_port).doc("Calculated artificial lidar distances");
  ports()->addPort( "artificial_lidar_angles_port", _artificial_lidar_angles_port).doc("Calculated artificial lidar angles");

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

  //Michiel
  loadEnvironment();

  std::cout << "Scanmatcher constructed !" <<std::endl;
}

bool Scanmatcher::configureHook(){
  _rays = _lidar_data_length - _sensenumber;
	_available_lidar_distances.resize(_lidar_data_length);
	_available_lidar_angles.resize(_lidar_data_length);
	_lidar_distances.resize(_rays);
	_lidar_angles.resize(_rays);

  _start_pose.resize(3);

  //Michiel
  _artificial_lidar_angles.resize(_rays);
  _artificial_lidar_distances.resize(_rays);
  for (int i = 0; i < _rays; i++){
    _artificial_lidar_angles[i] = 2*M_PI*i/(_rays);
  }
  std::fill(_artificial_lidar_distances.begin(), _artificial_lidar_distances.end(), _max_sense_range);

  //Michiel setDataSample to report
  _scanmatch_pose_port.setDataSample(_estimated_position_change);
  std::vector<double> example(9, 0.0);
  _scanmatch_covariance_port.setDataSample(example);
  _artificial_lidar_distances_port.setDataSample(_artificial_lidar_distances);
  _artificial_lidar_angles_port.setDataSample(_artificial_lidar_angles);

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
  
  std::cout << "Scanmatcher configured !" <<std::endl;
  return true;
}

bool Scanmatcher::startHook(){
  std::cout << "Scanmatcher started !" <<std::endl;
  return true;
}

void Scanmatcher::updateHook(){

  //std::cout << "Scanmatcher updateHook !" <<std::endl;
  
  int rays = _rays;
	sm_result result;

	// local scope array to put in ls-struct
	ls.laser_sens = ld_alloc_new(rays);
	ls.laser_ref = ld_alloc_new(rays*_scans);


  ls.first_guess[0] = 0;
  ls.first_guess[1] = 0;
  ls.first_guess[2] = 0;
  
  // Reading data from ports
  _cor_lidar_distance_port.read(_available_lidar_distances);
  _cor_lidar_angle_port.read(_available_lidar_angles);

  _scanstart_pose_port.read(_start_pose);
  //Michiel testing the start pose can improve robustness
  _correct_scanstart_pose = 1;
  //_correct_scanstart_pose = (inRange(-1.22, 1.22, _start_pose[0]) && inRange(-1.22, 1.22, _start_pose[1]));
  if ((_start_pose[0] != _start_pose[0]) || (_start_pose[1] != _start_pose[1])){
    std::cout << "nan as startpose" << std::endl;
  }

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

    //Michiel
    artificialLidar();

    //std::cout << "artificial Lidar data created!" <<std::endl;

    //Michiel
  	double anglesref[rays*_scans];
    double distancesref[rays*_scans];
    for(int i= 0; i<rays*_scans; i++){
        distancesref[i] = _artificial_lidar_distances[i];
        anglesref[i] = _artificial_lidar_angles[i];
    }
    for(int i=0; i< rays; i++){
      ls.laser_ref->readings[i] = distancesref[i];
      ls.laser_ref->theta[i] = anglesref[i];
      ls.laser_ref->valid[i] = _correct_scanstart_pose;
      //Michiel nu wordt gekeken of de startpositie zich in domein bevindt,
      //Zou aangepast kunnen worden naar startpositie binnen obstakel, ...
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

    //Michiel gezorgd dat de uitkomst van de scanmatcher geen al te grote sprongen maakt
    if((result.valid == 1) && (result.x[0] < 0.35) && (result.x[1] < 0.35)){
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
    		  _covariance[i*_rows+j] = 0.0001;
          //Michiel 0.0001 ipv 0.1 otherwise NAN, because of almost singular matrix
    		}
    	}
    }
    //std::cout << "scanstart: x: " << _start_pose[0] << "; y: " << _start_pose[1] << "; theta: " << _start_pose[2] << "; read;"<< std::endl;
    //std::cout << result.valid << "; dx: " << _estimated_position_change[0] << "; dy: " << _estimated_position_change[1] << "; dt: " <<_estimated_position_change[2] << std::endl;

    // set data on ports
    _scanmatch_covariance_port.write(_covariance);
    _scanmatch_pose_port.write(_estimated_position_change);

    //Michiel
    _artificial_lidar_distances_port.write(_artificial_lidar_distances);
    _artificial_lidar_angles_port.write(_artificial_lidar_angles);

    ld_free(ls.laser_ref);
    ld_free(ls.laser_sens);
  }
  else {
    fprintf(stderr, "Scanmatcher got all zeros!");
  }
  
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

void Scanmatcher::artificialLidar(){
  //Michiel
  //rekening gehouden met beperkingen LIDAR (_max_sense_range, _lidar_data_length)
  double angle;
  double intersect_distance;

  std::fill(_artificial_lidar_distances.begin(), _artificial_lidar_distances.end(), _max_sense_range);

  //Michiel
  //itereren over alle hoeken en telkens dichtste snijpunt zoeken
  //geen rekening gehouden met (on)mogelijkheid dat de ourbot zich in een obstakel bevindt
  for(int i = 0; i < _rays; i++){
    angle = 2*M_PI*i/(_rays) + _start_pose[2];
  
    for (unsigned int c = 0; c < _environment_circles.size(); c++){

      intersect_distance = getIntersectDistanceCircle(_environment_circles[c], angle);

    	if (intersect_distance < _artificial_lidar_distances[i]){
    		_artificial_lidar_distances[i] = intersect_distance;
    	}
    }

    for (unsigned int p = 0; p < _environment_polygons.size(); p++){
      Polygon polygon = _environment_polygons[p];
      int polygonSize = polygon.getSize();
      for (int w = 0; w < polygonSize; w++){

        intersect_distance = getIntersectDistanceLine( polygon.getXVertex(w),  
        	polygon.getYVertex(w), polygon.getXVertex((w+1)%polygonSize), 
          polygon.getYVertex((w+1)%polygonSize), angle);

        if (intersect_distance < _artificial_lidar_distances[i]){
          _artificial_lidar_distances[i] = intersect_distance;
        }
      }
    }
  }
};

double Scanmatcher::getIntersectDistanceLine(double x_a, double y_a, double x_b, double y_b, double angle){
  //Michiel berekent afstand dichtste snijpunt op verbindingslijn tussen twee punten
  //indien geen snijpunt, _max_sense_range
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

	if (std::abs(determinant) < 0.){
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

double Scanmatcher::getIntersectDistanceCircle(Circle circle, double angle){
  //Michiel berekent afstand dichtste snijpunt met cirkel
  //indien geen snijpunt, _max_sense_range
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
  B = -2.*(tangens*(tangens*x_estimate - y_estimate + y_centre) + x_centre);
  C = -radius*radius + x_centre*x_centre + 
            (y_estimate - y_centre - tangens*x_estimate)*(y_estimate - y_centre - tangens*x_estimate);

  discriminant = B*B - 4.*A*C;

  if (discriminant>0.){
    for (int i=0; i<2; i++){
      x_intersect = (-B + pow(-1,i) * sqrt(discriminant))/(2.*A);
      y_intersect = tangens*(x_intersect - x_estimate) + y_estimate;
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

bool Scanmatcher::inRange(double boundarie_1, double boundarie_2, double value){
//Michiel hulpfunctie, kijkt of value tussen boundaries valt, rekening houdend met precisie (config)
  return (((greaterThan(boundarie_1, value)) && (smallerThan(boundarie_2, value))) ||
          ((smallerThan(boundarie_1, value)) && (greaterThan(boundarie_2, value))));
}

bool Scanmatcher::greaterThan(double boundarie, double value){
  //Michiel hulpfunctie, kijkt of value groter is dan boundarie, rekening houdend met precisie (config)
  return (value > boundarie - _precision);
}

bool Scanmatcher::smallerThan(double boundarie, double value){
  //Michiel hulpfunctie, kijkt of value kleiner is dan boundarie, rekening houdend met precisie (config)
  return (value < boundarie + _precision);
}

double Scanmatcher::clip(double boundarie_1, double boundarie_2, double value){
  //Michiel hulpfunctie, clipt value
  if (value < boundarie_1)
    return boundarie_1;
  else if (value > boundarie_2)
    return boundarie_2;
  else
    return value;
}

bool Scanmatcher::correctIntersection(double x, double y, double x_intersect, double y_intersect, double angle){
  //Michiel checkt of het snijpunt zich in de juiste richting bevindt
  bool result = false;

  double corrected_angle = fmod(angle, 2.*M_PI);

  if ((inRange(0., M_PI/2., corrected_angle)) && (greaterThan(x, x_intersect)) &&
         (greaterThan(y, y_intersect)))
    result = true;
  if ((inRange(M_PI/2., M_PI, corrected_angle)) && (smallerThan(x, x_intersect)) &&
         (greaterThan(y, y_intersect)))
    result = true;
  if ((inRange(M_PI, M_PI*1.5, corrected_angle)) && (smallerThan(x, x_intersect)) &&
         (smallerThan(y, y_intersect)))
    result = true;
  if ((inRange(M_PI*1.5, M_PI*2., corrected_angle)) && (greaterThan(x, x_intersect)) &&
         (smallerThan(y, y_intersect)))
    result = true;

  return result;  
}

void Scanmatcher::loadEnvironment(){
  //Michiel loads environment
  //std::cout << "loading environment..." << std::endl;
  rapidxml::xml_document<> doc;

  std::ifstream myfile("/home/odroid/orocos/Scanmatcher/src/environment.xml");
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
