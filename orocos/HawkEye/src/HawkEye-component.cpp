#include "HawkEye-component.hpp"

HawkEye::HawkEye(std::string const& name) : TaskContext(name, PreOperational), _drawstar{0}, _drawmods{0}{

#ifndef HAWKEYE_TESTFLAG
  //Add properties
  addProperty("device", _device).doc("Video device driver");
  addProperty("resolution", _resolution).doc("Resolution");
  addProperty("fps",    _fps).doc   ("Frames per second");
  addProperty("nRobots", _nRobots).doc("Number of robots which are present")

#else //if in Test-mode
  _device = "/dev/video1"; //locate video device on Odroid
  _resolution = HD720p;
  _fps = 90;   //max for this resolution, if using USB 3.0
  _nRobots = 1; //Amount of robots in field
  _workspace_path = "/home/tim/orocos/ourbot/orocos"; //TODO: adapt to right path //directory where Orocos component is situated in
  _brightness = 10; //0...40
  _exposure = 30; //1...10000
  _iso = 200; //

#endif //HAWKEYE_TESTFLAG

  ports()->addPort("obstacles_state_port",_obstacles_state_port).doc("Obstacles state output port");
  ports()->addPort("robots_state_port",   _robots_state_port).doc("Robots state output port"); //We will have multiple robots
  ports()->addPort("width_port",          _width_port).doc("Frame width output port");
  ports()->addPort("height_port",         _height_port).doc("Frame height output port");
  ports()->addPort("fps_port",            _fps_port).doc("Frames per second output port");

  HAWKEYE_DEBUG_PRINT("HawkEye constructed!")
}

bool HawkEye::configureHook(){

  //Set operating flags and constants
  _save_image = true;  //record images/get new background at start
  _load_background_from_file = false; //select if you want to use an old background or make a new one
  _draw_markers = true; // draw detected and computed markers from templated matching
  _draw_contours = false; //draw detected obstacles and their contours
  _print_cam_info = true; //print camera capabilities while starting the camera
  
  _cntapprox = 0.025; //parameter of approxPolyDP which approximates a polygon/contour by another, simplified, polygon/contour
  _diffthresh = 30; //threshold for background subtraction for captured image vs background: determines diff image
  _matchThresh = 0.7; //threshold for template matching

  //Initialize class variables
  _save_img_path = _workspace_path + "/HawkEye/src/Images/"; //path where to save captured images
  _path = _workspace_path + "/HawkEye/src/";

  //Read in templates of robot markers
  HAWKEYE_DEBUG_PRINT("circle template path: "<<_path+"Images/templates/mod1.tiff")
  _template_circle= cv::imread(_path+"Images/templates/mod1.tiff", CV_LOAD_IMAGE_GRAYSCALE);
  HAWKEYE_DEBUG_PRINT("template_circle type: "<<_template_circle.type()) // 0 = CV_8U, see: http://ninghang.blogspot.be/2012/11/list-of-mat-type-in-opencv.html
  if(! _template_circle.data )                              // Check for invalid input
  {
      RTT::log(RTT::Error)<<"Could not open or find the circle template"<<RTT::endlog();
      return false;
  }
  _template_circlehollow= cv::imread(_path+"Images/templates/circlehollow.tiff",CV_LOAD_IMAGE_GRAYSCALE);
  if(! _template_circlehollow.data )                              // Check for invalid input
  {
      RTT::log(RTT::Error)<<"Could not open or find the hollow circle template"<<RTT::endlog();
      return false;
  }
  _template_star1= cv::imread(_path+"Images/templates/star3.tiff",CV_LOAD_IMAGE_GRAYSCALE);
  if(! _template_star1.data )                              // Check for invalid input
  {
      RTT::log(RTT::Error)<<"Could not open or find the star template"<<RTT::endlog();
      return false;
  }
  _template_star2= cv::imread(_path+"Images/templates/star3rot.tiff",CV_LOAD_IMAGE_GRAYSCALE);
  if(! _template_star2.data )                              // Check for invalid input
  {
      RTT::log(RTT::Error)<<"Could not open or find the starrot template"<<RTT::endlog();
      return false;
  }
  _template_cross= cv::imread(_path+"Images/templates/crosscircle.tiff",CV_LOAD_IMAGE_GRAYSCALE);
  if(! _template_cross.data )                              // Check for invalid input
  {
      RTT::log(RTT::Error)<<"Could not open or find the crosscircle template"<<RTT::endlog();
      return false;
  }
  _template_cross_rot= cv::imread(_path+"Images/templates/crosscirclerot.tiff",CV_LOAD_IMAGE_GRAYSCALE);
  if(! _template_cross_rot.data )                              // Check for invalid input
  {
      RTT::log(RTT::Error)<<"Could not open or find the crosscirclerot template"<<RTT::endlog();
      return false;
  }

  setResolution(_resolution); //load resolution into class variables
  checkFPS(_resolution); //check if selected fps is compatible with the resolution

  // Show example data sample to output ports to make data flow real-time
  int example = 0;
  std::vector<double> exampleObstacle(80, 0.0); //Todo: 8 inputs per obstacle required --> suppose you have 10 obstacles maximum
  std::vector<double> exampleRobot(27, 0.0); //Todo: 9 inputs per robot required --> suppose you have 3 robots maximum
  _obstacles_state_port.setDataSample(exampleObstacle);
  _robots_state_port.setDataSample(exampleRobot);
  _width_port.setDataSample(example);
  _height_port.setDataSample(example);
  _fps_port.setDataSample(example);

  HAWKEYE_DEBUG_PRINT("HawkEye configured!")

  return true;
}

bool HawkEye::startHook(){
  
  //Set-up camera
  startCamera();

  setBrightness(_brightness); //set camera brightness
  setExposure(_exposure); //set camera exposure
  setISO(_iso); //set camera setISO

  //Capture background and save for further use in backgroundSubtraction()
  getBackground(); 

  HAWKEYE_DEBUG_PRINT("HawkEye started!")
  return true;
}

void HawkEye::updateHook(){
  
  HAWKEYE_DEBUG_PRINT("Starting processImage")
  
  processImage(); //grab image, find patterns, discern obstacles from robot, save contours
  
  HAWKEYE_DEBUG_PRINT("Starting processResults")
  
  processResults(); //process contours into obstacles
  
  HAWKEYE_DEBUG_PRINT("Starting writeResults")
  
  writeResults(); //write results to output ports

  if (HAWKEYE_SAVE){ 
    HAWKEYE_DEBUG_PRINT("Starting drawResults")
    drawResults(); //save images of image processing
  }

  HAWKEYE_DEBUG_PRINT("HawkEye executes updateHook!")
}

void HawkEye::stopHook() {

  // reset the component so we don't get into trouble when restarting the component
  reset();

  close(_fd); //Stop the camera

  HAWKEYE_DEBUG_PRINT("HawkEye executes stopping!")
}

void HawkEye::pabort(const char *s){   //error catching
    RTT::log(RTT::Error) <<s<<", error string: "<<std::strerror(_errno)<<RTT::endlog();
}

int HawkEye::xioctl(int fd, int request, void *arg){ //adapted ioctl implementation for image capturing
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

void HawkEye::startCamera(){ //Open file descriptor, set resolution, initialize buffers. Adapted from v4l2_app of e-con systems

    //Set file descriptor
    _fd = open(_device.c_str(), O_RDWR); //open video device on Odroid, _device = /dev/video0 normally (or video1 if webcam present)
    if (_fd == -1){
      pabort("Error while opening video device");
    }

    //Print camera capabilities
    if (_print_cam_info == true){
      struct v4l2_capability caps = {};
      if (-1 == xioctl(_fd, VIDIOC_QUERYCAP, &caps)){
              pabort("Error while querying Capabilities");
      }

      printf( "Driver Caps:\n"
              "  Driver: \"%s\"\n"
              "  Card: \"%s\"\n"
              "  Bus: \"%s\"\n"
              "  Version: %d.%d\n"
              "  Capabilities: %08x\n",
              caps.driver,
              caps.card,
              caps.bus_info,
              (caps.version>>16)&&0xff,
              (caps.version>>24)&&0xff,
              caps.capabilities);

      struct v4l2_cropcap cropcap;
      cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl (_fd, VIDIOC_CROPCAP, &cropcap)){
              pabort("Querying Cropping Capabilities");
      }

      printf( "Camera Cropping:\n"
              "  Bounds: %dx%d+%d+%d\n"
              "  Default: %dx%d+%d+%d\n"
              "  Aspect: %d/%d\n",
              cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
              cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
              cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
    }

    //Print format description
    int support_grbg10 = 0; 
    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    while (0 == xioctl(_fd, VIDIOC_ENUM_FMT, &fmtdesc)){
      strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
      if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
          support_grbg10 = 1;
      c = fmtdesc.flags & 1? 'C' : ' ';
      e = fmtdesc.flags & 2? 'E' : ' ';
      HAWKEYE_DEBUG_PRINT("Format: "<<fourcc)
      HAWKEYE_DEBUG_PRINT("CE description: "<<c<<e<<fmtdesc.description)
      fmtdesc.index++;
    }

    //Set data format and resolution
    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = _width; //were already assigned in the configureHook
    fmt.fmt.pix.height = _height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;//V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (-1 == xioctl(_fd, VIDIOC_S_FMT, &fmt)){ 
        pabort("Error while setting Pixel Format");
    }

    if (_print_cam_info){
      strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
      printf( "Selected Camera Mode:\n"
              "  Width: %d\n"
              "  Height: %d\n"
              "  PixFmt: %s\n"
              "  Field: %d\n",
              fmt.fmt.pix.width,
              fmt.fmt.pix.height,
              fourcc,
              fmt.fmt.pix.field);
    }

    //Initialize buffers
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
 
    if (-1 == xioctl(_fd, VIDIOC_REQBUFS, &req)){
        pabort("Error while requesting Buffer");
    }
 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(_fd, VIDIOC_QUERYBUF, &buf)){
        pabort("Error while querying Buffer");
    }
 
    _buffer = (uint8_t*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, buf.m.offset); //mmap: create a "virtual" memory map for the whole file, without actually reading it into memory
    HAWKEYE_DEBUG_PRINT("Length: "<<buf.length)
    HAWKEYE_DEBUG_PRINT("Address: "<<_buffer)
    HAWKEYE_DEBUG_PRINT("Image length: "<<buf.bytesused)

    //Prepare for capturing images 
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(_fd, VIDIOC_QBUF, &buf)){
        pabort("Error while querying buffer");
    }
 
    if(-1 == xioctl(_fd, VIDIOC_STREAMON, &buf.type)){
        pabort("Error while starting capture");
    }
}

void HawkEye::setResolution(resolution_t resolution) {

    //Set resolution and check if the choice is valid
    switch (resolution){
      case LOW: 
        _width = 672;
        _height = 380;
        HAWKEYE_DEBUG_PRINT("Selected resolution: 672x380")
        break;
      case HD720p: 
        _width = 1280;
        _height = 720;
        HAWKEYE_DEBUG_PRINT("Selected resolution: 1280x720")
        break;
      case HD1080p:
        _width = 1920;
        _height = 1080; 
        HAWKEYE_DEBUG_PRINT("Selected resolution: 1920x1080")
        break;
      case FULL: 
        _width = 2688;
        _height = 1520; 
        HAWKEYE_DEBUG_PRINT("Selected resolution: 2688x1520")
        break;
      default: 
        RTT::log(RTT::Error) << "Invalid resolution selected, you selected: "<<resolution<<". The possiblities are: LOW(672x380) , HD720(1280x720) , HD1080(1920x1080), FULL(2688x1520)."<< RTT::endlog();
    }
}

void HawkEye::setBrightness(int brightness){ //From Qtcam source code Videostreaming::changeSettings()
    HAWKEYE_DEBUG_PRINT("Setting brightness to: "<<brightness)
    if (brightness < 0 || brightness > 40){
      RTT::log(RTT::Error) << "Invalid brightness selected, you selected: "<<brightness<<", but this value must lie in the interval [0,40]"<< RTT::endlog(); 
    }
    struct v4l2_control c;
    c.id = V4L2_CID_BRIGHTNESS;
    c.value = brightness;    
    if (-1 == ioctl(_fd, VIDIOC_S_CTRL, &c)) { //VIDIOC_G_CTRL, VIDIOC_S_CTRL — Get or set the value of a control
        pabort("Error in setting the value of the brightness");        
    }
  }

void HawkEye::setExposure(int exposure){ //From Videostreaming::changeSettings()
  HAWKEYE_DEBUG_PRINT("Setting exposure to: "<<exposure)
  if (exposure < 1 || exposure > 10000){
      RTT::log(RTT::Error) << "Invalid exposure selected, you selected: "<<exposure<<", but this value must lie in the interval [1,10000]"<< RTT::endlog(); 
  }
  struct v4l2_control c;
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = 0 ; //manual
  if (-1 == ioctl(_fd, VIDIOC_S_CTRL, &c)) { //VIDIOC_G_CTRL, VIDIOC_S_CTRL — Get or set the value of a control
      pabort("Error in setting the exposure to manual");        
  }
  c.id = V4L2_CID_EXPOSURE;
  c.value = exposure;   
  HAWKEYE_DEBUG_PRINT("Right before setting exposure") 
  if (-1 == ioctl(_fd, VIDIOC_S_CTRL, &c)) { //VIDIOC_G_CTRL, VIDIOC_S_CTRL — Get or set the value of a control
      pabort("Error in setting the value of the exposure");        
  }
}

void HawkEye::setISO(int iso){ //From Videostreaming::changeSettings()
  HAWKEYE_DEBUG_PRINT("Setting ISO to: "<<iso)
  if (iso < 100 || iso > 2500){
      RTT::log(RTT::Error) << "Invalid ISO selected, you selected: "<<iso<<", but a reasonable interval is [100,2500]"<< RTT::endlog(); 
  }
  struct v4l2_control c;
  c.id = V4L2_CID_ISO_SENSITIVITY_AUTO;
  c.value = 0; //manual, 1=auto
  if (-1 == ioctl(_fd, VIDIOC_S_CTRL, &c)) { //VIDIOC_G_CTRL, VIDIOC_S_CTRL — Get or set the value of a control
      pabort("Error in setting iso to manual");        
  }
  c.id = V4L2_CID_ISO_SENSITIVITY;
  c.value = iso;   
  HAWKEYE_DEBUG_PRINT("Right before setting iso") 
  if (-1 == ioctl(_fd, VIDIOC_S_CTRL, &c)) { //VIDIOC_G_CTRL, VIDIOC_S_CTRL — Get or set the value of a control
      pabort("Error in setting the iso");        
  }
}

void HawkEye::checkFPS(resolution_t resolution) {

    //Check if _fps for selected resolution is not too high
    if (resolution == LOW){
      if (_fps > 256){
        RTT::log(RTT::Error)<<"For the selected resolution: "<<_width<<"x"<<_height<<", "<<_fps<<"fps is too high, maximum value is 256 for USB 3.0."<<RTT::endlog();
      }
      else{
        HAWKEYE_DEBUG_PRINT("Checked FPS: the selected "<<_fps<< " frames per second are appropriate for the selected resolution")
      }
    }

    if (resolution == HD720p){
      if (_fps > 90){
        RTT::log(RTT::Error)<<"For the selected resolution: "<<_width<<"x"<<_height<<", "<<_fps<<"fps is too high, maximum value is 90 for USB 3.0."<<RTT::endlog();
      } 
      else{
        HAWKEYE_DEBUG_PRINT("Checked FPS: the selected "<<_fps<< " frames per second are appropriate for the selected resolution")
      }
    }

    if (resolution == HD1080p){
      if (_fps > 40){
        RTT::log(RTT::Error)<<"For the selected resolution: "<<_width<<"x"<<_height<<", "<<_fps<<"fps is too high, maximum value is 40 for USB 3.0."<<RTT::endlog();
      }
      else{
        HAWKEYE_DEBUG_PRINT("Checked FPS: the selected "<<_fps<< " frames per second are appropriate for the selected resolution")
      }
    }

    if (resolution == FULL){
      if (_fps > 14){
        RTT::log(RTT::Error)<<"For the selected resolution: "<<_width<<"x"<<_height<<", "<<_fps<<"fps is too high, maximum value is 14 for USB 3.0."<<RTT::endlog();
      }
      else{
        HAWKEYE_DEBUG_PRINT("Checked FPS: the selected "<<_fps<< " frames per second are appropriate for the selected resolution")
      }
    }
}

void HawkEye::bayer10_to_rgb24(uint16_t *pBay, uint8_t *pRGB24, int width, int height, int pix_order){ //Convert 10bit raw to RGB. Adapted from v4l2_app from e-con systems
//This function translates a bayer 10 bit RAW datatype to RGB data. The CMOS sensor captures a certain color in each element of its matrix, so you have to combine elements to get RGB
//values. This algorithm uses a nearest neighbour to translate Bayer10 bit RAW to RGB, but there are better solutions. What this algorithm does is actually interpolating
//between several pixels, which gives a bad resolution and the impression that pixels are grouped per 2x2. 
#define B(a,x,y) a[0 + 3 * ((x) + width * (y))]
#define G(a,x,y) a[1 + 3 * ((x) + width * (y))]
#define R(a,x,y) a[2 + 3 * ((x) + width * (y))]
/*clip value between 0 and 255*/
#define CLIP(value) (uint8_t)(((value)>0xFF)?0xff:(((value)<0)?0:(value)))
//#define CLIP(x) ((x < 0) ? 0 : ((x > 255) ? 255 : x))

#define Bay(a,x, y) a[(x) + width * (y)]
  int x = 0,y = 0;
  //B-G-IR-R Nearest Neighbor
  for(x = 0; x < width; x += 2){
    for(y = 0; y < height; y += 2){
      B(pRGB24,x, y) = B(pRGB24,x + 1, y) = B(pRGB24,x, y + 1) = B(pRGB24,x + 1, y + 1) = CLIP(Bay(pBay,x, y));// - Bay(x, y + 1));
      G(pRGB24,x, y) = G(pRGB24,x + 1, y) = G(pRGB24,x, y + 1) = G(pRGB24,x + 1, y + 1) = CLIP(Bay(pBay,x + 1, y));// - Bay(x, y + 1));
      R(pRGB24,x, y) = R(pRGB24,x + 1, y) = R(pRGB24,x, y + 1) = R(pRGB24,x + 1, y + 1) = CLIP(Bay(pBay,x + 1, y + 1));// - Bay(x, y + 1));
    }
  }  
}

void HawkEye::capture_image(){ //save the current image in _f
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;  //tv_sec is number of whole seconds of elapsed time, so amount of seconds to wait?
    int r = select(_fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r){
        pabort("Error while waiting for frame");
    }
 
    //Todo: _capture time should be set here? or after xioctl?
    _capture_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
    if(-1 == xioctl(_fd, VIDIOC_DQBUF, &buf)){
        pabort("Error while retrieving frame");
    }
    cv::Mat m_RGB(cv::Size(_width, _height), CV_8UC3);
    
    bayer10_to_rgb24(reinterpret_cast<uint16_t*>(_buffer), m_RGB.data, _width, _height, 2); //Todo: is this a new buffer or _buffer?

    HAWKEYE_DEBUG_PRINT("RGB channels: "<<m_RGB.channels())
    cv::Mat grayImg( cv::Size(_width,_height), CV_8UC1 );
    if (!m_RGB.empty() && !grayImg.empty()){
      cv::cvtColor(m_RGB , grayImg, CV_BGR2GRAY);
      HAWKEYE_DEBUG_PRINT("saving image")
      HAWKEYE_DEBUG_PRINT("m_RGB type: "<<m_RGB.type())
    }
    cv::imwrite("image_rgb.tiff", m_RGB);
    cv::imwrite("image_gray.tiff", grayImg);
    _f = m_RGB; //save as current frame _f

    cv::resize(_f, _f, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST); //Todo: improve this, now we capture at double resolution and resize to half the size because pixels are grouped per 4
    // _f = grayImg; //save as current frame _f
    HAWKEYE_DEBUG_PRINT("current frame channels: "<<_f.channels())
    HAWKEYE_DEBUG_PRINT("current frame type: "<<_f.type())

    cv::imwrite("image_rgb_rescaled.tiff", _f);
    //to grayscale
    cv::Mat grayImg2( _f.size(), CV_8UC1 );
    cvtColor(_f,grayImg2,CV_RGB2GRAY);
    cv::imwrite("image_gray_rescaled.tiff", grayImg2);


    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(_fd, VIDIOC_QBUF, &buf)){
        pabort("Error while querying buffer");
    }
}

void HawkEye::getBackground() //save background as average of 50 captured images
{
    //read frames from camera and take average of multiple background frames
    capture_image(); //update _f to current frame

    if (_load_background_from_file){
        _background = cv::imread(_path + "Images/background.png", CV_LOAD_IMAGE_COLOR);
        if (!_background.data){ //If the image cannot be read, the function returns an empty matrix ( Mat::data==NULL )
          RTT::log(RTT::Error)<<"You tried to load a background from a file with the _load_background_from_file boolean, but I could not open or find "<<_path<<"Images/background.png"<<RTT::endlog();
        }
    }
    else{    
        _background.create(_f.size(),CV_32FC3); //RGB image

        //loop over images and estimate background
        for (int i = 0; i < 50; ++i){
            capture_image(); //update _f to current frame
            cv::accumulateWeighted(_f,_background,0.1);
        }
        HAWKEYE_DEBUG_PRINT("Average aquired over 50 frames")
        _background.convertTo(_background, CV_8UC3); //convert background to uint8_t presentation
    }
        
    if (_save_image){
        cv::imwrite(_path+"Images/background.png",_background); //save new background
    }
}

void HawkEye::processImage()
{

  //Clear class variables of type array from previous iteration
  _boxcontours.clear();
  _rectanglesDetectedContours.clear();
  _rectanglesDetected.clear();
  _boxes.clear();
  _boxes_correct.clear();
  _circles.clear();
  _circles_correct.clear();
  _circlesDetected.clear();
  _roboboxes.clear();
  _robots.clear();
  _obstacles.clear();

  std::vector<std::vector<cv::Point> > contours; //will hold all contours
  std::vector<cv::Vec4i> hierarchy; //<Vec4i> is a vector of integers: [x1,y1,x2,y2]
  HAWKEYE_DEBUG_PRINT("Starting background subtraction")
  
  backgroundSubtraction(&contours, &hierarchy); //find obstacle contours

  HAWKEYE_DEBUG_PRINT("Finished background subtraction")
  if (hierarchy.empty()){
    HAWKEYE_DEBUG_PRINT("No contours were found...")
  }

  if (!hierarchy.empty()){
    std::vector<cv::Point> c; //holds a contour
    cv::Rect rectangle; 
    double area; //rectangle or circle area
    cv::RotatedRect rotrect; //minimum area rectangle around contour
    cv::Point2f ccenter; //circle center
    float cradius; //circle radius

    double cx;
    double cy;
    cv::Size fsize;

    int rorigx; 
    int rorigy; 
    int rorigh; 
    int rorigw; 
    int roriArray[4];
    // First find the robots
    for (int c_i = 0; c_i < contours.size(); c_i++){
        cv::convexHull(contours[c_i], c); //convex hull of the detected obstacle/contour
        rectangle = cv::boundingRect(c); //rectangle around contour: rectangle.x, y, width, height
        area = cv::contourArea(c);
        // HAWKEYE_DEBUG_PRINT("Contour area: "<<area);
        rotrect= cv::minAreaRect(c); //minimum area rectangle around contour

        if (area>900){
            fsize = _f.size();             

            rorigx= rectangle.x;
            rorigy= rectangle.y;
            rorigh= rectangle.height;
            rorigw= rectangle.width;
            roriArray[0] = rorigx;
            roriArray[1] = rorigy;
            roriArray[2] = rorigh;
            roriArray[3] = rorigw;
            _rorig.assign(roriArray,roriArray + 4); 
            _roi = _f(rectangle);

            HAWKEYE_DEBUG_PRINT("Starting to find robots")
            findRobots();  // in each contour which is large enough try to find the robot
            // if success{blank out contour?}
        }
    }

    // Then find the obstacles
    for (int c_i = 0; c_i < contours.size(); c_i++){ //take first element of hierarchy and combine with first contour etc.
        cv::convexHull(contours[c_i], c); //convex hull of the detected obstacle/contour
        rectangle = cv::boundingRect(c); //rectangle around contour: rectangle.x, y, width, height
        area = cv::contourArea(c);
        // HAWKEYE_DEBUG_PRINT("Contour area: "<<area);
        rotrect= cv::minAreaRect(c); //minimum area rectangle around contour
        cv::minEnclosingCircle(c, ccenter, cradius); //minimum area circle around contour

        // add small sized objects to object contours
        if (area>10 and area<140){ //shuttle-sized obstacles
            _boxcontours.push_back(c);
        }

        // add medium sized objects to object contours
        if (area>=140 and area<=900){
            // simplify contour and add to box contours              
            cv::approxPolyDP(c, c, _cntapprox*cv::arcLength(c,true), true); // approximate contours: the function approxPolyDP approximates a curve or a polygon with another curve/polygon with less vertices so that the distance between them is less or equal to the specified precision. 
            _boxcontours.push_back(c);
        }
        
        // process large objects: this is where the robot will be detected
        // double cx;
        // double cy;
        // cv::Size fsize;

        // int rorigx; 
        // int rorigy; 
        // int rorigh; 
        // int rorigw; 
        // int roriArray[4];

        if (area>900){// and hier[3]==-1: //Todo: adapt value for new camera
            cx = ccenter.x;
            cy = ccenter.y;
            fsize = _f.size();             

            // integer coordinates only
            cx = static_cast<int>(cx);
            cy = static_cast<int>(cy);
            ccenter.x = cx;
            ccenter.y = cy;
            cradius = static_cast<int>(cradius);

            rorigx= rectangle.x;
            rorigy= rectangle.y;
            rorigh= rectangle.height;
            rorigw= rectangle.width;
            roriArray[0] = rorigx;
            roriArray[1] = rorigy;
            roriArray[2] = rorigh;
            roriArray[3] = rorigw;
            _rorig.assign(roriArray,roriArray + 4); 
            _roi = _f(rectangle);

            // findRobots();  // in each contour which is large enough try to find the robot
            // todo: skip this if already found robot(s)?

            findBigObstacles(rotrect, c, cx, cy, cradius, &contours, &hierarchy);  
        }
        else{
          HAWKEYE_DEBUG_PRINT("No contours with area > 900 found")
        }
    }
  }
}

void HawkEye::backgroundSubtraction(std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy){
    double read_time = 0;
    // while (read_time < 1/(_fps)){//while read_time is too low, i.e., it's buffered... discard the buffered one
        // _capture_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        capture_image(); //update _f to current frame
        read_time =  difftime(time(0) , _capture_time); //Todo: not used? Time it took to read image
    // }    

    if (_save_image){
        HAWKEYE_DEBUG_PRINT("Saving current frame")
        HAWKEYE_DEBUG_PRINT(_save_img_path + "1img-" + std::to_string(_capture_time) + ".png")
        cv::imwrite(_save_img_path + "1img-" + std::to_string(_capture_time) + ".png" ,_f);
    }

    cv::absdiff(_background, _f, _diff);  //Todo: why on RGB? Background subtraction on RGB images
    cv::cvtColor(_diff, _diff , CV_RGB2GRAY); //convert _diff to grayscale 
    
    if (_save_image){
        HAWKEYE_DEBUG_PRINT("Saving current difference with background")
        cv::imwrite(_save_img_path + "2diff-" + std::to_string(_capture_time) + ".png",_diff);
    }
    cv::threshold(_diff, _mask,_diffthresh, 255, cv::THRESH_BINARY); //gives a black/white image: compared background to captured frame and makes pixels which differ enough from background black (= obstacle)
                                                                  //+cv::THRESH_OTSU: didn't seem to work better 
    if (_save_image){
        HAWKEYE_DEBUG_PRINT("Saving current mask")
        cv::imwrite(_save_img_path + "3mask-" + std::to_string(_capture_time) + ".png",_mask);
    }

    if (HAWKEYE_PLOT){ //Plot results of background subtraction
        HAWKEYE_DEBUG_PRINT("Showing current frame, difference and mask")
        cv::imshow("unprocessed", _f);
        cv::imshow("diffimage", _diff);
        cv::imshow("mask",_mask);
        cv::waitKey(); //display the image
    } 

    _maskcopy = _mask.clone();//Todo: is this correct or does mask change now if copy changes?

    HAWKEYE_DEBUG_PRINT("Starting to look for contours")

    cv::findContours(_maskcopy, *contours, *hierarchy, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE); //find contours in the mask
}

void HawkEye::findRobots() //match templates of robot
{    
    bool success;
    //Todo: put in vector<double> or in double[array]? Vector is dynamic datatype, but size is known, so use array.
    double robottocks[7] = {0}; //maxpoints((x,y),(x,y)), w, h, max_val --> positions of circular markers
    double starpat[5] = {0};    //temploc(x,y), w, h, max_val --> position of star marker
    double crosspat[5] = {0};    //temploc(x,y), w, h, max_val --> position of starfull marker
    double circlehollowpat[5] = {0};    //temploc(x,y), w, h, max_val --> position of circle marker
    printedMatch(_roi, _template_circle, _template_star1, _template_star2, _template_cross, _template_cross_rot, _template_circlehollow, &success, robottocks, starpat, crosspat, circlehollowpat, _matchThresh, _draw_markers, _rorig);
    HAWKEYE_DEBUG_PRINT("printedMatch completed")
    // Add robots
    if (success == true){
        // We found a robot, now decide which one
        if (starpat[4] != 0){ //if a hollow star was detected (max_val != 0)
            HAWKEYE_DEBUG_PRINT("Found robot with hollow star pattern!")
            HAWKEYE_DEBUG_PRINT("max_val starpat in findrobots():"<<starpat[4])
            addRobot(robottocks, starpat); //further processing to get coordinates and robot direction, save in a robot instance
            HAWKEYE_DEBUG_PRINT("Added robot with hollow star pattern!")
        }
        if (crosspat[4] != 0){ //if a cross star was detected (max_val != 0)
            HAWKEYE_DEBUG_PRINT("Found robot with cross pattern!")
            HAWKEYE_DEBUG_PRINT("max_val crosspat in findrobots():"<<crosspat[4])
            addRobot(robottocks, crosspat); //further processing to get coordinates and robot direction, save in a robot instance
            HAWKEYE_DEBUG_PRINT("Added robot with cross pattern!")
        }
        if (circlehollowpat[4] != 0){ //if a hollow circle was detected (max_val != 0)
            HAWKEYE_DEBUG_PRINT("Found robot with hollow circle pattern!")
            HAWKEYE_DEBUG_PRINT("max_val circlehollow in findrobots():"<<circlehollowpat[4])
            addRobot(robottocks, circlehollowpat); //further processing to get coordinates and robot direction, save in a robot instance
            HAWKEYE_DEBUG_PRINT("Added robot with hollow circle pattern!")
        }  
    }
    else{
      HAWKEYE_DEBUG_PRINT("No robots were found")
    }

}

void HawkEye::addRobot(double *robottocks, double *pattern)
{
  //determine robot direction from detected markers
    HAWKEYE_DEBUG_PRINT("Starting to calculate robot center and orientation")
    HAWKEYE_DEBUG_PRINT("robottocks: "<<robottocks[0]<<", "<<robottocks[1]<<", "<<robottocks[2]<<", "<<robottocks[3])
    try{
        double lineside = (robottocks[2] - robottocks[0])*(pattern[1]-robottocks[1]) - (robottocks[3] - robottocks[1])*(pattern[0] - robottocks[0]); //(circle2_x-circle1_x)*(star_y-circle1_y) - (circle2_y-circle1_y)*(star_x-circle1_x)
        double moddirection = 180/M_PI*(std::atan2((robottocks[3]-robottocks[1]),(robottocks[2]-robottocks[0])));
        double robdirection;
        if (lineside>0){
            robdirection = moddirection + 90; //we need the direction of the orthogonal to the line of the mod points
        }
        else{
            robdirection = moddirection - 90;
        }
            
        double b1 = robottocks[1]-(std::tan(M_PI/180*moddirection)*robottocks[0]);
        double b2 = pattern[1]- (std::tan(M_PI/180*robdirection)*pattern[0]);
        double xmodcen = (b1-b2)/(std::tan(M_PI/180*robdirection)-std::tan(M_PI/180*moddirection));
        double ymodcen = ((std::tan(M_PI/180*moddirection)*xmodcen)+b1);
        
        robdirection += 90; // north should be 0
        if (robdirection < 0){
            robdirection += 360;   
        }
        
        //Todo: adapt to ourBot scale, scale is used to determine the reference point on the youbot. pattern is the location of the pattern. xrobcen and yrobcen are computed by adding to the position of the star a certain scale times the distance between the star and the midpoint of the two circles. In reality, this distance is 100 mm. The distance from the center of the star to the front of the plate on which the markers are placed is 75.9 mm, and the distance from the front of this plate to the center of the youbot (midpoint between the 4 wheels) is 40.0 mm. This 40.0 mm is only a rough measurement done by Kurt Geebelen and can be off a few milimeters. If you do not trust this, you can remeasure it and propose to change it.
        double scale = (40.0 + 75.9)/100.0;
        double xrobcen= pattern[0] + std::abs(pattern[0] - xmodcen) * scale * std::sin(M_PI/180 * robdirection);
        double yrobcen= pattern[1] - std::abs(pattern[1] - ymodcen) * scale * std::cos(M_PI/180 * robdirection);
        double robcen[2] = {xrobcen + _rorig[0], yrobcen + _rorig[1]};
        cv::Size fsize = _f.size(); 

        if (_draw_markers){
            cv::circle(_f, cv::Point(static_cast<int>(robcen[0]),static_cast<int>(robcen[1])), 2, cv::Scalar(255,0,255), 4);
        }

        //Gather results
        cv::RotatedRect robobox;
        robobox = cv::RotatedRect(cv::Point2f(robcen[0],robcen[1]), cv::Size2f(80,130), robdirection); //fixed size of box: [x,y,w,h,theta]
        _roboboxes.push_back(robobox);

        HAWKEYE_DEBUG_PRINT("Making new robot instance")
        //Build robot instance
        Robot *robot = new Robot();
        HAWKEYE_DEBUG_PRINT("Made new robot instance")
        robot->setPos(robobox.center.x, robobox.center.y);
        robot->setVel(0, 0); //Todo: combine with Kalman
        robot->setWidth(robobox.size.width);
        robot->setLength(robobox.size.height);
        robot->setRadius(0); //Todo: fill in right value
        robot->setTheta(robobox.angle);
        robot->setOmega(0); //Todo: combine with Kalman
        robot->setRoi(_rorig[0], _rorig[1]);
        robot->setMarker(robottocks, pattern);
        HAWKEYE_DEBUG_PRINT("Pushing new robot instance")
        _robots.push_back(robot);
    }  
    catch (const std::exception &e){
        HAWKEYE_DEBUG_PRINT("No direction and center can be calculated!") 
        HAWKEYE_DEBUG_PRINT(e.what())
    }
}

void HawkEye::findBigObstacles(cv::RotatedRect rotrect, std::vector<cv::Point> c, int cx, int cy, int cradius, std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy)
{

    HAWKEYE_DEBUG_PRINT("In findBigObstacles")
    HAWKEYE_DEBUG_PRINT("width and height of rotatedrectangle: "<<rotrect.size.width<<", "<<rotrect.size.height)
    HAWKEYE_DEBUG_PRINT("center of rotatedrectangle: "<<rotrect.center.x<<", "<<rotrect.center.y)

    //If there is a large obstacle (rotrect) without markers, i.e. if the robot center is not inside the box, then you have a big obstacle --> add contours to boxcontour list
    cv::Point2f box[4];
    rotrect.points(box);
    std::vector<cv::Point> boxPoints;  //
    for (int k = 0 ; k < 4 ; k++){
      boxPoints.push_back(box[k]);
    }

    HAWKEYE_DEBUG_PRINT("Number of robot boxes in findBigObstacles: "<<_roboboxes.size())
    // No robots were found, add obstacles
    if (_roboboxes.size() == 0){
        HAWKEYE_DEBUG_PRINT("Found obstacle with area > 900 and there are no robots")
        _boxes.push_back(rotrect); //save rectangle representation of obstacle
        _boxcontours.push_back(c); //rectangle was an obstacle, so add it to contours
        std::vector<double> current_circle; //set up a vector to push in circles
        current_circle.push_back(cx); 
        current_circle.push_back(cy); 
        current_circle.push_back(cradius);
        _circles.push_back(current_circle); //save circle representation of obstacle
    }
    // Only add obstacles which are not the robot
    else{  // Todo: the obstacle can still be inside the robot and not contain its center point, use a better check!
        double isrobotbox[_roboboxes.size()];
        bool insideRobot = false;
        for(int k = 0; k<_roboboxes.size(); k++) {  //check all robots   
            isrobotbox[k]= cv::pointPolygonTest(boxPoints, _roboboxes[k].center, false); // only add boxes that don't contain the robot //i.e. test if robot center lies in box
            if (isrobotbox[k] > 0){  // returns positive value if robobox center is inside the contour
                insideRobot = true;  // this contour overlaps with robot k
                mergeContourWithRobot(_roboboxes[k], boxPoints, contours, hierarchy); //rectangle/box contained the robot, so delete its contour from the mask = draw the contour on the mask (via drawContours)
            }
        }
        if (insideRobot == false){  //contour overlaps with none of the robots
            HAWKEYE_DEBUG_PRINT("Found obstacle with area > 900 which is not the robot")
            _boxes.push_back(rotrect); //save rectangle representation of obstacle
            _boxcontours.push_back(c); //rectangle was an obstacle, so add it to contours
            std::vector<double> current_circle; //set up a vector to push in circles
            current_circle.push_back(cx); 
            current_circle.push_back(cy); 
            current_circle.push_back(cradius);
            _circles.push_back(current_circle); //save circle representation of obstacle
        }
    }
}

void HawkEye::mergeContourWithRobot(cv::RotatedRect robobox, std::vector<cv::Point> contourPoints, std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy){
    cv::Point2f roboboxcontour[4];
    robobox.points(roboboxcontour);  //select the robot which the contour overlaps with
    std::vector<cv::Point> roboboxcontourPoints;
    for (int k = 0 ; k < 4 ; k++){
      roboboxcontourPoints.push_back(roboboxcontour[k]);
    }
    if ( cv::contourArea(contourPoints) > cv::contourArea(roboboxcontourPoints) ){ //if area of new contour is bigger than previous robobox area, merge boxes
        HAWKEYE_DEBUG_PRINT("--------merged boxes--------") //since cv::drawContours updates your mask
        //Todo: move drawing code outside main algo
        std::vector<std::vector<cv::Point> > roboboxVectorPoints;
        roboboxVectorPoints.push_back(roboboxcontourPoints);
        cv::drawContours(_mask, roboboxVectorPoints, 0, cv::Scalar(0,0,0), -1); //draw robot contour on the mask = update the mask
        
        if (HAWKEYE_PLOT){
          cv::imshow("cleared mask", _mask); //plot the updated mask
        }

        cv::findContours(_mask, *contours, *hierarchy, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE); //find object in the remaining contours, to see if there were overlapping or touching objects with the robot box
        
        //Todo: this will only return obstacles touching the original robot box?
        if (hierarchy != NULL){
            for (int c_i = 0; c_i < contours->size(); c_i++){
                // std::vector<Point> c = cv::convexHull(contours[c_i]); //Todo: was there in previous loop of this kind and now not anymore?
                std::vector<cv::Point> c = contours->at(c_i);
                cv::Rect rectangle = cv::boundingRect(c); //rectangle around contour: rectangle.x, y, width, height
                _boxcontours.push_back(c);
                int area = rectangle.width * rectangle.height;
                cv::RotatedRect rotrect= cv::minAreaRect(c);
                cv::Point2f ccenter;
                float cradius; 
                cv::minEnclosingCircle(c, ccenter, cradius); //circle around contour   
    
                if (area > 10){// and hier[3]==-1: //minimum area to see something as an obstacle
                    int cx = ccenter.x;
                    int cy = ccenter.y;
                    cv::Size fsize = _f.size(); 
                    std::vector<double> current_circle; //set up a vector to push in circles
                    current_circle.push_back(cx); 
                    current_circle.push_back(cy); 
                    current_circle.push_back(cradius);
                    _circles_correct.push_back(current_circle); //save circle representation of contour
                    _boxes_correct.push_back(rotrect); //save rectangle representation of contour
                }
            }
        }
    }
}

void HawkEye::processResults(){
  
  if ( (_circles_correct.size())>0 ){
      HAWKEYE_DEBUG_PRINT("circles_correct size: "<<_circles_correct.size()<<" boxes_correct size: "<<_boxes_correct.size())
      _circlesDetected =_circles_correct;
  }
  else {
      // HAWKEYE_DEBUG_PRINT("boxes_correct size: "<<_boxes_correct.size())
      HAWKEYE_DEBUG_PRINT("_circles size: "<<_circles.size())
      _circlesDetected =_circles;      
  }

  if (_boxes_correct.size() > 0){
      HAWKEYE_DEBUG_PRINT(" boxes_correct size: "<<_boxes_correct.size()<<"circles_correct size: "<<_circles_correct.size())
      _rectanglesDetected = _boxes_correct;
  }
  else {
      // HAWKEYE_DEBUG_PRINT("circles_correct size: "<<_circles_correct.size())
      HAWKEYE_DEBUG_PRINT("_boxes size: "<<_boxes.size())
      _rectanglesDetected = _boxes; //flip //TODO: put this back if you flip it correctly
  }

  //Todo: print timings correctly
  // HAWKEYE_DEBUG_PRINT("Time offsets from capture [ms]: encoded %6.1f -- received %6.1f -- decoded %6.1f\n" % ((meta["timestamp_encoded"]-meta["timestamp_grabbed"])*1000,(meta["timestamp_received"]-meta["timestamp_grabbed"])*1000,(meta["timestamp_decoded"]-meta["timestamp_grabbed"])*1000);
  // result['time']=meta["timestamp_grabbed"] //the most accurate one, gives the time at which the image was taken (without influence of encoding, sending, decoding)

  HAWKEYE_DEBUG_PRINT("in processResults")
  HAWKEYE_DEBUG_PRINT("rectangles_detected_size: "<<_rectanglesDetected.size())
  HAWKEYE_DEBUG_PRINT("circles_detected_size: "<<_circlesDetected.size())

  // for (int k=0; k<_roboboxes.size(); k++){
  //     if (_roboboxes[k].size.width != 0 && (_rectanglesDetected.size()) > 0){ //replaced _robobox != NULL by this because _robobox initialized with size = (0,0)
  //         _roboboxes[k].points(roboboxcontour);
  //         for (int j = 0 ; j < 4 ; j++){
  //           roboboxcontourPoints.push_back(roboboxcontour[j]); //add robobox vertices to contour points
  //         }
  //         for (int j = 0 ; j < _rectanglesDetected.size() ; j++){
  //             double isboxwithinrobot= cv::pointPolygonTest(roboboxcontourPoints, _rectanglesDetected[j].center, false); //only add boxes that are not within the robot
  //             if (isboxwithinrobot < 0){ //not within robot and not yet added
  //                 filter_boxes.push_back(_rectanglesDetected[j]); //save rectangle in filtered boxes
  //             }
  //         }
  //     }
  // }

  cv::Point2f roboboxcontour[4];
  std::vector<cv::Point> roboboxcontourPoints;
  std::vector<cv::RotatedRect> filter_boxes;
  bool toAdd = false;
  for (int j = 0 ; j < _rectanglesDetected.size() ; j++){
      toAdd = false;
      for (int k=0; k<_roboboxes.size(); k++){
          _roboboxes[k].points(roboboxcontour);
          for (int l = 0 ; l < 4 ; l++){
              roboboxcontourPoints.push_back(roboboxcontour[l]); //add robobox vertices to contour points
          }
          double isboxwithinrobot= cv::pointPolygonTest(roboboxcontourPoints, _rectanglesDetected[j].center, false); //only add boxes that are not within the robot
          if (isboxwithinrobot < 0){ //not within robot and not yet added
              toAdd = true;
          }
      }
      if (toAdd == true){
          filter_boxes.push_back(_rectanglesDetected[j]); //save rectangle in filtered boxes
      }
      roboboxcontourPoints.clear();
  }


  HAWKEYE_DEBUG_PRINT("filter_boxes.size(): "<<filter_boxes.size())
  if (filter_boxes.size() > 0){ // if-loop was executed for at least one of the roboboxes
    _rectanglesDetected = filter_boxes; //overwrite detected rectangles vector with filtered version
  }
  HAWKEYE_DEBUG_PRINT("rectangles_detected_size: "<<_rectanglesDetected.size())

  if (_draw_contours){ //this part only needs to be executed if _draw_contours is true

    _rectanglesDetectedContours = _boxcontours;   

    std::vector<std::vector<cv::Point> > filter_objcontours;
    cv::Point2f roboboxcontour[4];
    std::vector<cv::Point> roboboxcontourPoints;
    cv::Moments rectangleMoments;
    for (int k = 0; k<_roboboxes.size(); k++){
        if (_roboboxes[k].size.width != 0 && _rectanglesDetectedContours.size() > 0){
            _roboboxes[k].points(roboboxcontour);
            for (int j = 0 ; j < 4 ; j++){
              roboboxcontourPoints.push_back(roboboxcontour[j]);
            }
            double iscontwithinrobot;
            for (int j = 0 ; j < _rectanglesDetectedContours.size() ; j++){
                std::vector<std::vector<cv::Point> > conthull ( _rectanglesDetectedContours.size() ); //Todo: or put this _rectanglesDetectedContours.size()?
                cv::convexHull(_rectanglesDetectedContours[j], conthull[j]); // avoid butterfly contours as they have m00= 0
                rectangleMoments = cv::moments(conthull[j]);
                if (rectangleMoments.m00!=0){
                    cv::Point2f contcen(static_cast<int>(rectangleMoments.m10/rectangleMoments.m00), static_cast<int>(rectangleMoments.m01/rectangleMoments.m00));
                    iscontwithinrobot= cv::pointPolygonTest(roboboxcontourPoints, contcen, false);    // only add boxes that are not within the robot
                }
                else{
                    iscontwithinrobot = -1;
                }
                if (iscontwithinrobot < 0){
                    cv::approxPolyDP(_rectanglesDetectedContours[j], _rectanglesDetectedContours[j], _cntapprox*cv::arcLength(_rectanglesDetectedContours[j],true), true); // approximate contours
                    filter_objcontours.push_back(_rectanglesDetectedContours[k]); //add contours which are not within the robot
                }
            }
        }
    }
    if (filter_objcontours.size()>0){
        HAWKEYE_DEBUG_PRINT("filtered the object contours")
        _rectanglesDetectedContours = filter_objcontours;
    }
  }

  //Process results to put on data port

  //All robots are already in _robots

  //For each obstacle decide if it is best represented by a circle or by a rectangle
  // _obstacles.clear(); //No, this will cause a memory leak since the pointers to the object will be deleted, but not the objects themselves
  //But it seems like vector.clear() also clears the objects, so you can still use this. An alternative is:
  // for (int k = 0 ; k < _obstacles.size() ; k++) //since we used new below
  // {
  //   HAWKEYE_DEBUG_PRINT("k: "<<k)
  //   HAWKEYE_DEBUG_PRINT("Deleting previous obstacles")
  //   delete _obstacles[k]; 
  // }

  // _obstacles.resize(0);

  for (int k = 0 ; k < _rectanglesDetected.size() ; k++){ //size of rectanglesDetected and circlesDetected is the same
    HAWKEYE_DEBUG_PRINT("size of circlesDetected: "<<_circlesDetected.size())
    HAWKEYE_DEBUG_PRINT("size of rectanglesDetected: "<<_rectanglesDetected.size())
    HAWKEYE_DEBUG_PRINT("k: "<<k)
    double circleArea = pi * _circlesDetected[k][2] * _circlesDetected[k][2];
    double rectangleArea = _rectanglesDetected[k].size.width * _rectanglesDetected[k].size.height;
    HAWKEYE_DEBUG_PRINT("circleArea: "<<circleArea)
    HAWKEYE_DEBUG_PRINT("rectangleArea: "<<rectangleArea)
    Circle *circle = new Circle(); 
    Rectangle *rectangle = new Rectangle();
    if (circleArea < rectangleArea){
      HAWKEYE_DEBUG_PRINT("Processresults: new circle pushed")
      circle->setPos(_circlesDetected[k][0], _circlesDetected[k][1]);
      circle->setRadius(_circlesDetected[k][2]); //radius
      circle->setArea(circleArea);
      _obstacles.push_back(circle);
      delete rectangle; //declared in the loop but was not used
    }
    else {
      HAWKEYE_DEBUG_PRINT("Processresults: new rectangle pushed")

      HAWKEYE_DEBUG_PRINT("Rectangle width: "<<_rectanglesDetected[k].size.width<<" Rectangle length: "<<_rectanglesDetected[k].size.height)

      rectangle->setPos(_rectanglesDetected[k].center.x, _rectanglesDetected[k].center.y);
      rectangle->setWidth(_rectanglesDetected[k].size.width);
      rectangle->setLength(_rectanglesDetected[k].size.height);
      rectangle->setTheta(_rectanglesDetected[k].angle);
      rectangle->setArea(rectangleArea);
      _obstacles.push_back(rectangle);
      delete circle; //declared in the loop but was not used
    }
  }

HAWKEYE_DEBUG_PRINT("constructed obstacle vector")

}

void HawkEye::writeResults(){
  
  //Convert Robot and Obstacle objects to a vector
  std::vector<double> robotVec; //(27) but then this makes a 27*0 vector first and then the real robot if you use push_back in Robot.cpp

  HAWKEYE_DEBUG_PRINT("Calling obj2vec for robot")

  for(int k = 0; k<_robots.size() ; k++) {
    Robot::obj2vec(*_robots[k], &robotVec);
  }

  std::vector<double> obstacleVec; //(80) 80 = 8*10 = number of entrances per obstacle * a max of 10 obstacles
  std::vector<double> tmpObstacleVec(10);
  tmpObstacleVec.clear();

  HAWKEYE_DEBUG_PRINT("Calling obj2vec for obstacles")
  HAWKEYE_DEBUG_PRINT("size of _obstacles: "<<_obstacles.size())
  for (int k = 0 ; k < _obstacles.size() ; k++){
    HAWKEYE_DEBUG_PRINT("shape of obstacle: "<<int(_obstacles[k]->getShape()))
    if(_obstacles[k]->getShape() == CIRCLE){
      Circle::obj2vec(*static_cast<Circle*>(_obstacles[k]), &tmpObstacleVec);
      for (int j = 0 ; j < 7 ; j++){ //Todo: check if 7 is right
        HAWKEYE_DEBUG_PRINT("Pushing back circle")
        obstacleVec.push_back(tmpObstacleVec[j]);  
      }
      for (int j = 0 ; j < 3 ; j++){ //fill remaining spaces with zeros, in order to get 10 entries for both circles and rectangles
        HAWKEYE_DEBUG_PRINT("Filling rest of circle vector with zeros")
        obstacleVec.push_back(0); 
      }
    }
    if(_obstacles[k]->getShape() == RECTANGLE){
      Rectangle::obj2vec(*static_cast<Rectangle*>(_obstacles[k]), &tmpObstacleVec);
      for (int j = 0 ; j < 10 ; j++){ //Todo: check if 10 is right
        HAWKEYE_DEBUG_PRINT("Pushing back rectangle attributes into obstacleVec")
        HAWKEYE_DEBUG_PRINT("elements: "<<tmpObstacleVec[j])
        obstacleVec.push_back(tmpObstacleVec[j]);
      }
    }    
  }

  obstacleVec.resize(80); //resize to 80 elements
  robotVec.resize(27); //resize to 27 elements, i.e. 9 elements per robot, max 3 robots

  //Todo: check if size of obstacleVec is not exceeded?

  HAWKEYE_DEBUG_PRINT("Starting to write data to the Orocos ports")

  //Put data on output ports
  _robots_state_port.write(robotVec);
  _obstacles_state_port.write(obstacleVec);
  // _width_port.write(data);
  // _height_port.write(data);
  // _fps_port.write(data);

  // Empty robot vector
  // for (int k = 0 ; k < _robots.size() ; k++) //since we used new below
  // {
  //   HAWKEYE_DEBUG_PRINT("k: "<<k)
  //   HAWKEYE_DEBUG_PRINT("Deleting current robots")
  //   delete _robots[k]; 
  // }
  // _robots.resize(0);

  //Todo: also empty _obstacles here instead of above?

}

void HawkEye::drawResults(){
        
    HAWKEYE_DEBUG_PRINT("Plotting obstacles")
    int circleRadius;
    cv::Point2i circleCenter;

    cv::Point2i rectanglePt1; //holds bottom left vertex
    cv::Point2i rectanglePt2; //holds top right vertex

    for (int k = 0 ; k < _obstacles.size(); k++){
        if(_obstacles[k]->getShape() == CIRCLE){

          circleRadius = static_cast<Circle*>(_obstacles[k])->getRadius();
          circleCenter.x = _obstacles[k]->getPos()[0];
          circleCenter.y = _obstacles[k]->getPos()[1];
          cv::circle(_f, circleCenter, circleRadius, cv::Scalar(0,0,255), 2);

        }
        if(_obstacles[k]->getShape() == RECTANGLE){

          //Plot non-rotated rectangle around obstacle
          // rectanglePt1.x = _obstacles[k]->getPos()[0] - static_cast<Rectangle*>(_obstacles[k])->getWidth()/2.0; //bottom left vertex
          // rectanglePt1.y = _obstacles[k]->getPos()[1] - static_cast<Rectangle*>(_obstacles[k])->getLength()/2.0;

          // rectanglePt2.x = _obstacles[k]->getPos()[0] + static_cast<Rectangle*>(_obstacles[k])->getWidth()/2.0; //top right vertex
          // rectanglePt2.y = _obstacles[k]->getPos()[1] + static_cast<Rectangle*>(_obstacles[k])->getLength()/2.0;
          // cv::rectangle(_f, rectanglePt1, rectanglePt2, cv::Scalar(128,200,255), 2);

          //Plot rotated rectangle around obstacle
          cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(_obstacles[k]->getPos()[0],_obstacles[k]->getPos()[1]), cv::Size2f(static_cast<Rectangle*>(_obstacles[k])->getWidth(),static_cast<Rectangle*>(_obstacles[k])->getLength()), static_cast<Rectangle*>(_obstacles[k])->getTheta());
          cv::Point2f vertices[4];
          rRect.points(vertices);
          for (int k = 0; k < 4; k++){
            cv::line(_f, vertices[k], vertices[(k+1)%4], cv::Scalar(0,0,255), 2);
          }
        }
    }    

    HAWKEYE_DEBUG_PRINT("Plotting robots")
    cv::Point2f roboboxPoints[4];
    for(int k = 0; k<_roboboxes.size(); k++) {
      _roboboxes[k].points(roboboxPoints);
      HAWKEYE_DEBUG_PRINT(roboboxPoints[0])
      HAWKEYE_DEBUG_PRINT(roboboxPoints[1])
      HAWKEYE_DEBUG_PRINT(roboboxPoints[2])
      HAWKEYE_DEBUG_PRINT(roboboxPoints[3])
      for( int k = 0; k < 4; k++ ){
        cv::line( _f, roboboxPoints[k], roboboxPoints[(k+1)%4], cv::Scalar(0,255,0), 2, 8 );
      }
    }

    //Draw markers
    HAWKEYE_DEBUG_PRINT("Plotting markers")
    if (_draw_markers){
        Robot::marker robotMarker;
        double* bottomMarkers;
        double* topMarker;
        int* roi;
        for(int k = 0; k<_robots.size(); k++) {
            // robotMarker =  _robots[k].getMarkers();
            bottomMarkers = _robots[k]->getBottomMarkers();
            topMarker = _robots[k]->getTopMarker();
            roi = _robots[k]->getRoi();
            //Bottom markers
            HAWKEYE_DEBUG_PRINT("Plotting bottom markers")
            for (int k = 0 ; k <= 1 ; k++ ){
                HAWKEYE_DEBUG_PRINT("pos x and y: "<<bottomMarkers[2*k]<<" , "<<bottomMarkers[2*k+1])
                HAWKEYE_DEBUG_PRINT("radius: "<<bottomMarkers[4])
                cv::circle(_f, cv::Point(bottomMarkers[2*k]+roi[0], bottomMarkers[2*k+1]+roi[1]), bottomMarkers[4]/2.0, cv::Scalar(255,0,0), 2);
            }
            //Top markers
            HAWKEYE_DEBUG_PRINT("Plotting top markers")
            HAWKEYE_DEBUG_PRINT("pos x and y: "<<topMarker[0]<<" , "<<topMarker[1])
            HAWKEYE_DEBUG_PRINT("radius: "<<topMarker[2])
            cv::circle(_f, cv::Point(topMarker[0]+roi[0], topMarker[1]+roi[1]), topMarker[2]/2.0, cv::Scalar(255,255,0), 2);
        }
    }

    if (_draw_contours){
        HAWKEYE_DEBUG_PRINT("Plotting contours of rectangles")     
        cv::drawContours(_f , _rectanglesDetectedContours , -1 , cv::Scalar(128,200,255) , 2); 
    }

    if (_save_image){
        HAWKEYE_DEBUG_PRINT("Saving final image with markers")
        cv::imwrite(_save_img_path + "4final-" + std::to_string(_capture_time) + ".png" , _f);    
    }
}

void HawkEye::printedMatch(cv::Mat roi, cv::Mat template_circle, cv::Mat template_star1, cv::Mat template_star2, cv::Mat template_cross, cv::Mat template_cross_rot, cv::Mat template_circlehollow, bool *success, double *robottocks, double *starpat, double *crosspat, double *circlehollowpat, float matchThresh, bool draw_markers, std::vector<int> rorig){ //Todo: adapt input: should become varType varName
    
    cv::Mat image = roi; //image to examine is limited to roi

    bool star = false; 
    bool cross = false;
    bool circlehollow = false;  
    bool mods = false;

    int rorigx = rorig[0]; //region of interest midpoint [x,y] , width, height [w,h] --> from recognized rectangle
    int rorigy = rorig[1];
    int rorigw = rorig[2];
    int rorigh = rorig[3];        
    
    cv::Size temp_circle_size = template_circle.size(); 
    cv::Size temp_circlehollow_size = template_circlehollow.size();
    cv::Size temp_star1_size = template_star1.size(); 
    cv::Size temp_star2_size = template_star2.size();
    cv::Size temp_cross_size = template_cross.size();
    cv::Size temp_cross_rot_size = template_cross_rot.size();
    
    // only process when roi is larger than the template
    if (rorigw > temp_circle_size.width && rorigh > temp_circle_size.height){ //Todo: selected right dimensions? Or flip width and height? 
        // Check for mods bottom markers
        double max_val = 0;
        std::vector<int> maxpoints;
        try{
            multiObject(image, template_circle, matchThresh, &temp_circle_size.width, &temp_circle_size.height, &max_val, &maxpoints); //returns most probable position, width and height of robot circle markers (=mod)
            
            HAWKEYE_DEBUG_PRINT("Maxpoints in printedMatch: "<<maxpoints[0]<<" , "<<maxpoints[1]<<" , "<<maxpoints[2]<<" , "<<maxpoints[3])
            robottocks[0] = maxpoints[0]; //[x1,y1,x2,y2,w,h,max_val]
            robottocks[1] = maxpoints[1];
            robottocks[2] = maxpoints[2];
            robottocks[3] = maxpoints[3];
            robottocks[4] = temp_circle_size.width;
            robottocks[5] = temp_circle_size.height;
            // robottocks[6] = max_val;

            //Todo: translated this correctly? robottocks[6] = max_value was forgotten in original code? This is the value of the first template which did not reach the threshold --> no info!
            robottocks[0] = robottocks[0]+robottocks[4]/2; // the coords of the should be at the centre of the pattern //calculate pos, w, h of marker
            robottocks[1] = robottocks[1]+robottocks[5]/2;
            robottocks[2] = robottocks[2]+robottocks[4]/2;
            robottocks[3] = robottocks[3]+robottocks[5]/2;

            // HAWKEYE_DEBUG_PRINT("Mod max score: "<<robottocks[6])
            HAWKEYE_DEBUG_PRINT("circle1 pos x: "<<(robottocks)[0]<<" circle1 pos y: "<<(robottocks)[1]<<" circle2 pos x: "<<(robottocks)[2]<<" circle2 pos y: "<<(robottocks)[3])

            mods = true;

            //Blank out the detected circles to avoid false detections of other markers
            cv::RotatedRect templ1 = cv::RotatedRect(cv::Point2f(robottocks[0],robottocks[1]), cv::Size2f(temp_circle_size.width,temp_circle_size.height), 0);
            cv::RotatedRect templ2 = cv::RotatedRect(cv::Point2f(robottocks[2],robottocks[3]), cv::Size2f(temp_circle_size.width,temp_circle_size.height), 0);
            cv::Point2f circleContour1[4];
            cv::Point2f circleContour2[4];
            templ1.points(circleContour1);  //select the robot which the contour overlaps with
            templ2.points(circleContour2);  //select the robot which the contour overlaps with
            std::vector<cv::Point> circlesPoints1;
            std::vector<cv::Point> circlesPoints2;
            for (int k = 0 ; k < 4 ; k++){
                circlesPoints1.push_back(circleContour1[k]);
                circlesPoints2.push_back(circleContour2[k]);
            }
            std::vector<std::vector<cv::Point> > circleVectorPoints1;
            circleVectorPoints1.push_back(circlesPoints1);
            std::vector<std::vector<cv::Point> > circleVectorPoints2;
            circleVectorPoints2.push_back(circlesPoints2);
            cv::drawContours(image, circleVectorPoints1, 0, cv::Scalar(255,255,255), -1); //draw circle template contour on the image, in white
            cv::drawContours(image, circleVectorPoints2, 0, cv::Scalar(255,255,255), -1); //draw circle template contour on the image, in white

        }
        catch(const std::exception &e){
            if (HAWKEYE_PLOT){
              cv::imshow("mods", image);
            }
            //Todo: it seemed like no patterns detected was always printed?
            RTT::log(RTT::Error) << "No mod patterns detected! Error message: "<<e.what()<< RTT::endlog();
            mods = false;
        }
        // Check for star top marker
        double starcand1[5] = {0};
        double max_val1;
        cv::Point temploc1;
        try{
            oneObject(image, template_star1, matchThresh, &temp_star1_size.width, &temp_star1_size.height, &max_val1, &temploc1); //star candidate1
            starcand1[0] = temploc1.x; //x,y,w,h,max_val
            starcand1[1] = temploc1.y;
            starcand1[2] = temp_star1_size.width;
            starcand1[3] = temp_star1_size.height;
            starcand1[4] = max_val1;
            HAWKEYE_DEBUG_PRINT("Star1 max score: "<<std::to_string(starcand1[4]))
            starcand1[0] = starcand1[0]+starcand1[2]/2;
            starcand1[1] = starcand1[1]+starcand1[3]/2;
            starcand1[2] = starcand1[2];
            starcand1[3] = starcand1[3];
            starcand1[4] = starcand1[4]; 
        }
        catch(const std::exception &e){
            // starcand1 = {0, 0, 0, 0, 0};
            if (HAWKEYE_PLOT){
              cv::imshow("star1", image);
            }
            RTT::log(RTT::Error) << "No star1 patterns detected! Error message: "<<e.what()<< RTT::endlog();
        }
        double starcand2[5] = {0};
        double max_val2;
        cv::Point temploc2;
        try{
            oneObject(image, template_star2, matchThresh, &temp_star2_size.width, &temp_star2_size.height, &max_val2, &temploc2);
            starcand2[0] = temploc2.x; //x,y,w,h,max_val
            starcand2[1] = temploc2.y;
            starcand2[2] = temp_star2_size.width;
            starcand2[3] = temp_star2_size.height;
            starcand2[4] = max_val2;
            HAWKEYE_DEBUG_PRINT("Star2 max score: "<<std::to_string(starcand2[4]))
            starcand2[0] = starcand2[0]+starcand2[2]/2;
            starcand2[1] = starcand2[1]+starcand2[3]/2;
            starcand2[2] = starcand2[2];
            starcand2[3] = starcand2[3];
            starcand2[4] = starcand2[4];
        }
        catch(const std::exception &e){
            // starcand2 = {0 , 0 , 0 , 0 , 0};
            if (HAWKEYE_PLOT){
              cv::imshow("star2", image);
            }
            RTT::log(RTT::Error) << "No star2 patterns detected! Error message: "<<e.what()<< RTT::endlog();
        }
        
        HAWKEYE_DEBUG_PRINT("Deciding about type of star pattern")
        if (starcand1[4] > starcand2[4] && starcand1[4] > matchThresh){ //compare scores to decide which is the detected star pattern
            for (int k = 0 ; k <=4 ; k++){
              starpat[k] = starcand1[k];
            }
            star = true;
            HAWKEYE_DEBUG_PRINT("star pos x"<<(starpat)[0]<<"star pos y"<<(starpat)[1])
        }
        else if (starcand1[4] < starcand2[4] && starcand2[4] > matchThresh){
            for (int k = 0 ; k <=4 ; k++){
              starpat[k] = starcand2[k];
            }
            star = true;
            HAWKEYE_DEBUG_PRINT("star pos x"<<(starpat)[0]<<"star pos y"<<(starpat)[1])
        }
        //else{star keeps standard value = false} 

        // Check for starFull top marker
        double crosscand[5] = {0};
        double max_val_cross;
        cv::Point temploc_cross;
        try{
            oneObject(image, template_cross, matchThresh, &temp_cross_size.width, &temp_cross_size.height, &max_val_cross, &temploc_cross); //star candidate1
            crosscand[0] = temploc_cross.x; //x,y,w,h,max_val
            crosscand[1] = temploc_cross.y;
            crosscand[2] = temp_cross_size.width;
            crosscand[3] = temp_cross_size.height;
            crosscand[4] = max_val_cross;
            HAWKEYE_DEBUG_PRINT("cross max score: "<<std::to_string(crosscand[4]))
            crosscand[0] = crosscand[0]+crosscand[2]/2;
            crosscand[1] = crosscand[1]+crosscand[3]/2;
            crosscand[2] = crosscand[2];
            crosscand[3] = crosscand[3];
            crosscand[4] = crosscand[4]; 
        }
        catch(const std::exception &e){
            // starcand1 = {0, 0, 0, 0, 0};
            if (HAWKEYE_PLOT){
              cv::imshow("cross", image);
            }
            RTT::log(RTT::Error) << "No cross patterns detected! Error message: "<<e.what()<< RTT::endlog();
        }
        double crossrotcand[5] = {0};
        double max_val_crossrot;
        cv::Point temploc_crossrot;
        try{
            oneObject(image, _template_cross_rot, matchThresh, &temp_cross_rot_size.width, &temp_cross_rot_size.height, &max_val_crossrot, &temploc_crossrot);
            crossrotcand[0] = temploc_crossrot.x; //x,y,w,h,max_val
            crossrotcand[1] = temploc_crossrot.y;
            crossrotcand[2] = temp_cross_rot_size.width;
            crossrotcand[3] = temp_cross_rot_size.height;
            crossrotcand[4] = max_val_crossrot;
            HAWKEYE_DEBUG_PRINT("crossrot max score: "<<std::to_string(crossrotcand[4]))
            crossrotcand[0] = crossrotcand[0]+crossrotcand[2]/2;
            crossrotcand[1] = crossrotcand[1]+crossrotcand[3]/2;
            crossrotcand[2] = crossrotcand[2];
            crossrotcand[3] = crossrotcand[3];
            crossrotcand[4] = crossrotcand[4];
        }
        catch(const std::exception &e){
            // starcand2 = {0 , 0 , 0 , 0 , 0};
            if (HAWKEYE_PLOT){
              cv::imshow("crossrot", image);
            }
            RTT::log(RTT::Error) << "No crossrot patterns detected! Error message: "<<e.what()<< RTT::endlog();
        }
        
        HAWKEYE_DEBUG_PRINT("Deciding about type of star pattern")
        if (crosscand[4] > crossrotcand[4] && crosscand[4] > matchThresh){ //compare scores to decide which is the detected star pattern
            for (int k = 0 ; k <=4 ; k++){
              crosspat[k] = crosscand[k];
            }
            cross = true;
            HAWKEYE_DEBUG_PRINT("cross pos x"<<(crosspat)[0]<<"cross pos y"<<(crosspat)[1])
        }
        else if (crosscand[4] < crossrotcand[4] && crossrotcand[4] > matchThresh){
            for (int k = 0 ; k <=4 ; k++){
              crosspat[k] = crossrotcand[k];
            }
            cross = true;
            HAWKEYE_DEBUG_PRINT("cross pos x"<<(crosspat)[0]<<"cross pos y"<<(crosspat)[1])
        }
        //else{star keeps standard value = false}
        // Todo: make function of checkstarPattern

        // Check for circleFull marker
        // Check for star top marker
        double circlehollowcand[5] = {0};
        double max_valcirclehollow;
        cv::Point temploccirclehollow;
        try{
            oneObject(image, template_circlehollow, matchThresh, &temp_circlehollow_size.width, &temp_circlehollow_size.height, &max_valcirclehollow, &temploccirclehollow); //star candidate1
            circlehollowcand[0] = temploccirclehollow.x; //x,y,w,h,max_val
            circlehollowcand[1] = temploccirclehollow.y;
            circlehollowcand[2] = temp_circlehollow_size.width;
            circlehollowcand[3] = temp_circlehollow_size.height;
            circlehollowcand[4] = max_valcirclehollow;
            HAWKEYE_DEBUG_PRINT("circlehollow max score: "<<std::to_string(circlehollowcand[4]))
            circlehollowcand[0] = circlehollowcand[0]+circlehollowcand[2]/2;
            circlehollowcand[1] = circlehollowcand[1]+circlehollowcand[3]/2;
            circlehollowcand[2] = circlehollowcand[2];
            circlehollowcand[3] = circlehollowcand[3];
            circlehollowcand[4] = circlehollowcand[4]; 
        }
        catch(const std::exception &e){
            // starcand1 = {0, 0, 0, 0, 0};
            if (HAWKEYE_PLOT){
              cv::imshow("circlehollow", image);
            }
            RTT::log(RTT::Error) << "No hollow circle patterns detected! Error message: "<<e.what()<< RTT::endlog();
        }
        if (circlehollowcand[4] > matchThresh){
            for (int k = 0 ; k <=4 ; k++){
              circlehollowpat[k] = circlehollowcand[k];
            }
            circlehollow = true;
            HAWKEYE_DEBUG_PRINT("starhollow pos x"<<(circlehollowpat)[0]<<"starhollow pos y"<<(circlehollowpat)[1])
        }
    }
    else{
        *success = false; //robot was not detected
    }
    if ((mods == true && star == true) or (mods == true && cross == true) or (mods == true && circlehollow == true)){
        *success = true; //robot was detected
    }
    else{
        *success = false;
    }
}

void HawkEye::oneObject(cv::Mat image, cv::Mat templim, float thresh, int *w, int *h, double *max_val, cv::Point *temploc){ //captured frame, limits from the template you want to match
    cv::Size templim_size = templim.size(); 
    *h = templim_size.height;
    *w = templim_size.width;
    cv::Size image_size = image.size(); 
    cv::cvtColor(image, image , CV_RGB2GRAY); //Convert current frame (i.e. image) to grayscale, for use in matchtemplate (since templim is grayscale)

    //Make a result image, based on the input image and the template (see: http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/template_matching/template_matching.html)
    cv::Mat result;
    int result_cols =  image.cols - templim.cols + 1;
    int result_rows = image.rows - templim.rows + 1;
    result.create( result_rows, result_cols, CV_32FC1 );
    
    double min_val;
    cv::Point min_loc;
    cv::Point max_loc;

    if (*h < image_size.height && *w < image_size.width){ //image is bigger than the template
        HAWKEYE_DEBUG_PRINT("image type: "<<image.type())
        HAWKEYE_DEBUG_PRINT("template type: "<<templim.type())
        cv::matchTemplate(image , templim , result, cv::TM_CCOEFF_NORMED); //match the template
        cv::minMaxLoc(result, &min_val, max_val, &min_loc, &max_loc); //get best matches + their positions e.g. star pattern position of robot
        HAWKEYE_DEBUG_PRINT("match score: "<<std::to_string(*max_val))
        cv::imwrite("inputOneMatch.jpg", image);
        if (*max_val > thresh){ //accept the match
            *temploc = max_loc;
            HAWKEYE_DEBUG_PRINT("Found a top marker!")
            HAWKEYE_DEBUG_PRINT("template pos x: "<<(*temploc).x<<" template pos y: "<<(*temploc).y<<" template width: "<<*w<<" template height: "<<*h)    
            if (HAWKEYE_SAVE){ 
              HAWKEYE_DEBUG_PRINT("Plotting results of star template matching")
              cv::circle(result, cv::Point(max_loc.x + templim.cols/2.0 , max_loc.y + templim.rows/2.0), 4, cv::Scalar(255,255,0), 2); //plot marker location on images
              cv::circle(image,  cv::Point(max_loc.x + templim.cols/2.0 , max_loc.y + templim.rows/2.0), 4, cv::Scalar(255,255,0), 2); //plot marker location on images
              cv::imwrite("resultOneMatch.jpg", result); //Todo: why is this all black? Normed the image 0...1! But image data goes 0...255??
              cv::imwrite("inputOneMatch.jpg", image); 
              cv::imwrite("templateOneMatch.jpg", templim); 
            } 
        }
        else{
            RTT::log(RTT::Error)<<"Maximum value of detection was lower than threshold:"<<*max_val<<" < "<<thresh<<" No top marker detected"<<RTT::endlog();
        }
    }
}
    
void HawkEye::multiObject(cv::Mat image, cv::Mat templim, float thresh, int *w, int *h, double *max_val, std::vector<int> *maxpoints){ //use this for the circles, since there are two circle patterns on the robot
    maxpoints->clear(); //initialize vector as empty

    cv::Size templim_size = templim.size(); 
    *h = templim_size.height;
    *w = templim_size.width;
    cv::Size image_size = image.size(); 
    cv::cvtColor(image, image , CV_RGB2GRAY); //Convert current frame (i.e. image) to grayscale, for use in matchtemplate (since templim is grayscale)

    //Make a result image, based on the input image and the template (see: http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/template_matching/template_matching.html)
    cv::Mat result;
    int result_cols =  image.cols - templim.cols + 1;
    int result_rows = image.rows - templim.rows + 1;
    result.create( result_rows, result_cols, CV_32FC1 );

    double min_val;
    cv::Point min_loc;
    cv::Point max_loc;

    if (*h < image_size.height && *w < image_size.width){
        cv::matchTemplate(image , templim , result, cv::TM_CCOEFF_NORMED); //match the template
        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
        // cv::minMaxLoc(result, &min_val, max_val, &min_loc, &max_loc);
        //Image is a matrix with each element being a pixel in grayscale so with a value 0...255
        //result is a matrix with each element being the correspondance with the template
        //Loop over all elements of the result matrix, see which pixels are >= thresh
        //Put in result vector which contains the position of the corresponding elements = pixel number = position
        //Postprocess vector to kick out points which are too close to each other: euclidean distance
        //Finally you get 2 points which give the position of the templates

        //Todo: how do you make sure that you get the midpoint of the template and not a random point on the template? You find the whole template so...     

        while (true) 
        {
            // double minval, maxval, threshold = 0.8;
            // cv::Point minloc, maxloc;
            cv::minMaxLoc(result, &min_val, max_val, &min_loc, &max_loc);

            if (*max_val >= thresh)
            {
                HAWKEYE_DEBUG_PRINT("Found a bottom marker")
                HAWKEYE_DEBUG_PRINT("maximum value of match: "<<*max_val)  
                maxpoints->push_back(max_loc.x);
                maxpoints->push_back(max_loc.y);
                cv::rectangle(image, max_loc, cv::Point(max_loc.x + templim.cols, max_loc.y + templim.rows), CV_RGB(0,255,0), 2);
                cv::floodFill(result, max_loc, cv::Scalar(0), 0, cv::Scalar(.1), cv::Scalar(1.));
            }
            else
                break;
        }


        /* REMOVED OLD IMPLEMENTATION OF NP.WHERE()
        unsigned char *myData = result.data; //By using pointers, should be faster

        cv::Point2i a; //help variables for Euclidean distance
        cv::Point2i b;
        cv::Point2i tmpPoint; //temporary variable which allows computing the distance only between maxpoints[-1] and current b
        HAWKEYE_DEBUG_PRINT("thresh: "<<thresh)
        for (int k = 0 ; k < result.rows; k++){
            for (int j = 0 ; j < result.cols; j++){
                HAWKEYE_DEBUG_PRINT("myData[k+j] = "<<(int)myData[k+j])

                // if (result.at<double>(Point(j, k)) > thresh){ //without pointers
                if (myData[k + j] > thresh){ //By using pointers, should be faster
                  // HAWKEYE_DEBUG_PRINT("k,j = "<<k<< " , "<<j)
                  // HAWKEYE_DEBUG_PRINT("myData[k+j] = "<<(int)myData[k+j])
                    if (maxpoints->empty()){ //test if vector is still empty
                        maxpoints->push_back(k); //add first point
                        maxpoints->push_back(j);
                        // HAWKEYE_DEBUG_PRINT("indices > thresh in multiObject1: "<<k<<" , "<<j) 
                        a.x = k;
                        a.y = j;
                        tmpPoint.x = k;
                        tmpPoint.y = j;
                    }
                    else{
                        b.x = k;
                        b.y = j;
                        double dist = cv::norm(cv::Mat(tmpPoint), cv::Mat(b),cv::NORM_L2); //Euclidean distance
                        // HAWKEYE_DEBUG_PRINT("distance: "<<dist)
                        // HAWKEYE_DEBUG_PRINT("a and b: ("<<a.x<<" , "<<a.y<<") , ("<<b.x<<" , "<<b.y<<")")
                        if (dist > *w){ //test if distance of point you check > template width, if so then add
                            maxpoints->push_back(k);
                            maxpoints->push_back(j);
                            tmpPoint.x = k;
                            tmpPoint.y = j;
                            // HAWKEYE_DEBUG_PRINT("distance: "<<dist)
                            // HAWKEYE_DEBUG_PRINT("indices > thresh in multiObject2: "<<k<<" , "<<j)
                        }
                    }
                }
            }
        }
        */

        HAWKEYE_DEBUG_PRINT("maximum circle template location: x = "<<max_loc.x<<" y = "<<max_loc.y)
        if (HAWKEYE_SAVE){
          cv::circle(result, cv::Point((*maxpoints)[0] + templim.cols/2.0, (*maxpoints)[1] + templim.rows/2.0 ), 4, cv::Scalar(255,255,0), 2); //plot marker location on images
          cv::circle(result, cv::Point((*maxpoints)[2] + templim.cols/2.0, (*maxpoints)[3] + templim.rows/2.0), 4, cv::Scalar(255,255,0), 2); //plot marker location on images
          cv::circle(image,  cv::Point((*maxpoints)[0] + templim.cols/2.0, (*maxpoints)[1] + templim.rows/2.0), 4, cv::Scalar(255,255,0), 2); //plot marker location on images
          cv::circle(image,  cv::Point((*maxpoints)[2] + templim.cols/2.0, (*maxpoints)[3] + templim.rows/2.0), 4, cv::Scalar(255,255,0), 2); //plot marker location on images
          cv::imwrite("resultMultiMatch.jpg", result); //Todo: why is this all black? Normed the image 0...1! But image data goes 0...255??
          cv::imwrite("inputMultiMatch.jpg", image); 
          cv::imwrite("templateMultiMatch.jpg", templim); 
        } 

        if(maxpoints->empty()){ //if maxpoints still empty no patterns were found
            RTT::log(RTT::Error)<<"Maximum value of detection was lower than threshold:"<<*max_val<<" < "<<thresh<<" No circle patterns detected"<<RTT::endlog();
        }
        else{    
          HAWKEYE_DEBUG_PRINT("template locations: "<<(*maxpoints)[0]<<" , "<<(*maxpoints)[1]<<" , "<<(*maxpoints)[2]<<" , "<<(*maxpoints)[3]<<" template width: "<<*w<<"template height: "<<*h)  
        }

    }
}

std::vector<int> HawkEye::twoTemplate(cv::Mat image, cv::Mat templim1, cv::Mat templim2, float thresh){ //Todo: Only used for testing now. Can also be used if you have two templates with a different shape in one image?
    cv::Size templim1_size = templim1.size(); 
    int h1 = templim1_size.height;
    int w1 = templim1_size.width;

    cv::Size templim2_size = templim2.size(); 
    int h2 = templim2_size.height;
    int w2 = templim2_size.width;

    cv::Mat result1;
    double min_val1;
    double max_val1;
    cv::Point2i min_loc1;
    cv::Point2i max_loc1;

    cv::cvtColor(image, image , CV_RGB2GRAY); //Convert current frame (i.e. image) to grayscale, for use in matchtemplate (since templim is grayscale)

    cv::matchTemplate(image, templim1, result1, cv::TM_CCOEFF_NORMED);
    cv::minMaxLoc(result1, &min_val1, &max_val1, &min_loc1, &max_loc1);
    HAWKEYE_DEBUG_PRINT(max_val1)

    cv::Point2i top_left1;
    cv::Point2i bottom_right1;

    if (max_val1 > thresh){
        top_left1 = max_loc1;
        bottom_right1.x = top_left1.x + w1;
        bottom_right1.y = top_left1.y + h1;
    }
    HAWKEYE_DEBUG_PRINT(top_left1<<bottom_right1)
        
    cv::Mat result2;
    double min_val2;
    double max_val2;
    cv::Point2i min_loc2;
    cv::Point2i max_loc2;

    cv::matchTemplate(image, templim2, result2, cv::TM_CCOEFF_NORMED);
    cv::minMaxLoc(result2, &min_val2, &max_val2, &min_loc2, &max_loc2);
    HAWKEYE_DEBUG_PRINT(max_val2)

    cv::Point2i top_left2;
    cv::Point2i bottom_right2;

    if (max_val2 > thresh){
        top_left2 = max_loc2;
        bottom_right2.x = top_left2.x + w2;
        bottom_right2.y = top_left2.y + h2;
    }
    HAWKEYE_DEBUG_PRINT(top_left2<<bottom_right2)

    int outputArray[8] = {top_left1.x, top_left1.y, bottom_right1.x, bottom_right1.y, top_left2.x, top_left2.y, bottom_right2.x, bottom_right2.y};
    std::vector<int> output(outputArray, outputArray+8);
}

bool  HawkEye::reset()
{
    // send reset request to camera
    // deviceReset();

    // set the state to idle as we can predict what is going to happen, but cannot rely on the serial data because the link will be broken

    // set the buffers to their initial states
}

resolution_t HawkEye::getResolution()
{
  return _resolution;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HawkEye)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(HawkEye)
