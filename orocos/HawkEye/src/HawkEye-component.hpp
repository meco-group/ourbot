#ifndef OROCOS_HAWKEYE_COMPONENT_HPP
#define OROCOS_HAWKEYE_COMPONENT_HPP

//Set flags:
#define HAWKEYE_TESTFLAG
// #define HAWKEYE_PLOTFLAG
#define HAWKEYE_DEBUGFLAG

#ifdef HAWKEYE_DEBUGFLAG //Print statements on/off
	#define HAWKEYE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define HAWKEYE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#ifdef HAWKEYE_PLOTFLAG //plot intermediate figures on/off
    #define HAWKEYE_PLOT 1
#else
    #define HAWKEYE_PLOT 0
#endif

//General
#include <iostream>
#include <stdlib.h>  //setenv and getenv
#include <time.h> //to get timing information: difftime(), time(),...
#include <chrono> //to get time in milliseconds
#include <math.h> //atan2

#include <string> //for std::to_string
#include <sstream>
#include <iterator> //for std::begin, std::end,...
#include <stdint.h> //for uint8_t

//Orocos
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <map>

//Camera
#include <opencv2/core/core.hpp> //basic building blocks of the library
#include <opencv2/highgui/highgui.hpp> //for imread
#include "opencv2/imgproc/imgproc.hpp"

//File handling for camera
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <errno.h> //error logging
#include <fcntl.h>
#include <linux/videodev2.h> //v4l2

//To use Robot and Obstacle classes
#include "Robot.hpp"
#include "Circle.hpp"
#include "Rectangle.hpp"

//Define For use in image acquisition
typedef uint16_t WORD; //or use int?
typedef uint8_t  BYTE; //or use int?


typedef enum resolution_t{ //define enum to hold possibilities for resolution
    LOW = 1,
    HD720p = 2,
    HD1080p = 3,
    FULL = 4
} resolution_t;


using namespace RTT; //otherwise you have to use RTT::OutputPort etc.

class HawkEye : public RTT::TaskContext{

  private:

    //Ports
    OutputPort <std::vector<double> >  _obstacles_state_port; //state of all obstacles
    OutputPort <std::vector<double> >  _robots_state_port;    //state of all robots (we will have 3)
    OutputPort <int>  _width_port;
    OutputPort <int>  _height_port;
    OutputPort <int>  _fps_port;

    //Class variables
    std::string  _device; // standard value is "/dev/video1" , since video0 is the webcam. The name is: See3CAM_CU40
    std::string  _workspace_path; 
    int _fps;
    int _brightness; 
    int _exposure;
    int _width;
    int _height;
    resolution_t _resolution;
    uint8_t *_buffer;
    int _errno;  //error handling
    // int _shutter_speed;

    double const pi=4*atan(1); //define constant pi

    //operating flags and paths
    bool _save_image;   //record images
    bool _load_background_from_file; //if you don't want to make a new background
    bool _draw_markers; //draw detected and computed markers from templated matching
    bool _draw_contours; //draw detected contours
    bool _print_cam_info; //print the camera info while starting the camera (resolution, pixel format, capabilities,...)
    double _cntapprox;
    int _diffthresh;
    float _matchThresh;
    std::string _save_img_path; //where to save images
    std::string _path; //workspace directory

    //Templates
    cv::Mat _template_circle; //Mat is the cpp type of an image
    cv::Mat _template_star1;
    cv::Mat _template_star2;  

    //Camera
    int _fd; //file descriptor for camera

    //Images
    unsigned long _capture_time; //timestamp for current frame
    cv::Mat _f; //current frame
    cv::Mat _background; //background image
    cv::Mat _diff; //current difference with background image

    cv::Mat _mask; //the current mask
    cv::Mat _maskcopy; //copy of the current mask

    std::vector<int> _rorig; //region of interest
    cv::Mat _roi; //Todo: add size? Always 4?

    //Output
    Robot _robot; //Make instance of robot
    Circle _circle; //Make instance of circle
    Rectangle _rectangle; //Make instance of rectangle
    std::vector<Obstacle*> _obstacles;
    std::vector<std::vector<cv::Point> > _boxcontours; //holds contours of all obstacles contours
    std::vector<std::vector<cv::Point> > _rectanglesDetectedContours; //holds contours of all detected rectangles
    std::vector<cv::RotatedRect> _boxes;       //holds all rectangle representations of obstacles
    std::vector<cv::RotatedRect> _boxes_correct;//holds all rectangle representations of obstacles, when the robot was touching an obstacle
    std::vector<cv::RotatedRect> _rectanglesDetected; // holds all detected rectangular obstacles
    std::vector<std::vector<double> > _circlesflip;     //holds all circle representations of obstacles, flipped
    std::vector<std::vector<double> > _circlesflip_correct; //holds all circle representations of obstacles, when the robot was touching an obstacle
    std::vector<std::vector<double> > _circlesDetected; //holds all detected circles

    std::vector<int> _template_circles_pos; 
    cv::Point _template_star_pos; 

    cv::RotatedRect _robobox; //hold contours of robot
    double _robcenflip[2]; //holds flipped center of robot
    double _roboState[3]; //holds State of the robot

    double _drawmods[7]; //draw circular patterns
    double _drawstar[5]; //draw star pattern
    double _drorigx; //draw robot position x
    double _drorigy;  //draw robot position y

    //Class methods - general
    void setResolution(resolution_t resolution);
    void setBrightness(int brightness);
    void setExposure(int exposure);
    void checkFPS(resolution_t resolution);
    void getBackground();
    void backgroundSubtraction(std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy);
    void startCamera();
    void processImage(); 
    void findRobots();  
    void findBigObstacles(cv::RotatedRect rotrect, std::vector<cv::Point> c, int cx, int cyflip, int cradius, std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy);
    void processResults();
    void writeResults();
    void drawResults();
    bool reset();

    //Low level image capturing
    void pabort(const char *s); //Error catching
    static int xioctl(int fd, int request, void *arg); //Adapted ioctl use
    void bayer10_to_rgb24(WORD *pBay, BYTE *pRGB24, int width, int height, int pix_order);
    void capture_image();

    //Class methods - matcher
    void printedMatch(cv::Mat roi, cv::Mat template_circle, cv::Mat template_star1, cv::Mat template_star2, bool *success, double *robottocks, double *starpat, float matchThresh, bool draw_markers, std::vector<int> rorig);
    void oneObject(cv::Mat image, cv::Mat templim, float thresh, int *w, int *h, double *max_val, cv::Point *temploc);
    void multiObject(cv::Mat image, cv::Mat templim, float thresh, int *w, int *h, double *max_val, std::vector<int> *maxpoints);
    std::vector<int> twoTemplate(cv::Mat image, cv::Mat templim1, cv::Mat templim2, float thresh);

  public:
    HawkEye(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();

    //Getters
    resolution_t getResolution();
};
#endif

