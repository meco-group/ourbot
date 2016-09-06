#ifndef OROCOS_HAWKEYE_COMPONENT_HPP
#define OROCOS_HAWKEYE_COMPONENT_HPP

//Set flags:
// #define HAWKEYE_PLOTFLAG
// #define HAWKEYE_SAVEFLAG
// #define HAWKEYE_DEBUGFLAG

#ifdef HAWKEYE_DEBUGFLAG //print statements on/off
	#define HAWKEYE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define HAWKEYE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#ifdef HAWKEYE_PLOTFLAG //plot intermediate figures on/off
    #define HAWKEYE_PLOT 1
#else
    #define HAWKEYE_PLOT 0
#endif

#ifdef HAWKEYE_SAVEFLAG //save image processing results on/off
    #define HAWKEYE_SAVE 1
#else
    #define HAWKEYE_SAVE 0
#endif

//General
#include <iostream>
#include <time.h> //to get timing information: difftime(), time(),...
#include <ctime>  // to use clock_t
#include <chrono> //to get time in milliseconds
#include <math.h> //atan2

#include <string> //for std::to_string
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

//UDP stuff
#include <stdlib.h> /* defines exit and other sys calls */
#include <stdio.h>
#include <string.h> // needed for memset
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

//To use Obstacle class
#include "Circle.hpp"
#include "Rectangle.hpp"

//Define For use in image acquisition
typedef uint16_t WORD;
typedef uint8_t  BYTE;


typedef enum resolution_t{ //define enum to hold possibilities for resolution
    LOW = 1,
    HD720p = 2,
    HD1080p = 3,
    FULL = 4
} resolution_t;

using namespace RTT;

class HawkEye : public RTT::TaskContext{

  private:
    double const pi=4*atan(1); //define constant pi

    // Ports
    OutputPort <std::vector<double> >  _obstacles_state_port; //state of all obstacles
    OutputPort <std::vector<double> >  _kurt_state_port;  //state of the robot
    OutputPort <std::vector<double> >  _dave_state_port;  //state of the robot
    OutputPort <std::vector<double> >  _krist_state_port;  //state of the robot

    // Properties
    std::string _video_port_name; // standard value is "/dev/video0". The name is: See3CAM_CU40
    std::string _image_path;
    int _reso;
    resolution_t _resolution;
    int _brightness;
    int _exposure;
    int _iso;
    bool _save_images;
    int _number_of_bg_samples;
    bool _capture_bg_at_start;
    bool _print_cam_info;
    bool _stream_images;
    std::string _server_address;
    std::vector<int> _stream_image_size;

    // TCP streaming stuff
    int _socket;
    int _port_nr;

    // width/heigt related to resolution
    int _width;
    int _height;

    uint8_t *_buffer; //camera buffer
    int _errno;  //error handling
    double _cntapprox; //setting of approxPolyDP function which approximates polygons/contours by simplified versions
    int _diffthresh; //threshold for background subtraction
    float _matchThresh; //threshold for template matching

    // Camera
    int _fd; //file descriptor for camera
    double _pix2meter; //transform pixels to meter
    unsigned long _capture_time; //timestamp for current frame

    // Templates
    cv::Mat _template_circle; //Mat is the openCV type of an image, it's a kind of vector
    cv::Mat _template_circlehollow;
    cv::Mat _template_star1;
    cv::Mat _template_star2;
    cv::Mat _template_cross;
    cv::Mat _template_cross_rot;

    //Images
    cv::Mat _f; //current frame
    cv::Mat _background; //background image
    cv::Mat _diff; //current difference with background image
    cv::Mat _mask; //the current mask
    cv::Mat _maskcopy; //copy of the current mask
    std::vector<int> _rorig; //region of interest
    cv::Mat _roi; //Todo: add size? Always 4?


    //Intermediate results
    std::vector<std::vector<cv::Point> > _boxcontours; //holds contours of all obstacles contours
    std::vector<std::vector<cv::Point> > _rectanglesDetectedContours; //holds contours of all detected rectangles
    std::vector<cv::RotatedRect> _boxes;       //holds all rectangle representations of obstacles
    std::vector<cv::RotatedRect> _boxes_correct;//holds all rectangle representations of obstacles, when the robot was touching an obstacle
    std::vector<cv::RotatedRect> _rectanglesDetected; // holds all detected rectangular obstacles
    std::vector<std::vector<double> > _circles;     //holds all circle representations of obstacles, flipped
    std::vector<std::vector<double> > _circles_correct; //holds all circle representations of obstacles, when the robot was touching an obstacle
    std::vector<std::vector<double> > _circlesDetected; //holds all detected circles
    std::vector<cv::RotatedRect> _roboboxes; //vector with roboboxes, which holds robot state: position, width, height, angle

    //Results
    std::vector<double> _kurt;  //robot is described by its 3 markers with x and y position + added timestamp: 7 elements
    std::vector<double> _dave;
    std::vector<double> _krist;
    bool _found_kurt; //did we find Kurt?
    bool _found_dave;
    bool _found_krist;
    std::vector<Obstacle*> _obstacles;

    // Class methods
    bool setResolution(resolution_t resolution);
    void setBrightness(int brightness);
    void setExposure(int exposure);
    void setISO(int iso);
    bool loadTemplates();
    bool loadBackground();
    void captureBackground();
    bool startCamera();
    void processImage();
    void backgroundSubtraction(std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy);
    void findRobots();
    void findBigObstacles(cv::RotatedRect rotrect, std::vector<cv::Point> c, int cx, int cy, int cradius, std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy);
    void mergeContourWithRobot(cv::RotatedRect robobox, std::vector<cv::Point> contourPoints, std::vector<std::vector<cv::Point> > *contours, std::vector<cv::Vec4i> *hierarchy); //found a contour which contains the robot, blank it out
    void processResults();
    void writeResults();
    void drawResults();
    void transform(std::vector<double> &values);
    void printedMatch(cv::Mat roi, cv::Mat template_circle, cv::Mat template_star1, cv::Mat template_star2, cv::Mat template_cross, cv::Mat template_cross_rot, cv::Mat template_circlehollow, bool *success, double *templ_locs, double *robottocks, double *starpat, double *crosspat, double *circlehollowpat, float matchThresh, std::vector<int> rorig);
    void oneObject(cv::Mat image, cv::Mat templim, float thresh, int *w, int *h, double *max_val, cv::Point *temploc);
    void multiObject(cv::Mat image, cv::Mat templim, float thresh, int *w, int *h, double *max_val, std::vector<int> *maxpoints);

    // TCP related methods
    bool connectToServer();
    bool sendImage(const cv::Mat& image);

    // Low level image capturing
    void pabort(const char *s); //Error catching
    static int xioctl(int fd, int request, void *arg); //Adapted ioctl use
    void bayer10_to_rgb24(WORD *pBay, BYTE *pRGB24, int width, int height, int pix_order);
    void capture_image();


  public:
    HawkEye(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
};
#endif

