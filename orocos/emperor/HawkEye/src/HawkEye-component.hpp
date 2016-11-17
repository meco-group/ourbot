#ifndef OROCOS_HAWKEYE_COMPONENT_HPP
#define OROCOS_HAWKEYE_COMPONENT_HPP

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(x) std::cout << x << std::endl;
#else
    #define DEBUG_PRINT(x) //std::cout << x << std::endl;
#endif

#include <iostream>
#include <stdint.h>
#include <chrono>
#include <ctime>
#include <math.h>
#include <string>
#include <vector>

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "Robot.hpp"
#include "Camera.hpp"
#include "Gui.hpp"

using namespace RTT;

class HawkEye : public RTT::TaskContext{

    private:
        // ports
        std::vector<OutputPort<std::vector<double> >* > _robot_markers_port;
        OutputPort<std::vector<double> > _obstacle_port;
        OutputPort<std::vector<double> > _target_pose_port;

        std::vector<InputPort<std::vector<double> >* > _robot_est_pose_port;
        std::vector<InputPort<std::vector<double> >* > _robot_ref_x_port;
        std::vector<InputPort<std::vector<double> >* > _robot_ref_y_port;

        // properties
        std::vector<std::string> _robot_names;
        std::vector<std::string> _top_markers_path;
        std::string _bottom_marker_path;
        std::vector<int> _top_marker_table;
        std::vector<double> _marker_locations;
        std::vector<double> _robot_sizes;
        std::vector<int> _robot_colors;
        std::string _video_port_name;
        int _brightness;
        int _exposure;
        int _iso;
        std::vector<double> _camera_cfs;
        std::vector<double> _distortion_cfs;
        int _max_detectable_obstacles;
        std::string _image_path;
        std::vector<int> _gui_resolution;
        int _threshold_bgst;
        double _threshold_match;
        double _min_robot_area;
        double _min_obstacle_area;
        int _pixelspermeter;
        bool _capture_bg_at_start;
        int _number_of_bg_samples;
        bool _show_prev_frame;
        double _capture_time_mod;
        bool _save_video;
        double _hawkeye_sample_rate;

        std::vector<cv::Mat> _top_markers;
        cv::Mat _bottom_marker;
        cv::Mat _frame;
        cv::Mat _prev_frame;
        cv::Mat _background;
        cv::Mat _mask;

        Gui* _gui;
        Camera* _camera;
        std::vector<Robot*> _robots;
        std::vector<double> _obstacles;
        std::vector<double> _target_pose;

        double _capture_time;

        bool loadMarkers();
        void buildMatrices();
        bool startCamera();
        bool getBackground();
        bool loadBackground();
        void createRobots();
        void reset();
        void processFrame(const cv::Mat& frame);
        void backgroundSubtraction(const cv::Mat& frame, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec4i>& hierarchy);
        void detectRobots(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours);
        void subtractRobots(std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec4i>& hierarchy);
        void detectObstacles(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours);
        void filterObstacles(const std::vector<cv::RotatedRect>& rectangles, const std::vector<std::vector<double> >& circles);
        void addObstacle(const cv::RotatedRect& rectangle, const std::vector<double>& circle, std::vector<std::vector<double>>& obstacles, std::vector<double>& areas);
        void sortObstacles(std::vector<std::vector<double> >& obstacles, std::vector<double>& areas);
        bool matchMarkers(cv::Mat& roi, const std::vector<int>& roi_location, std::vector<int>& marker_locations, std::vector<int>& robot_indices);
        void matchTemplates(cv::Mat& frame, const cv::Mat& marker, double threshold, std::vector<int>& marker_locations, std::vector<double>& marker_scores);
        void blank(cv::Mat& frame, const cv::Point& location, int width, int height);
        void writeResults();
        void drawResults(cv::Mat& frame);

    public:
        HawkEye(std::string const& name);
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        bool captureBackground();
        void setTargetPose();
};
#endif
