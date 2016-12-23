#ifndef OROCOS_HAWKEYE_COMPONENT_HPP
#define OROCOS_HAWKEYE_COMPONENT_HPP

// #define DEBUG

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
#include "opencv2/opencv.hpp"

#include "Robot.hpp"
#include "Camera.hpp"
#include "Gui.hpp"
#include "Detector.hpp"

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
        std::vector<double> _robot_sizes;
        std::vector<int> _robot_colors;
        std::vector<double> _marker_locations;
        std::string _video_port_name;
        int _brightness;
        std::vector<double> _camera_cfs;
        std::vector<double> _distortion_cfs;
        int _max_detectable_obstacles;
        std::string _image_path;
        std::vector<int> _gui_resolution;
        int _threshold_bgst;
        double _threshold_kp;
        double _threshold_top;
        double _min_robot_area;
        double _min_obstacle_area;
        int _pixelspermeter;
        bool _capture_bg_at_start;
        int _number_of_bg_samples;
        double _capture_time_mod;
        bool _save_video;
        double _hawkeye_sample_rate;
        std::vector<double> _marker_params;
        double _crop_ratio;

        Gui* _gui;
        Camera* _camera;
        Detector* _detector;
        std::vector<Robot*> _robots;
        std::vector<double> _obstacles;
        std::vector<double> _target_pose;
        cv::Mat _frame;
        double _capture_time;

        bool getBackground(cv::Mat& background);
        bool loadBackground(cv::Mat& background);
        void createRobots();
        void reset();
        void writeResults();
        void drawResults(cv::Mat& frame);

    public:
        HawkEye(std::string const& name);
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        bool captureBackground(cv::Mat& background);
        std::vector<double> getRoomSize();
        void setTargetPose();
};
#endif
