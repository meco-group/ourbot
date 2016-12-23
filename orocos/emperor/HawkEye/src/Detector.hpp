#ifndef DETECTOR_HPP
#define DETECTOR_HPP

// #define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(x) std::cout << x << std::endl;
#else
    #define DEBUG_PRINT(x) //std::cout << x << std::endl;
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "Robot.hpp"

#include <rtt/RTT.hpp>

using namespace RTT;

class Detector {
    private:
        std::vector<double> _marker_params;
        cv::SimpleBlobDetector* _blob_detector;
        cv::Mat _background;
        cv::Mat _mask;

        int _nbit_x;
        int _nbit_y;
        double _triangle_ratio;
        double _qr_rel_pos;
        double _qr_rel_width;
        int _threshold_bgst;
        double _threshold_kp;
        double _threshold_top;
        double _min_robot_area;
        double _min_obstacle_area;
        int _max_detectable_obstacles;
        int _pixelspermeter;
        int _frame_height;

        void initDetector();
        void backgroundSubtraction(const cv::Mat& frame, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec4i>& hierarchy);
        void detectRobots(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours, std::vector<Robot*>& robots);
        bool findRobots(cv::Mat& roi, const std::vector<int>& roi_location, std::vector<int>& marker_locations, std::vector<int>& robot_indices);
        int isRobot(cv::Mat& roi, const std::vector<cv::Point2f>& points, std::vector<double>& robot_markers);
        int getTopIndex(const std::vector<cv::Point2f>& points);
        void subtractRobots(std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec4i>& hierarchy, std::vector<Robot*>& robots);
        void detectObstacles(const cv::Mat& frame, const std::vector<std::vector<cv::Point> >& contours, std::vector<double>& obstacle_data, const std::vector<Robot*>& robots);
        void filterObstacles(const std::vector<cv::RotatedRect>& rectangles, const std::vector<std::vector<double> >& circles, std::vector<double>& obstacle_data, const std::vector<Robot*>& robots);
        void addObstacle(const cv::RotatedRect& rectangle, const std::vector<double>& circle, std::vector<std::vector<double>>& obstacles, std::vector<double>& areas);
        void sortObstacles(std::vector<std::vector<double> >& obstacles, std::vector<double>& areas);

    public:
        Detector(const std::vector<double>& marker_params, int threshold_bgst, double threshold_kp, double threshold_top, double min_robot_area, double min_obstacle_area, int max_detectable_obstacles, int pixelspermeter);
        bool start(const cv::Mat& background);
        void update(const cv::Mat& frame, std::vector<Robot*>& robots, std::vector<double>& obstacle_data);
};


#endif
