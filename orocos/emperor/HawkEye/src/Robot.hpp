#ifndef ROBOT_HPP
#define ROBOT_HPP


#include <iostream>
#include <stdint.h>
#include <math.h>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Robot {
    private:
        std::vector<double> _pose;
        std::string _name;
        std::vector<cv::Mat> _top_markers;
        std::vector<double> _local_marker_locations_m;
        std::vector<int> _global_marker_locations_px;
        cv::RotatedRect _box;
        bool _detected;
        int _width;
        int _height;

        void createBox();
        std::vector<double> transform(const std::vector<double>& point, int pixelspermeter, int height);
        std::vector<double> invtransform(const std::vector<double>& point, int pixelspermeter, int height);

    public:
        Robot(const std::string& name, const std::vector<cv::Mat>& top_markers,
            const std::vector<double>& local_marker_locations_m, int width, int height);
        void reset();
        void setMarkers(const std::vector<int>& marker_locations_px);
        std::string getName();
        bool detected();
        cv::RotatedRect getBox();
        double getBottomTopDistance();
        void getMarkers(std::vector<double>& marker_vector, int pixelspermeter, int height);
        void setPose(const std::vector<double>& pose);
        void getTopMarkers(std::vector<cv::Mat>& top_markers);
        void draw(cv::Mat& frame, const cv::Scalar& color, int pixelspermeter);
};

#endif
