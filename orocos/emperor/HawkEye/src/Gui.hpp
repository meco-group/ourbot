#ifndef GUI_HPP
#define GUI_HPP


#include <iostream>
#include <stdint.h>
#include <math.h>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "Robot.hpp"

class Gui {
    private:
        std::vector<int> _resolution;
        std::vector<int> _resolution_orig;
        bool _save_video;
        std::vector<int> _mouseclick_position;
        cv::Scalar _black;
        int _pixelspermeter;
        cv::VideoWriter _saved_video;

    public:
        Gui(const std::vector<int>& resolution, bool save_video, double sample_rate);
        bool start();
        void setPixelsPerMeter(int pixelspermeter);
        void stop();
        void draw(cv::Mat& frame, const std::vector<double>& obstacles, const std::vector<Robot*>& robots, const std::vector<int>& robot_colors);
        void setMouseClickPosition(int x, int y);
        void getClickPose(std::vector<double>& pose);
};


#endif
