#ifndef CAMERA_HPP
#define CAMERA_HPP


#include <iostream>
#include <stdint.h>
#include <math.h>
#include <string>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h> //v4l2
#include <unistd.h>

#include <rtt/RTT.hpp>

using namespace RTT;

class Camera {
    private:
        int _fd_cam;
        std::string _video_port_name;
        std::vector<int> _resolution;
        int _brightness;
        cv::Mat _camera_matrix;
        cv::Mat _distortion_vector;
        uint8_t* _cam_buffer;
        double _capture_time_mod;
        double _crop_ratio;

        void buildMatrices(const std::vector<double>& camera_cfs, const std::vector<double>& distortion_cfs);
        int xioctl(int fd, int request, void *arg);
        bool printInfo();
        bool initBuffers();
        void convertToRGB(uint16_t* bayer, uint8_t* rgb, int width, int height);

    public:
        Camera(const std::string& video_port_name, const std::vector<int>& resolution, int brightness,
                const std::vector<double>& camera_cfs, const std::vector<double>& distortion_cfs, double capture_time_mod, double crop_ratio);
        bool start(bool print_info);
        bool setResolution(const std::vector<int>& resolution);
        bool setBrightness(int brightness);
        bool setExposure(int exposure);
        bool setISO(int iso);
        bool capture(cv::Mat& frame, double& capture_time);
        double captureTime();
        void stop();
};


#endif
