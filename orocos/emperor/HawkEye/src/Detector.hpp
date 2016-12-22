#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <rtt/RTT.hpp>

using namespace RTT;

class Camera {
    private:
        std::vector<double> _marker_params;
        cv::SimpleBlobDetector* _blob_detector;
        cv::Mat _background;

        void initDetector();



        int _fd_cam;
        std::string _video_port_name;
        std::vector<int> _resolution;
        int _brightness;
        int _exposure;
        int _iso;
        cv::Mat _camera_matrix;
        cv::Mat _distortion_vector;
        uint8_t* _cam_buffer;
        double _capture_time_mod;


        void buildMatrices(const std::vector<double>& camera_cfs, const std::vector<double>& distortion_cfs);
        int xioctl(int fd, int request, void *arg);
        bool printInfo();
        bool initBuffers();
        void convertToRGB(uint16_t* bayer, uint8_t* rgb, int width, int height);

    public:
        Camera(const std::string& video_port_name, const std::vector<int>& resolution, int brightness, int exposure, int iso,
                const std::vector<double>& camera_cfs, const std::vector<double>& distortion_cfs, double capture_time_mod);
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
