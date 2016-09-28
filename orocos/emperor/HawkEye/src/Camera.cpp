#include "Camera.hpp"

using namespace RTT;

Camera::Camera(const std::string& video_port_name, const std::vector<int>& resolution, int brightness, int exposure, int iso,
    const std::vector<double>& camera_cfs, const std::vector<double>& distortion_cfs):
    _video_port_name(video_port_name), _resolution(resolution),
    _brightness(brightness), _exposure(exposure), _iso(iso){
    buildMatrices(camera_cfs, distortion_cfs);
}

void Camera::buildMatrices(const std::vector<double>& camera_cfs, const std::vector<double>& distortion_cfs){
    _camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    _camera_matrix.at<double>(0,0) = camera_cfs[0];  //fx
    _camera_matrix.at<double>(1,1) = camera_cfs[2];  //fy
    _camera_matrix.at<double>(0,2) = camera_cfs[1];  //cx
    _camera_matrix.at<double>(1,2) = camera_cfs[3];  //cy

    _distortion_vector = cv::Mat::zeros(5, 1, CV_64F);
    for (int k=0; k<5; k++){
        _distortion_vector.at<double>(k,0) = distortion_cfs[k];
    }
}

int Camera::xioctl(int fd, int request, void *arg){
    // adapted ioctl implementation for image capturing
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

bool Camera::start(bool print_info){
    // open camera
    if ((_fd_cam = open(_video_port_name.c_str(), O_RDWR)) == -1){
        log(Error) << "Error while opening video device!" << endlog();
        return false;
    }
    // set data format and resolution
    if (!setResolution(_resolution)){
        return false;
    }
    // initialize buffers
    if (!initBuffers()){
        return false;
    }
    // print camera info
    if (print_info){
        if (!printInfo()){
            return false;
        }
    }
    // set brightness, exposure, iso
    if (!setBrightness(_brightness)){
        return false;
    }
    // if (!setExposure(_exposure)){
    //     return false;
    // }
    // if (!setISO(_iso)){
    //     return false;
    // }
  return true;
}

void Camera::stop(){
    close(_fd_cam);
}

bool Camera::initBuffers(){
    struct v4l2_requestbuffers request = {0};
    request.count = 1;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(_fd_cam, VIDIOC_REQBUFS, &request) == -1){
        log(Error) << "Error while requesting buffer!" << endlog();
        return false;
    }
    struct v4l2_buffer buffer = {0};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = 0;
    if (xioctl(_fd_cam, VIDIOC_QUERYBUF, &buffer) == -1){
        log(Error)  << "Error while querying buffer!" << endlog();
        return false;
    }
    _cam_buffer = (uint8_t*)mmap (NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd_cam, buffer.m.offset);
    // prepare for capturing frames
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = 0;
    if (xioctl(_fd_cam, VIDIOC_QBUF, &buffer) == -1){
        log(Error) << "Error while querying buffer!" << endlog();
        return false;
    }
    if (xioctl(_fd_cam, VIDIOC_STREAMON, &buffer.type) == -1){
        log(Error) << "Error while starting video capture!" << endlog();
        return false;
    }
    return true;
}

bool Camera::printInfo(){
    char fourcc[5] = {0};
    struct v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = _resolution[0];
    format.fmt.pix.height = _resolution[1];
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    strncpy(fourcc, (char *)&format.fmt.pix.pixelformat, 4);
    struct v4l2_capability capabilities = {};
    if (xioctl(_fd_cam, VIDIOC_QUERYCAP, &capabilities) == -1){
      log(Error) << "Error while querying capabilities!" << endlog();
      return false;
    }
    struct v4l2_cropcap crop_capabilities;
    crop_capabilities.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl (_fd_cam, VIDIOC_CROPCAP, &crop_capabilities) == -1){
      log(Error) << "Error while querying cropping capabilities" << endlog();
      return false;
    }
    printf("Camera started!\n\nSelected mode:\n"
           "\twidth: %d\n\theight: %d\n\tpixel format: %s\n\tfield: %d\n\n"
           "Driver capabilities:\n"
           "\tdriver: %s\n\tcard: %s\n\tbus: %s\n\tversion: %s\n\t capabilities: %08x\n\n"
           "Camera cropping:\n"
           "\tbounds: %dx%d+%d+%d\n\tdefault: %dx%d+%d+%d\n\taspect: %d/%d\n",
           format.fmt.pix.width, format.fmt.pix.height, fourcc, format.fmt.pix.field,
           capabilities.driver, capabilities.card, capabilities.bus_info,
           (capabilities.version>>16)&&0xff, (capabilities.version>>24)&&0xff, capabilities.capabilities,
           crop_capabilities.bounds.width, crop_capabilities.bounds.height,
           crop_capabilities.bounds.left, crop_capabilities.bounds.top,
           crop_capabilities.defrect.width, crop_capabilities.defrect.height,
           crop_capabilities.defrect.left, crop_capabilities.defrect.top,
           crop_capabilities.pixelaspect.numerator, crop_capabilities.pixelaspect.denominator);
    return true;
}

bool Camera::setResolution(const std::vector<int>& resolution){
    std::vector<std::vector<int> > possible_resolutions = {{672, 380}, {1280, 720}, {1920, 1080}, {2688, 1520}};
    bool check;
    for (uint k=0; k<possible_resolutions.size(); k++){
        check = true;
        for (int i=0; i<2; i++){
            check &= (possible_resolutions[k][i] == resolution[i]);
        }
        if (check) {
            break;
        }
    }
    if (!check){
        log(Error) << "Wrong resolution set!" << endlog();
        return false;
    }
    struct v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = _resolution[0];
    format.fmt.pix.height = _resolution[1];
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(_fd_cam, VIDIOC_S_FMT, &format) == -1){
        log(Error) << "Error while setting pixel format!" << endlog();
        return false;
    }
    return true;
}

bool Camera::setBrightness(int brightness){
    if (brightness < 0 || brightness > 40){
        log(Error) << "Brighness should lie between 0 and 40!" << endlog();
        return false;
    }
    struct v4l2_control control;
    control.id = V4L2_CID_BRIGHTNESS;
    control.value = brightness;
    if (ioctl(_fd_cam, VIDIOC_S_CTRL, &control) == -1){
        log(Error) << "Error while setting the brighness!" << endlog();
        return false;
    }
    return true;
}

bool Camera::setExposure(int exposure){
    if (exposure < 1 || exposure > 10000){
        log(Error) << "Exposure should lie between 1 and 10000!" << endlog();
        return false;
    }
    struct v4l2_control control;
    control.id = V4L2_CID_EXPOSURE_AUTO;
    control.value = 0; // manual
    if (ioctl(_fd_cam, VIDIOC_S_CTRL, &control) == -1){
        log(Error) << "Error while setting the exposure to manual!" << endlog();
        return false;
    }
    control.id = V4L2_CID_EXPOSURE;
    control.value = exposure;
    if (ioctl(_fd_cam, VIDIOC_S_CTRL, &control) == -1){
        log(Error) << "Error while setting the exposure!" << endlog();
    }
    return true;
}

bool Camera::setISO(int iso){
    if (iso < 100 || iso > 2500){
        log(Error) << "ISO should lie between 100 and 2500!" << endlog();
        return false;
    }
    struct v4l2_control control;
    control.id = V4L2_CID_ISO_SENSITIVITY_AUTO;
    control.value = 0; // manual
    if (ioctl(_fd_cam, VIDIOC_S_CTRL, &control) == -1){
        log(Error) << "Error while setting the ISO to manual!" << endlog();
        return false;
    }
    control.id = V4L2_CID_ISO_SENSITIVITY;
    control.value = iso;
    if (ioctl(_fd_cam, VIDIOC_S_CTRL, &control) == -1){
        log(Error) << "Error while setting the iso!" << endlog();
    }
    return true;
}

bool Camera::capture(cv::Mat& frame, double& capture_time){
    struct v4l2_buffer buffer = {0};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = 0;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd_cam, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;  //wait 2 seconds before giving up
    capture_time = captureTime();
    if (select(_fd_cam+1, &fds, NULL, NULL, &tv) == -1){
        log(Error) << "Error while waiting for frame!" << endlog();
        return false;
    }
    if (xioctl(_fd_cam, VIDIOC_DQBUF, &buffer) == -1){
        log(Error) << "Error while retrieving frame!" << endlog();
        return false;
    }
    cv::Mat frame_rgb(cv::Size(_resolution[0]/2, _resolution[1]/2), CV_8UC3);
    convertToRGB(reinterpret_cast<uint16_t*>(_cam_buffer), frame_rgb.data, _resolution[0], _resolution[1]);
    // undistort image
    cv::undistort(frame_rgb, frame, _camera_matrix, _distortion_vector);
    // reset buffer
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = 0;
    if (xioctl(_fd_cam, VIDIOC_QBUF, &buffer) == -1){
        log(Error) << "Error while querying buffer!" << endlog();
        return false;
    }
    return true;
}

void Camera::convertToRGB(uint16_t* bayer, uint8_t* rgb, int width, int height){
    // convert 10bit raw to RGB
    uint16_t blue_16, green_16, red_16;
    uint8_t blue, green, red;
    for (int x=0; x<width; x+=2){
        for (int y=0; y<height; y+=2){
            // get colors from raw matrix
            blue_16 = bayer[x + width*y];
            green_16 = bayer[x+1 + width*y];
            red_16 = bayer[x+1 + width*(y+1)];
            // clip
            blue = (uint8_t)((blue_16>0xff)?0xff:blue_16);
            green = (uint8_t)((green_16>0xff)?0xff:green_16);
            red = (uint8_t)((red_16>0xff)?0xff:red_16);
            // assign to rgb matrix
            rgb[3*(x/2+width*y/4)+0] = blue;
            rgb[3*(x/2+width*y/4)+1] = green;
            rgb[3*(x/2+width*y/4)+2] = red;
        }
    }
}

double Camera::captureTime(){
    uint32_t ms;
    double s;
    ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();
    s = ms * double(std::chrono::milliseconds::period::num) / std::chrono::milliseconds::period::den;
    return s;
}
