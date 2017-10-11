#include "Gui.hpp"

void mouseCallBack(int event, int x, int y, int flags, void* data){
    Gui* gui = (Gui*) data;
    if (event == cv::EVENT_LBUTTONDOWN){
        gui->setMouseClickPosition(x, y);

        // if (flags == 40){ // ctrl + left mouse button
        //     gui->setMouseClickPosition(x, y);
        // }
    }
}

Gui::Gui(const std::vector<int>& resolution, bool save_video, double sample_rate):
    _resolution(resolution), _resolution_orig(2), _save_video(save_video),
    _mouseclick_position(2, -100),
    _black(cv::Scalar(77, 76, 75)), _pixelspermeter(1){
    if (save_video){
        _saved_video.open("movie.avi", CV_FOURCC('M','J','P','G'), sample_rate, cv::Size(_resolution[0], _resolution[1]), true);
    }
}

bool Gui::start(){
    cvStartWindowThread();
    cv::namedWindow("Frame", 1);
    cv::setMouseCallback("Frame", mouseCallBack, (void*)this);
    return true;
}

void Gui::stop(){
    cvDestroyAllWindows();
}

void Gui::setPixelsPerMeter(int pixelspermeter){
    _pixelspermeter = pixelspermeter;
}

void Gui::draw(cv::Mat& frame, const std::vector<double>& obstacles, const std::vector<Robot*>& robots, const std::vector<int>& robot_colors, int draw_amount){
    int height = frame.size().height;
    int width = frame.size().width;
    _resolution_orig[0] = width;
    _resolution_orig[1] = height;
    if (draw_amount >= 1) {
        // draw coordinate system
        cv::circle(frame, cv::Point2i(0, height), 5, _black, -2);
        cv::line(frame, cv::Point2i(0, height), cv::Point2i(20, height), _black, 2);
        cv::line(frame, cv::Point2i(0, height), cv::Point2i(0, height-20), _black, 2);
        for (int i=0; i<(height/_pixelspermeter); i++){
            cv::line(frame, cv::Point2i(0, height-(i+1)*_pixelspermeter), cv::Point2i(5, height-(i+1)*_pixelspermeter), _black, 2);
        }
        for (int i=0; i<(width/_pixelspermeter); i++){
            cv::line(frame, cv::Point2i((i+1)*_pixelspermeter, height), cv::Point2i((i+1)*_pixelspermeter, height-5), _black, 2);
        }
    }
    if (draw_amount >= 2) {
        // draw robots
        int n_colors = robot_colors.size();
        for (uint k=0; k<robots.size(); k++){
            if (robots[k]->detected()){
                robots[k]->draw(frame, cv::Scalar(robot_colors[(3*k+2)%n_colors], robot_colors[(3*k+1)%n_colors], robot_colors[(3*k)%n_colors]), _pixelspermeter, draw_amount);
            }
        }

    }
    if (draw_amount >= 3) {
        // draw obstacles
        for (uint k=0; k<obstacles.size(); k+=5){
            if (obstacles[k] == -100){
                break;
            }
            if (obstacles[k+3] < 0){
                // circle
                cv::circle(frame, cv::Point2f(obstacles[k]*_pixelspermeter, height-obstacles[k+1]*_pixelspermeter), obstacles[k+2]*_pixelspermeter, _black, 2);
            } else {
                // rectangle
                cv::RotatedRect rectangle = cv::RotatedRect(cv::Point2f(obstacles[k]*_pixelspermeter, height-obstacles[k+1]*_pixelspermeter), cv::Size2f(obstacles[k+3]*_pixelspermeter, obstacles[k+4]*_pixelspermeter), -obstacles[k+2]);
                cv::Point2f vertices[4];
                rectangle.points(vertices);
                for (int i=0; i<4; i++){
                    cv::line(frame, vertices[i], vertices[(i+1)%4], _black, 2);
                }
            }
        }
    }
    // flexonomy debugging
    if (robots[2]->detected()) {
        // draw table
        double w = robots[2]->getWidth()/_pixelspermeter;
        double h = robots[2]->getHeight()/_pixelspermeter;
        std::vector<double> markers;
        robots[2]->getMarkers(markers, _pixelspermeter, height);
        double x = (markers[0] + markers[2] + markers[4])/3.;
        double y = (markers[1] + markers[3] + markers[5])/3.;
        double theta = atan2(markers[5] - 0.5*(markers[1]+markers[3]), markers[4] - 0.5*(markers[0]+markers[2]));
        std::vector<cv::Point2f> verts(4);
        verts[0] = cv::Point2f(x+0.5*w*cos(theta) - 0.5*h*sin(theta), y+0.5*w*sin(theta) + 0.5*h*cos(theta));
        verts[1] = cv::Point2f(x+0.5*w*cos(theta) + 0.5*h*sin(theta), y+0.5*w*sin(theta) - 0.5*h*cos(theta));
        verts[2] = cv::Point2f(x-0.5*w*cos(theta) + 0.5*h*sin(theta), y-0.5*w*sin(theta) - 0.5*h*cos(theta));
        verts[3] = cv::Point2f(x-0.5*w*cos(theta) - 0.5*h*sin(theta), y-0.5*w*sin(theta) + 0.5*h*cos(theta));
        std::vector<cv::Point2f> verts_px(4);
        for (int i=0; i<4; i++) {
            verts_px[i] = cv::Point2f(verts[i].x*_pixelspermeter, height-verts[i].y*_pixelspermeter);
        }
        for (int i=0; i<4; i++) {
            cv::line(frame, verts_px[i], verts_px[(i+1)%4], _black, 2);
        }
        // draw loading position
        double x_l = x - 1.;
        double y_l = y - 0.2;
        cv::circle(frame, cv::Point2f(x_l*_pixelspermeter, height-y_l*_pixelspermeter), 0.1*_pixelspermeter, _black, -2);
    }

    // show frame
    cv::Mat displayed_frame;
    cv::resize(frame, displayed_frame, cv::Size(_resolution[0], _resolution[1]), 0, 0);
    if (draw_amount >= 1) {
        // click position
        cv::circle(displayed_frame, cv::Point2i(_mouseclick_position[0], _mouseclick_position[1]), 30, _black, 2);
        cv::circle(displayed_frame, cv::Point2i(_mouseclick_position[0], _mouseclick_position[1]), 10, _black, -1.5);
    }
    // save video
    if (_save_video){
        _saved_video.write(displayed_frame);
    }
    cv::imshow("Frame", displayed_frame);
    cv::waitKey(25);
}

void Gui::setMouseClickPosition(int x, int y){
    _mouseclick_position[0] = x;
    _mouseclick_position[1] = y;
}

void Gui::getClickPose(std::vector<double>& pose){
    pose[0] = (_resolution_orig[0]*1.0/_resolution[0])*_mouseclick_position[0]*1.0/_pixelspermeter;
    pose[1] = (_resolution_orig[1]*1.0/_resolution[1])*(_resolution[1] - _mouseclick_position[1])*1.0/_pixelspermeter;
    pose[2] = 0.0;
}

