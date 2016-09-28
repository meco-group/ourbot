#include "Gui.hpp"

void mouseCallBack(int event, int x, int y, int flags, void* data){
    Gui* gui = (Gui*) data;
    if (event == cv::EVENT_LBUTTONDOWN){
        if (flags == 40){ // ctrl + left mouse button
            gui->setMouseClickPosition(x, y);
        }
    }
}

Gui::Gui(const std::vector<int>& resolution): _resolution(resolution),
    _mouseclick_position(2, -100), _black(cv::Scalar(77, 76, 75)), _pixelspermeter(1){
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

void Gui::draw(cv::Mat& frame, const std::vector<double>& obstacles, const std::vector<Robot*>& robots, const std::vector<int>& robot_colors){
    int height = frame.size().height;
    int width = frame.size().width;
    // draw coordinate system
    cv::circle(frame, cv::Point2i(0, height), 10, _black, -3);
    cv::line(frame, cv::Point2i(0, height), cv::Point2i(30, height), _black, 3);
    cv::line(frame, cv::Point2i(0, height), cv::Point2i(0, height-30), _black, 3);
    for (int i=0; i<(height/_pixelspermeter); i++){
        cv::line(frame, cv::Point2i(0, (i+1)*_pixelspermeter), cv::Point2i(10, (i+1)*_pixelspermeter), _black, 3);
    }
    for (int i=0; i<(width/_pixelspermeter); i++){
        cv::line(frame, cv::Point2i((i+1)*_pixelspermeter, height), cv::Point2i((i+1)*_pixelspermeter, height-10), _black, 3);
    }
    // draw obstacles
    for (uint k=0; k<obstacles.size(); k+=5){
        if (obstacles[k] == -100){
            break;
        }
        if (obstacles[k+3] < 0){
            // circle
            cv::circle(frame, cv::Point2f(obstacles[k], obstacles[k+1]), obstacles[k+2], _black, 2);
        } else {
            // rectangle
            cv::RotatedRect rectangle = cv::RotatedRect(cv::Point2f(obstacles[k], obstacles[k+1]), cv::Size2f(obstacles[k+3], obstacles[k+4]), obstacles[k+2]);
            cv::Point2f vertices[4];
            rectangle.points(vertices);
            for (int i=0; i<4; i++){
                cv::line(frame, vertices[i], vertices[(i+1)%4], _black, 2);
            }
        }
    }
    // draw robots
    for (uint k=0; k<robots.size(); k++){
        if (robots[k]->detected()){
            robots[k]->draw(frame, cv::Scalar(robot_colors[3*k+2], robot_colors[3*k+1], robot_colors[3*k]), _pixelspermeter);
        }
    }
    // show frame
    cv::Mat displayed_frame;
    cv::resize(frame, displayed_frame, cv::Size(_resolution[0], _resolution[1]), 0, 0);
    // click position
    cv::circle(displayed_frame, cv::Point2i(_mouseclick_position[0], _mouseclick_position[1]), 30, _black, 2);
    cv::circle(displayed_frame, cv::Point2i(_mouseclick_position[0], _mouseclick_position[1]), 10, _black, -2);
    cv::imshow("Frame", displayed_frame);
    cv::waitKey(25);
}

void Gui::setMouseClickPosition(int x, int y){
    _mouseclick_position[0] = x;
    _mouseclick_position[1] = y;
}


