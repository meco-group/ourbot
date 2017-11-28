#ifndef OROCOS_CAMERA_COMPONENT_HPP
#define OROCOS_CAMERA_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <eagle.h>

using namespace RTT;

cv::Scalar mix_with_white(const cv::Scalar& color, double perc_white){
    cv::Scalar color_w;
    color_w[0] = int(((100. - perc_white)*color[0] + perc_white*255.)/100.);
    color_w[1] = int(((100. - perc_white)*color[1] + perc_white*255.)/100.);
    color_w[2] = int(((100. - perc_white)*color[2] + perc_white*255.)/100.);
    return color_w;
}

class Camera : public RTT::TaskContext{

  private:
    // ports
    OutputPort<std::vector<double> > _target_out_port;
    InputPort<std::vector<double> > _target_in_port;
    std::vector<InputPort<std::vector<double> >* > _robot_est_pose_port;
    std::vector<InputPort<std::vector<double> >* > _robot_ref_x_port;
    std::vector<InputPort<std::vector<double> >* > _robot_ref_y_port;

    // properties
    bool _local_detecting;
    bool _capture_bg_at_start;
    std::string _node_name;
    std::string _communication_group;
    std::string _eagle_config_path;
    std::vector<double> _image_size;
    std::vector<double> _frame_position;
    int _pixels_per_meter;
    bool _image_stream;
    bool _save_movie;
    std::vector<std::string> _eagles;
    int _verbose;
    int _draw;
    std::vector<int> _robot_ids;
    std::vector<double> _robot_sizes;
    std::vector<double> _robot_marker_translation;
    std::vector<int> _robot_colors;

    eagle::Camera* _camera;
    eagle::Detector* _detector;
    eagle::Projection* _projection;
    eagle::Collector* _collector;
    eagle::Communicator* _communicator;
    eagle::Message _message;
    cv::Mat _image;
    cv::Mat _displayed_image;
    uint32_t _capture_time;

    std::vector<eagle::Robot*> _robots;
    std::vector<eagle::Obstacle*> _obstacles;
    std::vector<uint32_t> _robot_timestamps;
    std::vector<uint32_t> _obstacle_timestamps;
    cv::VideoWriter _movie;


    bool load_background();
    void init_robots();
    bool stream(bool enable);
    bool shout_command(eagle::cmd_t CMD);
    void transmit_detected();
    void receive_detected();
    void print(int verbose);
    cv::Mat draw();

  public:
    Camera(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void cleanupHook();

    bool capture_background();
    void set_mouse_click(int x, int y);
};
#endif
