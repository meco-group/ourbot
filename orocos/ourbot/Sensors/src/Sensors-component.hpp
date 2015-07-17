#ifndef OROCOS_SENSORS_COMPONENT_HPP
#define OROCOS_SENSORS_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class Sensors : public RTT::TaskContext{
  private:
    OutputPort<std::vector<double> > _cal_lidar_x_port;
    OutputPort<std::vector<double> > _cal_lidar_y_port;
    OutputPort<std::vector<double> > _cal_ir_x_port;
    OutputPort<std::vector<double> > _cal_ir_y_port;
    OutputPort<std::vector<double> > _cal_imul_transacc_port;
    OutputPort<double> _cal_imul_dorientation_port;
    OutputPort<double> _cal_imul_orientation_port;
    OutputPort<std::vector<double> > _cal_imur_transacc_port;
    OutputPort<double> _cal_imur_dorientation_port;
    OutputPort<double> _cal_imur_orientation_port;
    OutputPort<std::vector<double> > _cal_enc_pose_port;
    OutputPort<std::vector<double> > _cal_motor_current_port;
    OutputPort<std::vector<double> > _cal_velocity_port;

    int _lidar_data_length;
    int _ir_data_length;
    int _obs_data_length;

  public:
    Sensors(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
};
#endif
