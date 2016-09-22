#ifndef OROCOS_ESTIMATORINTERFACE_COMPONENT_HPP
#define OROCOS_ESTIMATORINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
typedef std::vector<RTT::base::PortInterface*> Ports;

using namespace RTT;

class EstimatorInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _cal_lidar_x_port;
    InputPort<std::vector<double> > _cal_lidar_y_port;

    InputPort<std::vector<double> > _cal_imul_transacc_port;
    InputPort<std::vector<double> > _cal_imul_orientation_3d_port;
    InputPort<double> _cal_imul_orientation_port;
    InputPort<std::vector<double> > _cal_imul_dorientation_3d_port;
    InputPort<double> _cal_imul_dorientation_port;

    InputPort<std::vector<double> > _cal_imur_transacc_port;
    InputPort<std::vector<double> > _cal_imur_orientation_3d_port;
    InputPort<double> _cal_imur_orientation_port;
    InputPort<std::vector<double> > _cal_imur_dorientation_3d_port;
    InputPort<double> _cal_imur_dorientation_port;

    InputPort<std::vector<double> > _cal_enc_pose_port;
    InputPort<std::vector<double> > _cal_motor_current_port;
    InputPort<std::vector<double> > _cal_motor_voltage_port;
    InputPort<std::vector<double> > _cal_velocity_port;
    InputPort<std::vector<double> > _cmd_velocity_passthrough_port;

    OutputPort<std::vector<double> > _est_pose_port;
    OutputPort<std::vector<double> > _est_velocity_port;
    OutputPort<std::vector<double> > _est_acceleration_port;
    OutputPort<std::vector<double> > _est_pose_tx_port;

    std::vector<double> _cal_lidar_x;
    std::vector<double> _cal_lidar_y;
    std::vector<double> _cal_ir_x;
    std::vector<double> _cal_ir_y;

    std::vector<double> _cal_imul_transacc;
    std::vector<double> _cal_imul_orientation_3d;
    double _cal_imul_orientation;
    std::vector<double> _cal_imul_dorientation_3d;
    double _cal_imul_dorientation;

    std::vector<double> _cal_imur_transacc;
    std::vector<double> _cal_imur_orientation_3d;
    double _cal_imur_orientation;
    std::vector<double> _cal_imur_dorientation_3d;
    double _cal_imur_dorientation;

    std::vector<double> _cal_enc_pose;
    std::vector<double> _cal_motor_current;
    std::vector<double> _cal_motor_voltage;
    std::vector<double> _cal_velocity;
    std::vector<double> _cmd_velocity;

    std::vector<double> _est_pose;
    std::vector<double> _est_velocity;
    std::vector<double> _est_acceleration;

    double _control_sample_rate;
    int _lidar_data_length;

  protected:
    virtual bool estimateUpdate() = 0;
    virtual bool initialize() = 0;
    bool _transmit_estimate;

    double getControlSampleRate();
    int getLidarDataLength();

    std::vector<std::vector<double> > getLidarData();
    std::vector<double> getImuLTransAcc();
    double getImuLOrientation();
    std::vector<double> getImuL3dOrientation();
    double getImuLDOrientation();
    std::vector<double> getImuL3dDOrientation();
    std::vector<double> getImuRTransAcc();
    double getImuROrientation();
    std::vector<double> getImuR3dOrientation();
    double getImuRDOrientation();
    std::vector<double> getImuR3dDOrientation();
    std::vector<double> getEncPose();
    std::vector<double> getMotorCurrent();
    std::vector<double> getMotorVoltage();
    std::vector<double> getCalVelocity();
    std::vector<double> getCmdVelocity();

    void setEstPose(std::vector<double> const&);
    void setEstVelocity(std::vector<double> const&);
    void setEstAcceleration(std::vector<double> const&);

  public:
    EstimatorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    void writeSample();
};
#endif
