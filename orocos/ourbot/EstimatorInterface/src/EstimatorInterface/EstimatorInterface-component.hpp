#ifndef OROCOS_ESTIMATORINTERFACE_COMPONENT_HPP
#define OROCOS_ESTIMATORINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class EstimatorInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _cal_lidar_distances_port;
    InputPort<std::vector<double> > _cal_lidar_angles_port;
    InputPort<std::vector<double> > _cal_ir_distances_port;
    InputPort<std::vector<double> > _cal_ir_angles_port;
    InputPort<std::vector<double> > _cal_imul_transacc_port;
    InputPort<double> _cal_imul_dorientation_port;
    InputPort<double> _cal_imul_orientation_port;
    InputPort<std::vector<double> > _cal_imur_transacc_port;
    InputPort<double> _cal_imur_dorientation_port;
    InputPort<double> _cal_imur_orientation_port;
    InputPort<std::vector<double> > _cal_enc_pose_port;
    InputPort<std::vector<double> > _cal_motor_current_port;
    InputPort<std::vector<double> > _cal_velocity_port;

    OutputPort<std::vector<double> > _est_pose_port;
    OutputPort<std::vector<double> > _est_velocity_port;
    OutputPort<std::vector<double> > _est_acceleration_port;
    OutputPort<std::vector<double> > _est_global_offset_port;      // estimated offset of initial frame wrt world frame
    OutputPort<std::vector<double> > _map_obstacles_port;          // obstacles wrt to world frame

    std::vector<double> _cal_lidar_distances;
    std::vector<double> _cal_lidar_angles;
    std::vector<double> _cal_ir_distances;
    std::vector<double> _cal_ir_angles;
    std::vector<double> _cal_imul_transacc;
    double _cal_imul_dorientation;
    double _cal_imul_orientation;
    std::vector<double> _cal_imur_transacc;
    double _cal_imur_dorientation;
    double _cal_imur_orientation;
    std::vector<double> _cal_enc_pose;
    std::vector<double> _cal_motor_current;
    std::vector<double> _cal_velocity;

    std::vector<double> _est_pose;
    std::vector<double> _est_velocity;
    std::vector<double> _est_acceleration;
    std::vector<double> _est_global_offset;
    std::vector<double> _map_obstacles;

    double _control_sample_rate;
    int _lidar_data_length;
    int _ir_data_length;
    int _obs_data_length;

  protected:
    virtual bool estimateUpdate() = 0;
    virtual bool initialize() = 0;

    double getControlSampleRate();
    int getLidarDataLength();
    int getIRDataLength();
    int getObsDataLength();

    std::vector<std::vector<double> > getLidarData();
    std::vector<std::vector<double> > getIRData();
    std::vector<double> getImuLTransAcc();
    double getImuLDOrientation();
    double getImuLOrientation();
    std::vector<double> getImuRTransAcc();
    double getImuRDOrientation();
    double getImuROrientation();
    std::vector<double> getEncPose();
    std::vector<double> getMotorCurrent();
    std::vector<double> getCalVelocity();

    void setEstPose(std::vector<double> const&);
    void setEstVelocity(std::vector<double> const&);
    void setEstAcceleration(std::vector<double> const&);
    void setEstGlobalOffset(std::vector<double> const&);
    void setObstacleData(std::vector<double> const&);

  public:
    EstimatorInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
};
#endif