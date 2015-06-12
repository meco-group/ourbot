#ifndef OROCOS_ESTIMATORINTERFACE_COMPONENT_HPP
#define OROCOS_ESTIMATORINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

class EstimatorInterface : public RTT::TaskContext{
  private:
    InputPort<std::vector<double> > _cal_lidar_distances_inport;
    InputPort<std::vector<double> > _cal_lidar_angles_inport;
    InputPort<std::vector<double> > _cal_ir_distances_inport;
    InputPort<std::vector<double> > _cal_ir_angles_inport;
    InputPort<std::vector<double> > _cal_imul_transacc_inport;
    InputPort<double> _cal_imul_dorientation_inport;
    InputPort<double> _cal_imul_orientation_inport;
    InputPort<std::vector<double> > _cal_imur_transacc_inport;
    InputPort<double> _cal_imur_dorientation_inport;
    InputPort<double> _cal_imur_orientation_inport;
    InputPort<std::vector<double> > _cal_enc_pose_inport;
    InputPort<std::vector<double> > _cal_motor_current_inport;
    InputPort<std::vector<double> > _cmd_velocity_inport;

    OutputPort<std::vector<double> > _est_pose_outport;
    OutputPort<std::vector<double> > _est_velocity_outport;
    OutputPort<std::vector<double> > _est_acceleration_outport;

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
    std::vector<double> _cmd_velocity;
    std::vector<double> _est_pose;
    std::vector<double> _est_velocity;
    std::vector<double> _est_acceleration;

    int _sample_rate;
    int _lidar_data_length;
    int _nrof_ir;

  protected:
    virtual bool estimateUpdate() = 0;
    virtual bool initialize() = 0;

    int getSampleRate();
    int getLidarDataLength();
    int getNrofIR();

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
};
#endif
