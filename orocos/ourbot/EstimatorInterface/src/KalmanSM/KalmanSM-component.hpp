#ifndef OROCOS_KALMANSM_COMPONENT_HPP
#define OROCOS_KALMANSM_COMPONENT_HPP

#include "../EstimatorInterface/EstimatorInterface-component.hpp"
#include <Eigen/Dense>

class KalmanSM : public EstimatorInterface{

    private:
        // Input ports
        InputPort<std::vector<double> > _cor_lidar_angle_port;
        InputPort<std::vector<double> > _scanmatch_pose_port;
        InputPort<std::vector<double> > _scanmatch_covariance_port;

        // Output ports
        OutputPort<bool> _trigger_scanmatcher_port;

        // Measurements
        std::vector<double> _enc_pose;
        std::vector<double> _enc_pose_prev;
        std::vector<double> _cmd_velocity;
        std::vector<double> _left_acc;
        std::vector<double> _right_acc;
        std::vector<double> _scan_pose_rel; // from scanmatcher
        std::vector<double> _prev_scan_pose_abs;
        std::vector<double> _cov_scanmatch;

        // Estimated pose
        std::vector<double> _est_pose;

        std::vector<double> _sensorinformation;
        std::vector<double> _corresponding_lidar_angles;
        std::vector<std::vector<double> > _sensorbuffer;
        std::vector<std::vector<double> > _sensorbuffer2;

        // Buffers for the accelero-filtering
        std::vector<double> _buff_imu_r_x_x;
        std::vector<double> _buff_imu_r_y_x;
        std::vector<double> _buff_imu_l_x_x;
        std::vector<double> _buff_imu_l_y_x;
        std::vector<double> _buff_imu_r_x_y;
        std::vector<double> _buff_imu_r_y_y;
        std::vector<double> _buff_imu_l_x_y;
        std::vector<double> _buff_imu_l_y_y;

        // Filter parameters
        std::vector<double> _filt_a;
        std::vector<double> _filt_b;


        // Kalman vectors and matrices
        Eigen::VectorXf _state;
        Eigen::VectorXf _state_prev;
        Eigen::VectorXf _state_at_scanstart;
        Eigen::MatrixXf _F;
        Eigen::MatrixXf _F_T;
        Eigen::MatrixXf _P;
        Eigen::MatrixXf _P_at_scanstart;
        Eigen::MatrixXf _Q;

        Eigen::MatrixXf _H;
        Eigen::MatrixXf _H_T;
        Eigen::MatrixXf _H_hf;
        Eigen::MatrixXf _H_T_hf;
        Eigen::VectorXf _z;
        Eigen::VectorXf _y;
        Eigen::VectorXf _y_hf;
        Eigen::MatrixXf _S;
        Eigen::MatrixXf _S_hf;
        Eigen::MatrixXf _S_inv;
        Eigen::MatrixXf _S_inv_hf;
        Eigen::MatrixXf _R;
        Eigen::MatrixXf _R_hf;
        Eigen::MatrixXf _K;
        Eigen::MatrixXf _K_hf;
        Eigen::MatrixXf _I;
        Eigen::MatrixXf _H_lf;
        Eigen::MatrixXf _R_lf;

        // Other
        double _accelero_mounting_distance;
        double _dt;
        bool _first_iteration;
        bool _got_scan;
        bool _acc_calibrated;
        double _acc_scaler;
        double _left_bias_x;
        double _left_bias_y;
        double _right_bias_x;
        double _right_bias_y;

  public:
        KalmanSM(std::string const& name);

        bool estimateUpdate();
        bool initialize();
        void setInitData();
        void gatherMeasurements();
        void detectScanMatchInfo();
        void updateKalman();
        double filter(double value, std::vector<double> *a, std::vector<double> *b, std::vector<double> *buff_x, std::vector<double> *buff_y);
        void replaceVectorBlock(Eigen::VectorXf *block, Eigen::VectorXf *vect, int startindex, int endindex);
};

#endif
