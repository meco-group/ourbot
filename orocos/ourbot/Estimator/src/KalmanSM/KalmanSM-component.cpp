#include "KalmanSM-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <math.h>

using namespace Eigen;

KalmanSM::KalmanSM(std::string const& name) : EstimatorInterface(name),
_enc_pose(3), _enc_pose_prev(3), _cmd_velocity(3), _left_acc(3), _right_acc(3), _scan_pose_rel(3),
_prev_scan_pose_abs(3), _cov_scanmatch(9),
_est_pose(3), _sensorinformation(10), _corresponding_lidar_angles(400),
_buff_imu_r_x_x(3), _buff_imu_r_y_x(3), _buff_imu_l_x_x(3), _buff_imu_l_y_x(3),
_buff_imu_r_x_y(3), _buff_imu_r_y_y(3), _buff_imu_l_x_y(3), _buff_imu_l_y_y(3),
_filt_a(3), _filt_b(3),
_state(16), _state_prev(16), _state_at_scanstart(16), _F(16,16), _F_T(16,16),
_P(16,16), _P_at_scanstart(16,16), _Q(16,16), _H(10,16), _H_T(16,10),
_H_hf(7,16), _H_T_hf(16,7), _z(10), _y(10), _y_hf(7), _S(10,10), _S_hf(7,7),
_S_inv(10,10), _S_inv_hf(7,7), _R(10,10), _R_hf(7,7), _K(16,10), _K_hf(16,7),
_I(16,16), _H_lf(3,3), _R_lf(3,3)
{
  _accelero_mounting_distance = 0.10738;
  // Input ports
  ports()->addPort("cor_lidar_angle_port",_cor_lidar_angle_port).doc("Input port for corrected lidar node positions. Holds a vector of rplidar_node_buffer_size angles [m]");
  ports()->addPort("scanmatch_pose_port", _scanmatch_pose_port).doc("Estimated position-change of scanmatcher [x,y,orientation]");
  ports()->addPort("scanmatch_covariance_port", _scanmatch_covariance_port).doc("covariance scanmatcher [x,y,orientation]");
  // Output ports (to trigger SM)
  ports()->addPort("trigger_scanmatcher_port",_trigger_scanmatcher_port).doc("Output port which triggers scanmatcher");
  //@@@Michiel: add extra output port with pose at start scan
  // ports()->addPort("scanstart_pose_port", _scanstart_pose_port).doc("Pose at start of a scan");
  // Properties
  addProperty("accelero_mounting_distance", _accelero_mounting_distance).doc("Distance between the accelerometer and the center of the robot");
  // Set data vector, matrices, ... with initial values
  setInitData();
  std::cout << "KalmanSM constructed !" <<std::endl;
}

bool KalmanSM::initialize(){
  _dt = 1.0/getControlSampleRate();
  return true;
}

bool KalmanSM::estimateUpdate(){
  gatherMeasurements();
  detectScanMatchInfo();
  updateKalman();
  setEstPose(_est_pose);
  return true;
}

void KalmanSM::gatherMeasurements(){
  // measurements
  _enc_pose = getEncPose();
  _cmd_velocity = getCmdVelocity();
  _left_acc = getImuLTransAcc();
  _right_acc = getImuRTransAcc();

  // when new lidar data: trigger scanmatcher and save current state, covariance and abs pose
  if(_cor_lidar_angle_port.read(_corresponding_lidar_angles) == RTT::NewData){
    _trigger_scanmatcher_port.write(true);
    _state_at_scanstart = _state_prev;
    // @@@Michiel
    // _scanstart_pose_port.write(_state_prev);
    _P_at_scanstart = _P;
    // clear buffer: this buffer will save all information until scanmatcher writes a new update (i.e. is finished)
    _sensorbuffer.clear();
  }
  // save measurements in _sensorinformation
  if(_first_iteration){
    _enc_pose_prev = _enc_pose;
    _first_iteration = false;
  }
  _sensorinformation[0] = _cmd_velocity[0];
  _sensorinformation[1] = _cmd_velocity[1];
  _sensorinformation[2] = _cmd_velocity[2];
  _sensorinformation[3] = _enc_pose[0] - _enc_pose_prev[0];
  _sensorinformation[4] = _enc_pose[1] - _enc_pose_prev[1];
  _sensorinformation[5] = _enc_pose[2] - _enc_pose_prev[2];
  _enc_pose_prev = _enc_pose;
  // search bias of accelerations: by averaging while robot stands still at begin
  if(!_acc_calibrated && sqrt(_sensorinformation[3]*_sensorinformation[3] + _sensorinformation[4]*_sensorinformation[4]) < 0.001 && _sensorinformation[5] < 0.001){
    _left_bias_x += _left_acc[0];
    _left_bias_y += _left_acc[1];
    _right_bias_x += _right_acc[0];
    _right_bias_y += _right_acc[1];
    _acc_scaler++;
  }
  else if(!_acc_calibrated){
    _left_bias_x /= _acc_scaler;
    _left_bias_y /= _acc_scaler;
    _right_bias_x /= _acc_scaler;
    _right_bias_y /= _acc_scaler;
    _acc_calibrated = true;
  }
  else{
    // filter accelerometer information with butterworth filter designed in Matlab
    _left_acc[0] = filter(_left_acc[0], &_filt_a, &_filt_b, &_buff_imu_l_x_x, &_buff_imu_l_x_y);
    _left_acc[1] = filter(_left_acc[1], &_filt_a, &_filt_b, &_buff_imu_l_y_x, &_buff_imu_l_y_y);
    _right_acc[0] = filter(_right_acc[0], &_filt_a, &_filt_b, &_buff_imu_r_x_x, &_buff_imu_r_x_y);
    _right_acc[1] = filter(_right_acc[1], &_filt_a, &_filt_b, &_buff_imu_r_y_x, &_buff_imu_r_y_y);

    _sensorinformation[6] = _left_acc[0] - _left_bias_x;
    _sensorinformation[7] = _left_acc[1] - _left_bias_y;
    _sensorinformation[8] = _right_acc[0] - _right_bias_x;
    _sensorinformation[9] = _right_acc[1] - _right_bias_y;

    // save calibrated measurments to sensorbuffer
    _sensorbuffer.push_back(_sensorinformation);
  }
}

void KalmanSM::detectScanMatchInfo(){
  if(_scanmatch_pose_port.read(_scan_pose_rel) == RTT::NewData){
    _sensorbuffer2 = _sensorbuffer;
    _state_prev = _state_at_scanstart;
    _P = _P_at_scanstart;
    _got_scan = true;
  }
  else{
    if(_sensorbuffer.size() >= 1){
      _sensorbuffer2.push_back(_sensorbuffer[_sensorbuffer.size()-1]);
    }
    else {
      _sensorbuffer2 = _sensorbuffer;
    }
    _got_scan = false;
  }
}

void KalmanSM::updateKalman(){
  double dx_rel, dy_rel, dtheta_rel, theta;
  for(unsigned int i = 0; i<_sensorbuffer2.size(); i++) {
    _sensorinformation = _sensorbuffer2[i];
    // prediction step: evaluate model equation
    dx_rel = _sensorinformation[0]*_dt;
    dy_rel = _sensorinformation[1]*_dt;
    dtheta_rel = _sensorinformation[2]*_dt;
    theta = _state_prev(12);
    _state(0) = _sensorinformation[0];
    _state(1) = _sensorinformation[1];
    _state(2) = _sensorinformation[2];
    _state(3) = (_sensorinformation[0] - _state_prev(0))/_dt;
    _state(4) = (_sensorinformation[1] - _state_prev(1))/_dt;
    _state(5) = (_sensorinformation[2] - _state_prev(2))/_dt;
    _state(6) = _state_prev(6);
    _state(7) = _state_prev(7);
    _state(8) = _state_prev(8);
    _state(9) = _state_prev(9);
    _state(10) = _state_prev(10) + dx_rel*cos(theta) - dy_rel*sin(theta);
    _state(11) = _state_prev(11) + dx_rel*sin(theta) + dy_rel*cos(theta);
    _state(12) = _state_prev(12) + dtheta_rel;
    _state(13) = _state_prev(10);
    _state(14) = _state_prev(11);
    _state(15) = _state_prev(12);

    // Jacobian of model equation
    _F << 0,  0,  0,        0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
          0,  0,  0,        0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
          0,  0,  0,        0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,

          -1/_dt,  0,  0,   0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
          0,  -1/_dt,  0,   0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
          0,  0,  -1/_dt,   0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,

          0,  0,  0,        0,  0,  0,    1,  0,  0,  0,    0,  0,  0,    0,  0,  0,
          0,  0,  0,        0,  0,  0,    0,  1,  0,  0,    0,  0,  0,    0,  0,  0,
          0,  0,  0,        0,  0,  0,    0,  0,  1,  0,    0,  0,  0,    0,  0,  0,
          0,  0,  0,        0,  0,  0,    0,  0,  0,  1,    0,  0,  0,    0,  0,  0,

          0,  0,  0,        0,  0,  0,    0,  0,  0,  0,    1,  0, -dx_rel*sin(theta)-dy_rel*cos(theta),    0, 0, 0,
          0,  0,  0,        0,  0,  0,    0,  0,  0,  0,    0,  1,  dx_rel*cos(theta)-dy_rel*sin(theta),    0, 0, 0,
          0,  0,  0,        0,  0,  0,    0,  0,  0,  0,    0,  0,  1,                                      0, 0, 0,

          0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    1,  0,  0,    0, 0, 0,
          0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  1,  0,    0, 0, 0,
          0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  1,    0, 0, 0;

    // a priori covaricance of state
    _F_T = _F.transpose();
    _P = _F*_P*_F_T + _Q;

    // correction step: incorporate measurments
    _z << _sensorinformation[3], _sensorinformation[4], _sensorinformation[5], _sensorinformation[6], _sensorinformation[7], _sensorinformation[8], _sensorinformation[9];
    _y_hf(0) = _z(0) - ( (_state(10)-_state(13))*cos(theta) + (_state(11)-_state(14))*sin(theta));
    _y_hf(1) = _z(1) - (-(_state(10)-_state(13))*sin(theta) + (_state(11)-_state(14))*cos(theta));;
    _y_hf(2) = _z(2) - (_state(12) - _state(15));
    _y_hf(3) = _z(3) - (_state(3) - _accelero_mounting_distance*_state(5) + _state(6));
    _y_hf(4) = _z(4) - (_state(4) + _state(7));
    _y_hf(5) = _z(5) - (_state(3) + _accelero_mounting_distance*_state(5) + _state(8));
    _y_hf(6) = _z(6) - (_state(4) + _state(9));

    // Jacobian of measurement equation
    _H_hf <<  0,  0,  0,    0,  0,  0,    0,  0,  0,  0,     cos(theta), sin(theta), -(_state(10)-_state(13))*sin(theta) + (_state(11)-_state(14))*cos(theta),   -cos(theta), -sin(theta),  0,
              0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    -sin(theta), cos(theta), -(_state(10)-_state(13))*cos(theta) - (_state(11)-_state(14))*sin(theta),    sin(theta), -cos(theta),  0,
              0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  1,          0,  0,  -1,

              0,  0,  0,    1,  0,  -_accelero_mounting_distance, 1,  0,  0,  0,    0,  0,  0,    0,  0,  0,
              0,  0,  0,    0,  1,  0,                            0,  1,  0,  0,    0,  0,  0,    0,  0,  0,
              0,  0,  0,    1,  0,  _accelero_mounting_distance,  0,  0,  1,  0,    0,  0,  0,    0,  0,  0,
              0,  0,  0,    0,  1,  0,                            0,  0,  0,  1,    0,  0,  0,    0,  0,  0;

    _H_T_hf = _H_hf.transpose();

    if(_got_scan){
      // std::cout << "Received scanmatch data ... " << std::endl;
      replaceVectorBlock(&_y_hf, &_y, 0, 6);
      // extra measurement equations
      theta = _prev_scan_pose_abs[2];

      _y(7) = _scan_pose_rel[0] - ( (_state(10)-_prev_scan_pose_abs[0])*cos(theta) + (_state(11)-_prev_scan_pose_abs[1])*sin(theta));
      _y(8) = _scan_pose_rel[1] - (-(_state(10)-_prev_scan_pose_abs[0])*sin(theta) + (_state(11)-_prev_scan_pose_abs[1])*cos(theta));
      _y(9) = _scan_pose_rel[2] - (_state(12)-_prev_scan_pose_abs[2]);

      // @@@Michiel
      // _y(7) = _scan_pose_rel[0];
      // _y(8) = _scan_pose_rel[1];
      // _y(9) = _scan_pose_rel[2];

      _scanmatch_covariance_port.read(_cov_scanmatch);

      // subblock of measurment noise regarding scanmatch
      _R_lf <<  _cov_scanmatch[0]*100000000, _cov_scanmatch[1]*100000000, _cov_scanmatch[2]*100000000,
                _cov_scanmatch[3]*100000000, _cov_scanmatch[4]*100000000, _cov_scanmatch[5]*100000000,
                _cov_scanmatch[6]*100000000, _cov_scanmatch[7]*100000000, _cov_scanmatch[8]*100000000;
      _R.block<3,3>(7,7) = _R_lf;

      // subblock of jacobian regarding extra measurement equations
      _H_lf <<   cos(theta), sin(theta), 0,
                -sin(theta), cos(theta), 0,
                 0.,      0.,      1.;

      // @@@Michiel
      // _H_lf << 1., 0., 0.,
      //          0., 1., 0.,
      //          0., 0., 1.;

      _H.block<3,3>(7,10) = _H_lf;
      _H.block<7,16>(0,0) = _H_hf;

      _H_T = _H.transpose();
      _S = _H*_P*_H_T + _R;
      _S_inv = _S.inverse();
      _K = _P*_H_T*_S_inv;
      _state = _state + _K*_y;
      _P = (_I-_K*_H)*_P;

      // save absolute pose
      _prev_scan_pose_abs[0] = _state(10);
      _prev_scan_pose_abs[1] = _state(11);
      _prev_scan_pose_abs[2] = _state(12);

      _got_scan = false;

    }
    else{
      _S_hf = _H_hf*_P*_H_T_hf + _R_hf;//7x7
      _S_inv_hf = _S_hf.inverse();//7x7
      _K_hf = _P*_H_T_hf*_S_inv_hf;//16x16 * 16x7 * 7x7 = 16x7
      _state = _state + _K_hf*_y_hf;
      _P = (_I-_K_hf*_H_hf)*_P;
    }

    _state_prev = _state;

    _est_pose[0] = _state(10);
    _est_pose[1] = _state(11);
    _est_pose[2] = _state(12);
  }
  _sensorbuffer2.clear();
}

double KalmanSM::filter(double value, std::vector<double> *a, std::vector<double> *b, std::vector<double> *buff_x, std::vector<double> *buff_y){
  int order = a->size();
  std::vector<double>::iterator it;
  std::vector<double>::iterator it2;
  double filt_data = 0;
  buff_x->erase(buff_x->end()-1);
  buff_x->insert(buff_x->end(),value);
  double scaler = a->at(0);
  a->erase(a->begin());
  a->insert(a->begin(),0);
  for(unsigned int j = 0; j < a->size(); j++){
    filt_data =  filt_data + b->at(j)*buff_x->at(order-j-1) - a->at(order-j-1)*buff_y->at(j);
  }
  filt_data = filt_data/scaler;
  a->erase(a->begin());
  a->insert(a->begin(),scaler);
  // Shifting the values in the buffer and adding calculated values
  it = buff_x->begin();
  it2 = buff_y->begin();
  for(int i = 0 ; i < order-1;i++){
        if(i!=order-2){
            buff_y->erase(it2);
            buff_y->insert(buff_y->begin()+i,buff_y->at(i));
        }
        buff_x->erase(it);
        buff_x->insert(buff_x->begin()+i,buff_x->at(i));
    it++;
    it2++;
  }
  buff_y->erase(buff_y->begin()+order-2);
  buff_x->erase(buff_x->begin()+order-1);
  buff_y->insert(buff_y->begin()+order-2, filt_data);
  buff_x->insert(buff_x->begin()+order-1, buff_x->at(order-2));
  return filt_data;
}

void KalmanSM::replaceVectorBlock(Eigen::VectorXf *block, Eigen::VectorXf *vect, int startindex, int endindex){
  int index = 0;
  for(int i = startindex; i <= endindex; i++){
    (*vect)(i) = (*block)(index);
    index++;
  }
}

void KalmanSM::setInitData(){
  _state.setZero();
  _state_prev.setZero();
  _state_at_scanstart.setZero();
  _P_at_scanstart.setZero();
  _P.setZero();
  _Q.setZero();
  _F.setZero();
  _F_T.setZero();
  _H.setZero();
  _H_T.setZero();
  _H_hf.setZero();
  _H_T_hf.setZero();
  _z.setZero();
  _y.setZero();
  _y_hf.setZero();
  _R.setZero();
  _R_lf.setZero();
  _R_hf.setZero();
  _S.setZero();
  _S_hf.setZero();
  _S_inv.setZero();
  _S_inv_hf.setZero();
  _K.setZero();
  _K_hf.setZero();
  _I.setIdentity();
  _H_lf.setZero();


  _P<<  0.000001, 0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0.000001, 0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0.000001,   0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,

        0,  0,  0,    0.000001, 0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0.000001, 0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0.000001,   0,  0,  0,  0,    0,  0,  0,    0,  0,  0,

        0,  0,  0,    0,  0,  0,    0.000001, 0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0.000001, 0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0.000001, 0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0.000001,   0,  0,  0,    0,  0,  0,

        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0.000001, 0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0.000001, 0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0.000001,   0,  0,  0,

        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0.00000001, 0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0.00000001, 0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0.00000001;

  _Q<<  0.1,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0.1,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0.1,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,

        0,  0,  0,    0.1,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0.1,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0.1,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0,

        0,  0,  0,    0,  0,  0,    0.00001,  0,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0.00001,  0,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0.00001,  0,    0,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0.00001,    0,  0,  0,    0,  0,  0,

        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0.1,  0,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0.1,  0,    0,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0.1,    0,  0,  0,

        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0.1,  0,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0.1,  0,
        0,  0,  0,    0,  0,  0,    0,  0,  0,  0,    0,  0,  0,    0,  0,  0.1;

  _R_hf <<  0.00001,   0,      0,      0,      0,      0,      0,
            0,      0.00001,  0,      0,      0,      0,      0,
            0,      0,      0.00001,  0,      0,      0,      0,
            0,      0,      0,      0.1,    0,      0,      0,
            0,      0,      0,      0,      0.1,    0,      0,
            0,      0,      0,      0,      0,      0.1,    0,
            0,      0,      0,      0,      0,      0,      0.1;

  _R.block<7,7>(0,0) = _R_hf;
  _H_T_hf = _H_hf.transpose();

  // ButterWorth parameters for filter
  _filt_a[0] = 1.;
  _filt_a[1] = -1.9556;
  _filt_a[2] = 0.9565;
  _filt_b[0] = 0.0002414;
  _filt_b[1] = 0.0004827;
  _filt_b[2] = 0.0002414;

  _first_iteration = true;
  _got_scan = false;
  _acc_calibrated = false;
  _acc_scaler = 0;
  _left_bias_x = 0;
  _left_bias_y = 0;
  _right_bias_x = 0;
  _right_bias_y = 0;
}

ORO_LIST_COMPONENT_TYPE(KalmanSM);
