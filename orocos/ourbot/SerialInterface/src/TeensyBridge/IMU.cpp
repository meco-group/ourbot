#include "IMU.hpp"
#include <iostream>

IMU::IMU(std::string const& name):
	TaskContext(name)
{
	//Add ports
	ports()->addPort("raw_imu_acc_port",              _raw_imu_acc_port).doc("Raw accelerometer data from IMU");
	ports()->addPort("raw_imu_gyr_port",              _raw_imu_gyr_port).doc("Raw gyroscope data from IMU");
	ports()->addPort("raw_imu_mag_port",              _raw_imu_mag_port).doc("Raw magnetometer data from IMU");

	ports()->addPort("cal_imu_transacc_port",         _cal_imu_transacc_port).doc       ("Translational acceleration from IMU");
	ports()->addPort("cal_imu_orientation_3d_port",   _cal_imu_orientation_3d_port).doc ("3D orientation from IMU");
	ports()->addPort("cal_imu_dorientation_3d_port",  _cal_imu_dorientation_3d_port).doc("3D rotational velocity from IMU");
	ports()->addPort("cal_imu_orientation_port",      _cal_imu_orientation_port).doc    ("1D orientation from IMU");
	ports()->addPort("cal_imu_dorientation_port",     _cal_imu_dorientation_port).doc   ("1D rotational velocity from IMU");

	//Set data port sizes
	std::vector<double> example = std::vector<double>(3,0.0);
	_raw_imu_acc_port.setDataSample(example);
	_raw_imu_gyr_port.setDataSample(example);
	_raw_imu_mag_port.setDataSample(example);

	_cal_imu_transacc_port.setDataSample(example);
	_cal_imu_orientation_3d_port.setDataSample(example);
	_cal_imu_dorientation_3d_port.setDataSample(example);
}

//Define methods:
//--------------

void IMU::updateMeasurements(std::vector<double> acc, std::vector<double> gyr, std::vector<double> mag)
{
	_raw_imu_acc_port.write(acc);
	_raw_imu_gyr_port.write(gyr);
	_raw_imu_mag_port.write(mag);

	//Todo: compensate for roll/pitch/yaw in angular velocity measurements? No, better translate to world frame when using these measurements
	//Todo: or make separate variables for measurements in gauss and rpy angles?
	//Calibrate ports and write to output ports
	_cal_imu_transacc_port.write(_acc.calibrate(acc));
	_cal_imu_dorientation_3d_port.write(_gyr.calibrate(gyr));
	_cal_imu_dorientation_port.write(_gyr.getCalib().at(2));
	_mag.calibrate(mag);

	//compute rpy angles
	std::vector<double> rpy = computeRPY();

	_cal_imu_orientation_3d_port.write(rpy);
	_cal_imu_orientation_port.write(rpy.at(2));
	//_cal_imu_temperature_port.write     (_cal_imu_temperature[0]);
}

std::vector<double> IMU::computeRPY(){
	//Reference: https://www.pololu.com/file/download/LSM303DLH-compass-app-note.pdf?file_id=0J434
	std::vector<double> rpy(3); //contains roll pitch yaw angles
	std::vector<double> acc = _acc.getCalib(); //copy accelerometer data
	std::vector<double> mag = _mag.getCalib(); //copy magnetometer data

	IMU_DEBUG_PRINT("acceleration in rpy: "<<acc[0]<<" , "<<acc[1]<<" , "<<acc[2])
	IMU_DEBUG_PRINT("magnetic field in rpy: "<<mag[0]<<" , "<<mag[1]<<" , "<<mag[2])

	//Normalize accelerometer measurements
	double acc_norm=sqrt(pow(acc[0],2) + pow(acc[1],2) + pow(acc[2],2));
	acc[0]=acc[0]/acc_norm;
	acc[1]=acc[1]/acc_norm;
	acc[2]=acc[2]/acc_norm;
	//Normalize magnetometer measurements
	double mag_norm=sqrt(pow(mag[0],2) + pow(mag[1],2) + pow(mag[2],2));
	mag[0]=mag[0]/mag_norm;
	mag[1]=mag[1]/mag_norm;
	mag[2]=mag[2]/mag_norm;

	IMU_DEBUG_PRINT("acc normalized: "<<acc[0]<<" , "<< acc[1]<<" , "<<acc[2])
	IMU_DEBUG_PRINT("mag normalized: "<<mag[0]<<" , "<< mag[1]<<" , "<<mag[2])

	//Roll pitch yaw calculation:
	rpy[1] = asin(-acc[0]); //pitch
	rpy[0] = asin( acc[1]/cos(rpy[1])); //roll

	//Tilt compensation for heading/yaw calculation:
	double magx =  mag[0]*cos(rpy[1]) + mag[2]*sin(rpy[1]);
	double magy =  mag[0]*sin(rpy[0])*sin(rpy[1]) + mag[1]*cos(rpy[0]) - mag[2]*sin(rpy[0])*cos(rpy[1]);
	// double magz = -mag[0]*cos(rpy[0])*sin(rpy[1]) + mag[1]*sin(rpy[0]) + mag[2]*cos(rpy[0])*cos(rpy[1]); //not used

	//Option 1: this puts the heading in the range [0...2*pi]
	// if (magx > 0 and magy >= 0){
	// 	rpy[2] = atan(magy / magx);
	// }
	// else if (magx < 0 ){
	// 	rpy[2] = pi + atan(magy / magx);
	// }
	// else if (magx > 0 and magy <= 0 ){
	// 	rpy[2] = 2*pi + atan(magy / magx);
	// }
	// else if (magx = 0 and magy < 0 ){
	// 	rpy[2] = pi/2.0;
	// }
	// else if (magx < 0 and magy > 0 ){
	// 	rpy[2] = 3*pi/2.0;
	// }
	//Option 2: this puts the heading in the range [-pi...pi]
	rpy[2] = atan2(magy,magx);
	if (rpy[2]<0) { //shift to [0...2*pi]
		rpy[2] += 2*M_PI;
	}
	IMU_DEBUG_PRINT("roll: "<<rpy[0]<<" , "<<"pitch: "<<rpy[1]<<" , "<<"yaw: "<<rpy[2])
	return rpy;
}

uint8_t IMU::getID()
{
	return _ID;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(IMU)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
 ORO_LIST_COMPONENT_TYPE(IMU)
//ORO_CREATE_COMPONENT(IMU)
