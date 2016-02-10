#ifndef OROCOS_IMU_HPP
#define OROCOS_IMU_HPP

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <cmath>
#include "sensor3d.hpp"

// #define IMU_TESTFLAG //to manually set properties while testing
// #define IMU_DEBUGFLAG

#ifdef IMU_DEBUGFLAG
	#define IMU_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define IMU_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

class IMU : public RTT::TaskContext{
  private:
	uint8_t _ID;
	
	//Define output ports
  	//Raw data
	RTT::OutputPort<std::vector<double> > _raw_imu_acc_port; //accelerometer data input port
	RTT::OutputPort<std::vector<double> > _raw_imu_gyr_port; //gyroscope data input port
  	RTT::OutputPort<std::vector<double> > _raw_imu_mag_port; //magnetometer data input port
  	//Calibrated data
	RTT::OutputPort<std::vector<double> > _cal_imu_transacc_port;		    //translational acceleration port
	RTT::OutputPort<std::vector<double> > _cal_imu_dorientation_3d_port; //angular velocity (3d) port
  	RTT::OutputPort<std::vector<double> > _cal_imu_orientation_3d_port;  //orientation (3d) port
 	RTT::OutputPort<double>               _cal_imu_dorientation_port;    //angular velocity around z-axis port
  	RTT::OutputPort<double>               _cal_imu_orientation_port;     //orientation in x-y plane port

  	//Define properties:
  	//for calibration
		/*std::vector<double> _acc_offset; //user-defined offset of measurements
		std::vector<double> _gyr_offset;
		std::vector<double> _mag_offset;
		double _tmp_offset;
		std::vector<double> _acc_scale;  //user-defined scaling of measurements
		std::vector<double> _gyr_scale;
		std::vector<double> _mag_scale;
		double _tmp_scale;*/


  	std::vector<double> computeRPY(); //convert magnetic field measurements to roll pitch yaw angles
		
  public:
    IMU(std::string const& name); //Constructor

  	Sensor3D 	_acc; //accelerometer
  	Sensor3D 	_mag; //magnetometer
  	Sensor3D 	_gyr; //gyroscope
  	Sensor3D 	_tmp; //temperature sensor
  
  	void updateMeasurements(std::vector<double> acc, std::vector<double> gyr, std::vector<double> mag);
  	uint8_t getID();
  
	bool isConnected(); //Check if sensor is connected by reading its ID
	std::vector<double> getImuTransAcc();
    std::vector<double> getImuOrientation3D();
    std::vector<double> getImuDOrientation3D();
    double getImuOrientation();
    double getImuDOrientation();

};
#endif //IMU
