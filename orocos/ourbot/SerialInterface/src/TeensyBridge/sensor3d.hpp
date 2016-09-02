#ifndef SENSOR3D_HPP
#define SENSOR3D_HPP

#include <iostream>
#include <vector>
#include <stdint.h>

//#define SENSOR3D_DEBUGFLAG

#ifdef SENSOR3D_DEBUGFLAG
	#define SENSOR3D_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define SENSOR3D_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

//This is a general Sensor3D class. In the IMU Orocos component implements all specific aspects.

class Sensor3D
{
	friend class IMU;
	friend class TeensyBridge;

	private:
		std::vector<double> _calib;		//calibrated output data
		std::vector<double> _raw;		//(raw) sensor data
		std::vector<double> _offset;	//user-defined offset
		std::vector<double> _scale;		//user-defined scaling factor

	public:
    	Sensor3D();
    	
		std::vector<double> calibrate();
		std::vector<double> calibrate(std::vector<double> raw);
		void setOffset(double offset_x, double offset_y, double offset_z);
		void setScale(double scale_x, double scale_y, double scale_z);
		
		std::vector<double> getCalib();
		std::vector<double> getRaw();
};

#endif //SENSOR3D
