#ifndef SENSOR3D_HPP
#define SENSOR3D_HPP

#include <iostream>
#include <vector>
#include <stdint.h>

// #define SENSOR3D_DEBUGFLAG

#ifdef SENSOR3D_DEBUGFLAG
	#define SENSOR3D_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define SENSOR3D_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

//This is a general Sensor3D class. In the IMU Orocos component implements all specific aspects.

class Sensor3D
{
	private:
		const uint8_t _ID; 					//sensor ID
		// int16_t _raw[3]; 				//raw input data
		std::vector<double>  _calib;//calibrated output data
		double  _conversion_factor;	//conversion to SI-units
		double  _offset[3];					//user-defined offset
		double 	_scale[3];					//user-defined scaling factor
		std::vector<double> _data;	//(raw) sensor data

	public:
    	Sensor3D(const uint8_t ID = 0): //constructor 1, initializes class variables
    	_ID(ID),
			// _raw{0,0,0},
		  _calib(3,0.0),  
			_conversion_factor(1.0f),
			_offset{0.0f,0.0f,0.0f},
			_scale{1.0f,1.0f,1.0f}
		{
			//do nothing
		}

		std::vector<double> calibrate(){ //calibrate raw data using appropriate conversion factor, offset and scale
			for(uint8_t k=0;k<3;k++){ //for now supposed that all data is 3 dimensional

				_calib[k] = (_data[k]*_conversion_factor + _offset[k])*_scale[k];
				
				//TEST:
				SENSOR3D_DEBUG_PRINT("raw data: "<<_data[k])
				SENSOR3D_DEBUG_PRINT("conversion_factor: "<<_conversion_factor)
				SENSOR3D_DEBUG_PRINT("offset: "<<_offset[k])
				SENSOR3D_DEBUG_PRINT("scale: "<<_scale[k])
				SENSOR3D_DEBUG_PRINT("multiplication: "<<(_data[k]*_conversion_factor + _offset[k])*_scale[k])
				SENSOR3D_DEBUG_PRINT("calibrated data: "<<_calib[k])
			}
			SENSOR3D_DEBUG_PRINT("calibrated data: "<<_calib[0]<<" , "<<_calib[1]<<" , "<<_calib[2])//output calibrated data to user
			return _calib; 
		}

		void setOffset(double _offset_x, double _offset_y, double _offset_z){ //assign sensor offset
			_offset[0] = _offset_x;
			_offset[1] = _offset_y;
			_offset[2] = _offset_z;

			SENSOR3D_DEBUG_PRINT("Setting offset: "<<_offset_x<<" , "<<_offset_y<<" , "<<_offset_z)
		}

		void setScale(double _scale_x, double _scale_y, double _scale_z){ //assign sensor scaling factors
			_scale[0] = _scale_x;
			_scale[1] = _scale_y;
			_scale[2] = _scale_z;

			SENSOR3D_DEBUG_PRINT("Setting scale: "<<_scale_x<<" , "<<_scale_y<<" , "<<_scale_z)
		}

		void setConversionFactor(double conversion_factor){  //assign conversion factors to convert factory units to SI-units (see datasheet)
			_conversion_factor = conversion_factor;

			SENSOR3D_DEBUG_PRINT("Setting conversion factor: "<<conversion_factor)
		}

		void writeData(std::vector<double> data){ //write (raw) data into the sensor attribute _data
			_data = data;
		}

		double getConversionFactor(){ //function which returns the used conversion factor, mainly used for testing purposes
			double conversion_factor = 0; //initialize as 0 to avoid old data being remembered in the variable
			conversion_factor = _conversion_factor;
			return conversion_factor;
		}
};

#endif //SENSOR3D
