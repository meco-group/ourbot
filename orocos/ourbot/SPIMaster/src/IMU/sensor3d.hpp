#ifndef SENSOR3D_HPP
#define SENSOR3D_HPP

#include <iostream>
#include <vector>
#include <stdint.h>

//This is a general Sensor3D. In the IMU Orocos component implement all specific aspects.

class Sensor3D
{
	private:
		const uint8_t _ID; //sensor ID
		// int16_t _raw[3]; //raw input data
		std::vector<double>  _calib;//calibrated output data
		double  _conversion_factor;//conversion to SI-units
		double  _offset[3];//user-defined offset
		double 	_scale[3];//user-defined scaling factor
		std::vector<double> _data;//

	public:
    	Sensor3D(const uint8_t ID = 0): //constructor 1
    	_ID(ID),
			// _raw{0,0,0},
		  _calib(3,0.0),  //how initialize at ones/zeros? Todo: this seems wrong, puts everything to zero?
			_conversion_factor(1.0f),
			_offset{0.0f,0.0f,0.0f},
			_scale{1.0f,1.0f,1.0f}
		{
			//do nothing
		}

		// Sensor3D(const uint8_t ID = 0, int16_t offset[3] = {0,}, float scale[3] = {1.0f,1.0f,1.0f}, float conversion_factor = 1): //constructor 2
  //   		_ID(ID),
		// 	_raw{0,0,0},
		// 	_calib{0.0f,0.0f,0.0f},
		// 	_conversion_factor(conversion_factor),
		// 	_offset(offset),
		// 	_scale(scale)
		// {
		// 	//do nothing
		// }

		std::vector<double> calibrate(){
			for(uint8_t k=0;k<3;k++){
				std::cout<<"raw data: "<<_data[k]<<std::endl;
				std::cout<<"conversion_factor: "<<_conversion_factor<<std::endl;
				std::cout<<"offset: "<<_offset[k]<<std::endl;
				std::cout<<"scale: "<<_scale[k]<<std::endl;
				_calib[k] = (_data[k]*_conversion_factor + _offset[k])*_scale[k];
				std::cout<<"multiplication: "<<(_data[k]*_conversion_factor + _offset[k])*_scale[k]<<std::endl;
				std::cout<<"calibrated data: "<<_calib[k]<<std::endl;
			}
			std::cout<<"calibrated data: "<<_calib[0]<<" , "<<_calib[1]<<" , "<<_calib[2]<<std::endl;
			return _calib; 
		}

		void setOffset(double _offset_x, double _offset_y, double _offset_z){
			_offset[0] = _offset_x;
			_offset[1] = _offset_y;
			_offset[2] = _offset_z;

			std::cout<<"Setting offset: "<<_offset_x<<" , "<<_offset_y<<" , "<<_offset_z<<std::endl;
		}

		void setScale(double _scale_x, double _scale_y, double _scale_z){
			_scale[0] = _scale_x;
			_scale[1] = _scale_y;
			_scale[2] = _scale_z;

			std::cout<<"Setting scale: "<<_scale_x<<" , "<<_scale_y<<" , "<<_scale_z<<std::endl;
		}

		void setConversionFactor(double conversion_factor){
			std::cout<<"Setting conversion factor: "<<conversion_factor<<std::endl;
			_conversion_factor = conversion_factor;
		}

		void writeData(std::vector<double> data){
			_data = data;
		}

		double getConversionFactor(){
			double conversion_factor = 0;
			conversion_factor = _conversion_factor;
			return conversion_factor;
		}
};

#endif //SENSOR3D
