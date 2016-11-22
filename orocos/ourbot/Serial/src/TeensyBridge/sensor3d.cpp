#include "sensor3d.hpp"

Sensor3D::Sensor3D():
	_calib(3,0.0),  
	_raw(3,0.0),
	_offset(3,0.0),
	_scale(3,1.0)
{
	//do nothing
}

std::vector<double> Sensor3D::calibrate(){ //calibrate raw data using appropriate conversion factor, offset and scale
	for(uint8_t k=0;k<3;k++){ //for now supposed that all data is 3 dimensional

		_calib[k] = (_raw[k] + _offset[k])*_scale[k];
		
		//TEST:
		/*SENSOR3D_DEBUG_PRINT("raw data: "<<_data[k])
		SENSOR3D_DEBUG_PRINT("conversion_factor: "<<_conversion_factor)
		SENSOR3D_DEBUG_PRINT("offset: "<<_offset[k])
		SENSOR3D_DEBUG_PRINT("scale: "<<_scale[k])
		SENSOR3D_DEBUG_PRINT("multiplication: "<<(_data[k]*_conversion_factor + _offset[k])*_scale[k])
		SENSOR3D_DEBUG_PRINT("calibrated data: "<<_calib[k])*/
	}
	
	SENSOR3D_DEBUG_PRINT("original data: "<<_raw[0]<<" , "<<_raw[1]<<" , "<<_raw[2])//output calibrated data to user
	SENSOR3D_DEBUG_PRINT("calibrated data: "<<_calib[0]<<" , "<<_calib[1]<<" , "<<_calib[2])//output calibrated data to user
	return _calib; 
}

std::vector<double> Sensor3D::calibrate(std::vector<double> raw){
	_raw = raw;
	return calibrate();
}

void Sensor3D::setOffset(double offset_x, double offset_y, double offset_z){ //assign sensor offset
	_offset[0] = offset_x;
	_offset[1] = offset_y;
	_offset[2] = offset_z;

	SENSOR3D_DEBUG_PRINT("Setting offset: "<<offset_x<<" , "<<offset_y<<" , "<<offset_z)
}

void Sensor3D::setScale(double scale_x, double scale_y, double scale_z){ //assign sensor scaling factors
	_scale[0] = scale_x;
	_scale[1] = scale_y;
	_scale[2] = scale_z;

	SENSOR3D_DEBUG_PRINT("Setting scale: "<<scale_x<<" , "<<scale_y<<" , "<<scale_z)
}

std::vector<double> Sensor3D::getCalib()
{
	return _calib;
}
		
std::vector<double> Sensor3D::getRaw()
{
	return _raw;
}
