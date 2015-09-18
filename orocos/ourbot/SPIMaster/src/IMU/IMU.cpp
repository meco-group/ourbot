#include "IMU.hpp"
//#include <rtt/Component.hpp>
#include <iostream>

IMU::IMU(std::string const& name) : 
	SPIDeviceInterface(name), //call constructor of mother class
	_cal_imu_transacc(3), _cal_imu_dorientation_3d(3), _cal_imu_orientation_3d(3), _cal_imu_dorientation(0.), _cal_imu_orientation(0.),_cal_imu_temperature(3),
	_raw_imu_acc(3), _raw_imu_gyr(3), _raw_imu_mag(3), _raw_imu_tmp(3){

  //Add ports
  ports()->addPort("cal_imu_transacc_port",         _cal_imu_transacc_port).doc       ("Translational acceleration from IMU");
  ports()->addPort("cal_imu_dorientation_3d_port",  _cal_imu_dorientation_3d_port).doc("3D rotational velocity from IMU");
  ports()->addPort("cal_imu_orientation_3d_port",   _cal_imu_orientation_3d_port).doc ("3D orientation from IMU");
  ports()->addPort("cal_imu_dorientation_port",     _cal_imu_dorientation_port).doc   ("1D rotational velocity from IMU");
  ports()->addPort("cal_imu_orientation_port",      _cal_imu_orientation_port).doc    ("1D orientation from IMU");
  ports()->addPort("cal_imu_temperature_port",      _cal_imu_temperature_port).doc    ("temperature from IMU");

  ports()->addPort("raw_imu_acc_port",              _raw_imu_acc_port).doc("Raw accelerometer data from IMU");
  ports()->addPort("raw_imu_gyr_port",              _raw_imu_gyr_port).doc("Raw gyroscope data from IMU");
  ports()->addPort("raw_imu_mag_port",              _raw_imu_mag_port).doc("Raw magnetometer data from IMU");
  ports()->addPort("raw_imu_tmp_port",              _raw_imu_tmp_port).doc("Raw temperature data from IMU");

#ifndef IMU_TESTFLAG
  //Add properties
  addProperty("acc_offset", _acc_offset).doc("User defined offset of accelerometer");
  addProperty("mag_offset", _mag_offset).doc("User defined offset of magnetometer");
  addProperty("gyr_offset", _gyr_offset).doc("User defined offset of gyroscope");
  addProperty("tmp_offset", _tmp_offset).doc("User defined offset of temperature sensor");
  addProperty("acc_scale",  _acc_scale).doc("User defined  scaling factor for accelerometer");
  addProperty("mag_scale",  _mag_scale).doc("User defined  scaling factor for magnetometer");
  addProperty("gyr_scale",  _gyr_scale).doc("User defined  scaling factor for gyroscope");
  addProperty("tmp_scale",  _tmp_scale).doc("User defined  scaling factor for temperature sensor");
  
  addProperty("acc_range",  _acc_range).doc("Range for accelerometer");
  addProperty("mag_range",  _mag_range).doc("Range for magnetometer");
  addProperty("gyr_range",  _gyr_range).doc("Range for gyroscope");
  addProperty("mag_range",  _pin_accmag).doc("Chip select pin of accelerometer and magnetometer");
  addProperty("gyr_range",  _pin_gyr).doc("Chip select pin of gyroscope");

#else //if in Test-mode
  _acc_offset[0] = 0; _acc_offset[1] = 0; _acc_offset[2] = 0;
  _gyr_offset[0] = 0; _gyr_offset[1] = 0; _gyr_offset[2] = 0;
  _mag_offset[0] = 0; _mag_offset[1] = 0; _mag_offset[2] = 0;
  _tmp_offset = 21; //starting point, 0 value of sensor (see adafruit code)
  _acc_scale[0] = 1; _acc_scale[1] = 1; _acc_scale[2] = 1;
  _gyr_scale[0] = 1; _gyr_scale[1] = 1; _gyr_scale[2] = 1;
  _mag_scale[0] = 1; _mag_scale[1] = 1; _mag_scale[2] = 1;
  _tmp_scale = 1;
  // _acc_scale = [1,1,1];
  // _mag_scale = [1,1,1];
  // _gyr_scale = [1,1,1];

  _acc_range = 2;
  _mag_range = 2;
  _gyr_range = 245;
  _pin_accmag = 15;
  _pin_gyr = 13;

#endif //IMU_TESTFLAG

  std::cout << "IMU constructed!" <<std::endl;
}

bool IMU::configureHook(){

  std::cout << "Entering IMU configureHook!" <<std::endl;

  //Todo: load properties here, from .cfg file

  std::cout << "IMU configured!" <<std::endl;
  return true;
}

bool IMU::startHook(){
	
  std::cout << "Entering IMU startHook!" <<std::endl;

  std::cout << "Entering SPIDeviceInterface startHook!" <<std::endl;

	SPIDeviceInterface::startHook(); //To assign fd and SPI properties

	std::cout<< "Continuing IMU configureHook"<<std::endl;

	//TEST
	//Test if sending an SPI message is possible
	// transfer();

	init(); //Set sensor mode, declare range, make Sensor3D instances

	//Todo: connect ports to reporterNC for now. 

	// //Check if all ports are connected
	// bool portCheck = true;
	
 //  //Check output ports
 //  if (!_cal_imu_transacc_port.connected()){
 //    RTT::log(RTT::Error) << "cal_imu_transacc_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }
 //  if (!_cal_imu_dorientation_3d_port.connected()){
 //    RTT::log(RTT::Error) << "cal_imu_dorientation_3d_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }
 //  if (!_cal_imu_orientation_3d_port.connected()){
 //    RTT::log(RTT::Error) << "cal_imu_orientation_3d_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }
 //  if (!_cal_imu_dorientation_port.connected()){
 //    RTT::log(RTT::Error) << "cal_imu_dorientation_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }
 //  if (!_cal_imu_orientation_port.connected()){
 //    RTT::log(RTT::Error) << "cal_imu_orientation_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }

 //  //Check input ports
 //  if (!_raw_imu_acc_port.connected()){
 //    RTT::log(RTT::Error) << "raw_imu_acc_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }
 //  if (!_raw_imu_gyr_port.connected()){
 //    RTT::log(RTT::Error) << "raw_imu_gyr_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }
	// if (!_raw_imu_mag_port.connected()){
 //    RTT::log(RTT::Error) << "raw_imu_mag_port not connected!" <<RTT::endlog();
 //    portCheck = false;
 //  }

 //  if (!portCheck){
 //    RTT::log(RTT::Error) << "Error, some ports were not connected!" <<RTT::endlog();
 //    return false;
 //  }

	double test4 = _acc.getConversionFactor();
  std::cout<<"got conversion factor in IMU starthook: "<<test4<<std::endl;

  //Check sensor connection
  if (!isConnected()){
    RTT::log(RTT::Error) << "Error, the sensor is not connected, check wiring!" <<RTT::endlog();
    return false;
  }

	//else: everything went good
  std::cout << "IMU started!" <<std::endl;
  return true;
}

void IMU::updateHook(){ //Is executed by the IO component actually
  
  double test3 = _acc.getConversionFactor();
  std::cout<<"got conversion factor in IMU updatehook point 1: "<<test3<<std::endl;

  SPIDeviceInterface::updateHook();

  //TEMPORARY: reset conversion factors
  _acc.setConversionFactor(_acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000.0);
  _gyr.setConversionFactor(_gyr_dps_digit);
  _mag.setConversionFactor(_mag_mgauss_lsb/1000.0);
  _tmp.setConversionFactor(1.0/LSM9DS0_TEMP_LSB_DEGREE_CELSIUS);
  _tmp.setOffset(_tmp_offset,0,0);

  double test0 = _acc.getConversionFactor();
  std::cout<<"got conversion factor in IMU updatehook point 2: "<<test0<<std::endl;

  //Get measurements
  updateMeasurements();

  //Todo: Fatal if at some time no data on port?	
  //Todo: This is not possible, since these are all input ports?
  // if (_raw_imu_acc_port.read(_raw_imu_acc) == RTT::NoData) {RTT::log(RTT::Error) << "No data on raw_imu_acc_port!" <<RTT::endlog(); error();}
  // if (_raw_imu_gyr_port.read(_raw_imu_gyr) == RTT::NoData) {RTT::log(RTT::Error) << "No data on raw_imu_gyr_port!" <<RTT::endlog(); error();}
  // if (_raw_imu_mag_port.read(_raw_imu_mag) == RTT::NoData) {RTT::log(RTT::Error) << "No data on raw_imu_mag_port!" <<RTT::endlog(); error();}
  // if (_raw_imu_tmp_port.read(_raw_imu_tmp) == RTT::NoData) {RTT::log(RTT::Error) << "No data on raw_imu_tmp_port!" <<RTT::endlog(); error();}

  //Todo: only read when new data? No, this updateHook is executed by the external I/O component
	// std::vector<double> acc(3);
	// std::vector<double> gyr(3);
	// std::vector<double> mag(3);
	// if(_raw_imu_acc_port.read(acc) == RTT::NewData) {_raw_imu_acc = acc}
	// if(_raw_imu_gyr_port.read(gyr) == RTT::NewData) {_raw_imu_gyr = gyr}
	// if(_raw_imu_mag_port.read(mag) == RTT::NewData) {_raw_imu_mag = mag}
							
	std::cout << "IMU data updated!" <<std::endl;
}

void IMU::stopHook() {

  //Clean up GPIO's:
  std::cout << "Cleaning up the GPIO's" <<std::endl;
  cleanupGPIO(_pin_accmag);
  cleanupGPIO(_pin_gyr);

  std::cout << "IMU executes stopping!" <<std::endl;
}

void IMU::cleanupHook() {
  std::cout << "IMU cleaning up!" <<std::endl;
}

//Define methods:
//--------------

uint8_t IMU::pin2GPIO(uint8_t pin){ 
  //Odroid pins are internally connected to GPIO's which are located on GPIO chips. 
  //To make the GPIO accessible you need the GPIO number. 
  //The user only thas the pin number. This function makes the link between the two.
  uint8_t GPIO = 0; //need to initialize, otherwise its value is the same for different function calls
  std::cout<< "The pin number is: " << (int)pin << std::endl;
  switch(pin){ 
    case 13:      //CSG
      GPIO = 21; 
      std::cout<< "The GPIO number is: " << (int)GPIO << std::endl;
      break; //otherwise default is always selected afterwards
    case 15:			//CSXM
      GPIO = 18;
      std::cout<< "The GPIO number is: " << (int)GPIO << std::endl;
      break; //otherwise default is always selected afterwards
    // case 5:
    // 	GPIO = 174; //Vin of IMU -temporary-
    // 	std::cout<< "The GPIO number is: " << (int)GPIO << std::endl;
    // 	break;
    // case 24:
    // 	GPIO = 25;  //OE of level shifter -temporary-
    // 	std::cout<< "The GPIO number is: " << (int)GPIO << std::endl;
    //   break; //otherwise default is always selected afterwards
    default:
      RTT::log(RTT::Error) << "Invalid pin selected, normally only pins 13 and 15 should be connected" << RTT::endlog();
      //Todo: output some special GPIO then? Right now GPIO = 0 is output. An error seems enough?
  }
  return GPIO;
}

void IMU::init(){
  
  //Convert pin to GPIO number and activate GPIO's
  _cs_accmag  = pin2GPIO(_pin_accmag);
  _cs_gyr     = pin2GPIO(_pin_gyr);

  std::cout<< "Chip select of accmag is on GPIO: " << (int)_cs_accmag << std::endl;
  std::cout<< "Chip select of gyro is on GPIO: " << (int)_cs_gyr << std::endl;

  //Initialize GPIO as output pin with value 1 (high)
  initGPIO(_cs_accmag);
  initGPIO(_cs_gyr);

  //Make sensor settings
  std::cout<<"file descriptor before first write: " <<(int)_fd<<std::endl;
  std::cout<<"address of cs_accmag before first write: " <<&_cs_accmag<<std::endl;
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG1_XM, 0x67);        // 100hz XYZ
  std::cout<<"chip select accelerometer after first write: " <<(int)_cs_accmag<<std::endl;
	std::cout<<"address of cs_accmag after first write: " <<&_cs_accmag<<std::endl;
  std::cout<<"file descriptor after first write: " <<(int)_fd<<std::endl;
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000);  //temp sensor enabled, high res, 50Hz mag
  // enable mag continuous
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);  //normal mode
  // enable gyro continuous
  writeByte(_cs_gyr, LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F);            // turn on XYZ

  //Assign the conversion factors and the range registers, corresponding to the selected ranges of the sensor
  std::cout<<"acc range: "<<_acc_range<<std::endl;
  std::cout<<"mag range: "<<_mag_range<<std::endl;
  std::cout<<"gyr_range: "<<_gyr_range<<std::endl;

  switch(_acc_range)
  {
    case 2:
      _acc_mg_lsb = LSM9DS0_ACCEL_MG_LSB_2G;
      _acc_range_register = LSM9DS0_ACCELRANGE_2G;
      break;
    case 4:
      _acc_mg_lsb = LSM9DS0_ACCEL_MG_LSB_4G;
      _acc_range_register = LSM9DS0_ACCELRANGE_4G;
      break;
    case 6:
      _acc_mg_lsb = LSM9DS0_ACCEL_MG_LSB_6G;
      _acc_range_register = LSM9DS0_ACCELRANGE_6G;
      break;
    case 8:
      _acc_mg_lsb = LSM9DS0_ACCEL_MG_LSB_8G;
      _acc_range_register = LSM9DS0_ACCELRANGE_8G;
      break;
    case 16:
      _acc_mg_lsb = LSM9DS0_ACCEL_MG_LSB_16G;
      _acc_range_register = LSM9DS0_ACCELRANGE_16G;
      break;
    default:
      RTT::log(RTT::Error) << "Invalid accelerometer range selected, use 2, 4, 6, 8 or 16, you used: "<<(int)_acc_range<< RTT::endlog();
  }

    switch(_mag_range)
  {
    case 2:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_2GAUSS;
      _mag_range_register = LSM9DS0_MAGGAIN_2GAUSS;
      break;
    case 4:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_4GAUSS;
      _mag_range_register = LSM9DS0_MAGGAIN_4GAUSS;
      break;
    case 8:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_8GAUSS;
      _mag_range_register = LSM9DS0_MAGGAIN_8GAUSS;
      break;
    case 12:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_12GAUSS;
      _mag_range_register = LSM9DS0_MAGGAIN_12GAUSS;
      break;
    default:
      RTT::log(RTT::Error) << "Invalid magnetometer range selected, use 2, 4, 8 or 12, you used: "<<(int)_mag_range<< RTT::endlog();
  }

    switch(_gyr_range)
  {
    case 245:
      _gyr_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
      _gyr_range_register = LSM9DS0_GYROSCALE_245DPS;
      break;
    case 500:
      _gyr_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_500DPS;
      _gyr_range_register = LSM9DS0_GYROSCALE_500DPS;
      break;
    case 2000:
      _gyr_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_2000DPS;
      _gyr_range_register = LSM9DS0_GYROSCALE_2000DPS;
      break;
    default:
      RTT::log(RTT::Error) << "Invalid gyroscope range selected, use 245, 500 or 2000, you used: "<<(int)_gyr_range<< RTT::endlog();
  }

  std::cout<<"_acc_mg_lsb: "     <<_acc_mg_lsb<<std::endl;
  std::cout<<"_mag_mgauss_lsb: " <<_mag_mgauss_lsb<<std::endl;
  std::cout<<"_gyr_dps_digit: "  <<_gyr_dps_digit<<std::endl;

  //Set default ranges for the various sensors  
  setupAccel(_acc_range_register);
  setupMag  (_mag_range_register);
  setupGyro (_gyr_range_register);

  std::cout<<"_acc_range_register: "     <<_acc_range_register<<std::endl;
  std::cout<<"_mag_range_register: " <<_mag_range_register<<std::endl;
  std::cout<<"_gyr_range_register: "  <<_gyr_range_register<<std::endl;

  //Make Sensor3D instances
  _id = 0; //Todo: or not necessary?
  Sensor3D _acc(_id+1);//, _cs_accmag, _acc_offset, _acc_scale, _acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000);
  _acc.setOffset(_acc_offset[0],_acc_offset[1], _acc_offset[2]);
  _acc.setScale(_acc_scale[0],_acc_scale[1], _acc_scale[2]);
  _acc.setConversionFactor(_acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000.0);
  std::cout<<"conversion factor accelerometer: "<<_acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000.0<<std::endl;
  Sensor3D _gyr(_id+2);//, _cs_gyr,    _gyr_offset, _gyr_scale, _gyr_dps_digit/1000);
  _gyr.setOffset(_gyr_offset[0],_gyr_offset[1], _gyr_offset[2]);
  _gyr.setScale(_gyr_scale[0],_gyr_scale[1], _gyr_scale[2]);
  _gyr.setConversionFactor(_gyr_dps_digit);
  std::cout<<"conversion factor gyroscope: "<<_gyr_dps_digit<<std::endl;
  Sensor3D _mag(_id+3);//, _cs_accmag, _mag_offset, _mag_scale, _mag_mgauss_lsb/1000);
  _mag.setOffset(_mag_offset[0],_mag_offset[1], _mag_offset[2]);
  _mag.setScale(_mag_scale[0],_mag_scale[1], _mag_scale[2]);
  _mag.setConversionFactor(_mag_mgauss_lsb/1000.0);
  std::cout<<"conversion factor magnetometer: "<<_mag_mgauss_lsb/1000.0<<std::endl;
  Sensor3D _tmp(_id+4);//, _cs_accmag, _tmp_offset, _tmp_scale, _tmp_conv);
  _tmp.setOffset(_tmp_offset,0,0);
  _tmp.setScale(_tmp_scale,0,0);
  _tmp.setConversionFactor(1.0/LSM9DS0_TEMP_LSB_DEGREE_CELSIUS);
  std::cout<<"conversion factor temperature: "<<1.0/LSM9DS0_TEMP_LSB_DEGREE_CELSIUS<<std::endl;

  double test = _acc.getConversionFactor();
  std::cout<<"got conversion factor: "<<test<<std::endl;
} //init()

bool  IMU::isConnected(){
  //Check standard ID of sensor (see STM LSM9DS0 datasheet)

  std::cout<<"in IMU::isConnected(), chip select accelerometer: "<<(int)_cs_accmag<<std::endl;
  std::cout<<"in IMU::isConnected(), register: "<<(int)LSM9DS0_REGISTER_WHO_AM_I_XM<<std::endl;
  uint8_t id = readByte(_cs_accmag, LSM9DS0_REGISTER_WHO_AM_I_XM);
  std::cout<<"Accmag ID is: "<<(int)id<<std::endl;
  if (id != LSM9DS0_XM_ID)
    return false;

  std::cout<<"in IMU::isConnected(), chip select gyroscope: "<<(int)_cs_gyr<<std::endl;
  std::cout<<"in IMU::isConnected(), register: "<<(int)LSM9DS0_REGISTER_WHO_AM_I_G<<std::endl;
  id = readByte(_cs_gyr, LSM9DS0_REGISTER_WHO_AM_I_G);
  std::cout<<"Gyro ID is: "<<(int)id<<std::endl;
  if (id != LSM9DS0_G_ID) 
    return false;

  else
  	std::cout<<"Both sensors are connected!"<<std::endl;
    return true;
}

void IMU::setupAccel (uint8_t range)
{
  uint8_t reg = readByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG2_XM); //read out the current value
  reg &= ~(0b00111000);  //Make a number in which the bits that we will change = 1 and invert this number then AND the current value of reg with this number --> make all the numbers we will/want to change = 1
  reg |= range;          //OR the new value of reg with the range --> you OR with the 000 --> gets the value of 'range'
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG2_XM, reg); //write the updated value to the register = set-up of the hardware
}

void IMU::setupMag (uint8_t range)
{
  uint8_t reg = readByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG6_XM);
  reg &= ~(0b01100000);
  reg |= range;
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG6_XM, reg);
}

void IMU::setupGyro (uint8_t range)
{
  uint8_t reg = readByte(_cs_gyr, LSM9DS0_REGISTER_CTRL_REG4_G);
  reg &= ~(0b00110000);
  reg |= range;
  writeByte(_cs_gyr, LSM9DS0_REGISTER_CTRL_REG4_G, reg);
}

void IMU::updateMeasurements()
{

  double test1 = _acc.getConversionFactor();
  std::cout<<"got conversion factor in updateMeasurements: "<<test1<<std::endl;

  //Read all the sensors
  readAccel();
  readMag();
  readGyro();
  readTemp();

  std::cout<<"Ready to calibrate IMU data!"<<std::endl;

  double test2 = _acc.getConversionFactor();
  std::cout<<"got conversion factor after read: "<<test2<<std::endl;

  //Calibrate: convert raw measurements to SI-units, with user-defined offset and scaling factor and selected conversion factor
  _cal_imu_transacc         = _acc.calibrate(); //contains accelerations along all axes
  _cal_imu_orientation_3d   = _mag.calibrate(); //contains roll pitch yaw angles
  _cal_imu_dorientation_3d  = _gyr.calibrate(); //contains angular velocity along all axes
  _cal_imu_dorientation     = _cal_imu_dorientation_3d[2];//select angular velocity around z-axis
  _cal_imu_orientation      = _cal_imu_orientation_3d[2]; //select yaw angle
  _cal_imu_temperature      = _tmp.calibrate(); //contains temperature

  std::cout<<"_cal_imu_transacc: "       <<_cal_imu_transacc[0]<<" , "<<_cal_imu_transacc[1]<<" , "<<_cal_imu_transacc[2]<<" , "<<std::endl;
  std::cout<<"_cal_imu_orientation_3d: " <<_cal_imu_orientation_3d[0]<<" , "<<_cal_imu_orientation_3d[1]<<" , "<<_cal_imu_orientation_3d[2]<<" , "<<std::endl;
  std::cout<<"_cal_imu_dorientation_3d: "<<_cal_imu_dorientation_3d[0]<<" , "<<_cal_imu_dorientation_3d[1]<<" , "<<_cal_imu_dorientation_3d[2]<<" , "<<std::endl;
  std::cout<<"_cal_imu_dorientation z-axis: "   <<_cal_imu_dorientation<<std::endl;
  std::cout<<"_cal_imu_orientation z-axis: "    <<_cal_imu_orientation<<std::endl;
  std::cout<<"_cal_imu_temperature: "    <<_cal_imu_temperature[0]<<std::endl;

  //Todo: compensate for roll/pitch/yaw in angular velocity measurements? No, better translate to world frame when using these measurements

  //Write measurements
  _cal_imu_transacc_port.write        (_cal_imu_transacc);                
  _cal_imu_dorientation_3d_port.write (_cal_imu_dorientation_3d);
  _cal_imu_orientation_3d_port.write  (_cal_imu_orientation_3d);
  _cal_imu_dorientation_port.write    (_cal_imu_dorientation);    
  _cal_imu_orientation_port.write     (_cal_imu_orientation);               
}

void IMU::readAccel() {

  uint8_t length = 2 * 3; //read 3 16 bits (= 2 byte) numbers, so read out 6 bytes
  uint8_t reg = LSM9DS0_REGISTER_OUT_X_L_A;
  uint8_t buffer[7]; //make a buffer to hold 6 bytes, being what you will read
  //Todo: test if this works, maybe you always need to read/write a multiple of 4 bytes --> no
  //Todo: maybe you also need to do +1 to account for the address/register, which you also write --> yes

  std::cout<<"In readAccel"<<std::endl;

  readBytes(_cs_accmag, reg, length, buffer);

  uint8_t xlo = buffer[1]; //Todo: test if this order is correct, maybe buffer[0] contains invalid data and real values only start from buffer[1]
  int16_t xhi = buffer[2];
  uint8_t ylo = buffer[3];
  int16_t yhi = buffer[4];
  uint8_t zlo = buffer[5];
  int16_t zhi = buffer[6];
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  std::cout<<"Reading raw accelerometer data: "<<std::endl;
  std::cout<<"acc_x = "<<(int)xhi<<std::endl;
  std::cout<<"acc_y = "<<(int)yhi<<std::endl;
  std::cout<<"acc_z = "<<(int)zhi<<std::endl;

  std::vector<double> data(3);
  data[0] = xhi;
  data[1] = yhi;
  data[2] = zhi;
  _acc.writeData(data); 
  _raw_imu_acc_port.write(data); 
}

void IMU::readMag() {
  
  uint8_t length = 2 * 3; //read 3 2 byte numbers
  uint8_t reg = LSM9DS0_REGISTER_OUT_X_L_M;
  uint8_t buffer[7]; //make a buffer to hold 6 bytes (= 3*16bits), being what you will read

  std::cout<<"In readMag"<<std::endl;

  readBytes(_cs_accmag, reg, length, buffer);

  uint8_t xlo = buffer[1];
  int16_t xhi = buffer[2];
  uint8_t ylo = buffer[3];
  int16_t yhi = buffer[4];
  uint8_t zlo = buffer[5];
  int16_t zhi = buffer[6];
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  std::cout<<"Reading raw magnetometer data: "<<std::endl;
  std::cout<<"mag_x = "<<(int)xhi<<std::endl;
  std::cout<<"mag_y = "<<(int)yhi<<std::endl;
  std::cout<<"mag_z = "<<(int)zhi<<std::endl;

  //Todo: manipulate data to get roll pitch yaw angles
  //https://www.pololu.com/file/download/LSM303DLH-compass-app-note.pdf?file_id=0J434
  //Accelerometer calibration parameters: necessary?
  // xhi = arcsin(acc_yhi/cos(mag_yhi))
  // yhi = arcsin(-acc_xhi)
  // zhi = ...

  std::vector<double> data(3);
  data[0] = xhi;
  data[1] = yhi;
  data[2] = zhi;
  _mag.writeData(data);
  _raw_imu_mag_port.write(data); 

  // byte buffer[6];
  // readBuffer(XMTYPE, 
  //      0x80 | LSM9DS0_REGISTER_OUT_X_L_M, 
  //      6, buffer);
  
  // uint8_t xlo = buffer[0];
  // int16_t xhi = buffer[1];
  // uint8_t ylo = buffer[2];
  // int16_t yhi = buffer[3];
  // uint8_t zlo = buffer[4];
  // int16_t zhi = buffer[5];
  
  // // Shift values to create properly formed integer (low byte first)
  // xhi <<= 8; xhi |= xlo;
  // yhi <<= 8; yhi |= ylo;
  // zhi <<= 8; zhi |= zlo;
  // <std::vector<double> > data[3];
  // data[0] = xhi;
  // data[1] = yhi;
  // data[2] = zhi;
  // _mag.writeData(data); 
  // // magData.x = xhi;
  // // magData.y = yhi;
  // // magData.z = zhi;
}

void IMU::readGyro() {
  
  uint8_t length = 2 * 3; //read 3 2 byte numbers
  uint8_t reg = LSM9DS0_REGISTER_OUT_X_L_G;
  uint8_t buffer[7]; //make a buffer to hold 6 bytes (= 3*16bits), being what you will read

  std::cout<<"In readGyro"<<std::endl;

  readBytes(_cs_gyr, reg, length, buffer);

  uint8_t xlo = buffer[1];
  int16_t xhi = buffer[2];
  uint8_t ylo = buffer[3];
  int16_t yhi = buffer[4];
  uint8_t zlo = buffer[5];
  int16_t zhi = buffer[6];
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

	std::cout<<"Reading raw gyroscope data: "<<std::endl;
  std::cout<<"gyr_x = "<<(int)xhi<<std::endl;
  std::cout<<"gyr_y = "<<(int)yhi<<std::endl;
  std::cout<<"gyr_z = "<<(int)zhi<<std::endl;

  std::vector<double> data(3);
  data[0] = xhi;
  data[1] = yhi;
  data[2] = zhi;
  _gyr.writeData(data);
  _raw_imu_gyr_port.write(data); 


  // byte buffer[6];
  // readBuffer(GYROTYPE, 
  //      0x80 | LSM9DS0_REGISTER_OUT_X_L_G, 
  //      6, buffer);
  
  // uint8_t xlo = buffer[0];
  // int16_t xhi = buffer[1];
  // uint8_t ylo = buffer[2];
  // int16_t yhi = buffer[3];
  // uint8_t zlo = buffer[4];
  // int16_t zhi = buffer[5];
  
  // // Shift values to create properly formed integer (low byte first)
  // xhi <<= 8; xhi |= xlo;
  // yhi <<= 8; yhi |= ylo;
  // zhi <<= 8; zhi |= zlo;
  // <std::vector<double> > data[3];
  // data[0] = xhi;
  // data[1] = yhi;
  // data[2] = zhi;
  // _gyr.writeData(data); 
  // // gyroData.x = xhi;
  // // gyroData.y = yhi;
  // // gyroData.z = zhi;
}

void IMU::readTemp() {
  
  uint8_t length = 2 * 1; //read 1 2 byte number
  uint8_t reg = LSM9DS0_REGISTER_TEMP_OUT_L_XM;
  uint8_t buffer[3]; //make a buffer to hold 2 bytes (= 1*16bits), being what you will read

  //Todo: or is this not possible and do you need to read 4 bytes?

  std::cout<<"In readTemp"<<std::endl;

  readBytes(_cs_accmag, reg, length, buffer);

  std::cout<<"reading temperature0: "<<(int)buffer[0]<<std::endl;
  std::cout<<"reading temperature1: "<<(int)buffer[1]<<std::endl;
  std::cout<<"reading temperature2: "<<(int)buffer[2]<<std::endl;

  uint8_t xlo = buffer[1];
  int16_t xhi = buffer[2];  
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;

  std::cout<<"Reading raw temperature data: "<<std::endl;
  std::cout<<"tmp = "<<(int)xhi<<std::endl;

  std::vector<double> data(1);
  data[0] = xhi;
  _tmp.writeData(data);
  _raw_imu_tmp_port.write(data); 

  // byte buffer[2];
  // readBuffer(XMTYPE, 
  //      0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM, 
  //      2, buffer);
  // uint8_t xlo = buffer[0];
  // int16_t xhi = buffer[1];

  // xhi <<= 8; xhi |= xlo;
  // <std::vector<double> > data[1];
  // data[0] = xhi;
  // _tmp.writeData(data); 
  // Shift values to create properly formed integer (low byte first)
  // temperature = xhi;
}

//Get sensor data
std::vector<double> IMU::getImuTransAcc()      { return _cal_imu_transacc; }
std::vector<double> IMU::getImuDOrientation3D(){ return _cal_imu_dorientation_3d; }
std::vector<double> IMU::getImuOrientation3D() { return _cal_imu_orientation_3d; }
double              IMU::getImuDOrientation()  { return _cal_imu_dorientation; }
double              IMU::getImuOrientation()   { return _cal_imu_orientation; }

//Set sensor data
//Todo: only useful for testing?
//Todo: originally there was: std::vector<double>, const& cal_imu_transacc? pass by constant reference
void IMU::setImuTransAcc      (std::vector<double> cal_imu_transacc)        { cal_imu_transacc        = _cal_imu_transacc; }
void IMU::setImuDOrientation3D(std::vector<double> cal_imu_dorientation_3d) { cal_imu_dorientation_3d = _cal_imu_dorientation_3d; }
void IMU::setImuOrientation3D (std::vector<double> cal_imu_orientation_3d)  { cal_imu_orientation_3d  = _cal_imu_orientation_3d; }
void IMU::setImuDOrientation  (double              cal_imu_dorientation)    { cal_imu_dorientation    = _cal_imu_dorientation; }
void IMU::setImuOrientation   (double              cal_imu_orientation)     { cal_imu_orientation     = _cal_imu_orientation; }


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
