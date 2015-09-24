#include "IMU.hpp"
#include <iostream>

IMU::IMU(std::string const& name) :
	SPIDeviceInterface(name), //call constructor of mother class
	_cal_imu_transacc(3), _cal_imu_dorientation_3d(3), _cal_imu_orientation_3d(3), _cal_imu_dorientation(0.), _cal_imu_orientation(0.),_cal_imu_temperature(3),
	_raw_imu_acc(3), _raw_imu_gyr(3), _raw_imu_mag(3), _raw_imu_tmp(3),_acc_offset(3,0.0), _gyr_offset(3,0.0), _mag_offset(3,0.0), _acc_scale(3,1.0), _gyr_scale(3,1.0), _mag_scale(3,1.0){

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
  addProperty("accmag_pin", _pin_accmag).doc("Chip select pin of accelerometer and magnetometer");
  addProperty("gyr_pin",    _pin_gyr).doc("Chip select pin of gyroscope");

  addProperty("imu_name",  _imu_name).doc("Name of the IMU");

#else //if in Test-mode
  // _acc_offset[0] = 0; _acc_offset[1] = 0; _acc_offset[2] = 0; //based on calibration by inversion method
  _acc_offset[0] = 0.955348; _acc_offset[1] = 0.040480; _acc_offset[2] = 0.190965; //based on calibration by inversion method 
  // _acc_offset[0] = 0.984377; _acc_offset[1] = -0.11990; _acc_offset[2] = 0.325624; //based on calibration by inversion method   right
  _gyr_offset[0] = 0; _gyr_offset[1] = 0; _gyr_offset[2] = 0;
  _mag_offset[0] = 0; _mag_offset[1] = 0; _mag_offset[2] = 0;
  _tmp_offset = 21; //starting point, 0 value of sensor (see adafruit code)
  // _acc_scale[0] = 1; _acc_scale[1] = 1; _acc_scale[2] = 1; //based on calibration by inversion method
  _acc_scale[0] = 1.003799; _acc_scale[1] = 1.002724; _acc_scale[2] = 1.015012; //based on calibration by inversion method
  // _acc_scale[0] = 1.008924; _acc_scale[1] = 1.038506; _acc_scale[2] = 1.037258; //based on calibration by inversion method  right
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

  _imu_name = "imu_left";

#endif //IMU_TESTFLAG
  IMU_DEBUG_PRINT("IMU constructed!")
}

bool IMU::configureHook(){

  IMU_DEBUG_PRINT("Entering IMU configureHook!")
  IMU_DEBUG_PRINT("IMU configured!")

  //Show example data sample to output ports to make data flow real-time
  std::vector<double> example(3, 0.0);
  //set 3D ports
  _raw_imu_acc_port.setDataSample(example);
  _raw_imu_gyr_port.setDataSample(example);
  _raw_imu_mag_port.setDataSample(example);
  _raw_imu_tmp_port.setDataSample(example);
  _cal_imu_transacc_port.setDataSample(example);
  _cal_imu_dorientation_3d_port.setDataSample(example);
  _cal_imu_orientation_3d_port.setDataSample(example);

  //set 1D ports
  _cal_imu_dorientation_port.setDataSample(example[0]);
  _cal_imu_orientation_port.setDataSample(example[0]);
  _cal_imu_temperature_port.setDataSample(example[0]);

  //TEST: to test data acquisition
  // return this->setPeriod(1);
  return true;
}

bool IMU::startHook(){
	IMU_DEBUG_PRINT("Entering IMU startHook!")
	IMU_DEBUG_PRINT("Entering SPIDeviceInterface startHook!")

	SPIDeviceInterface::startHook(); //To assign fd and SPI properties
  IMU_DEBUG_PRINT("Continuing IMU configureHook")

	//TEST
	//Test if sending an SPI message is possible
	// transfer();

	init(); //Set sensor mode, declare range, make Sensor3D instances

 //TEST:
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

	// double test4 = _acc.getConversionFactor();
	// IMU_DEBUG_PRINT("got conversion factor in IMU starthook: "<<test4)

  //Check sensor connection
  if (!isConnected()){
    RTT::log(RTT::Error) << "Error, "<<_imu_name<<" is not connected, check wiring!" <<RTT::endlog();
    return false;
  }

	//else: everything went good
	IMU_DEBUG_PRINT("IMU started!")
  return true;
}

void IMU::updateHook(){ //Is executed by the IO component actually

  // double test3 = _acc.getConversionFactor();
  // IMU_DEBUG_PRINT("got conversion factor in IMU updatehook point 1: "<<test3)

  SPIDeviceInterface::updateHook();

  //Todo: re-set conversion factors, otherwise the initialized values are used --> some error, fix
  _acc.setConversionFactor(_acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000.0);
  _gyr.setConversionFactor(_gyr_dps_digit);
  _mag.setConversionFactor(_mag_mgauss_lsb/1000.0);
  _tmp.setConversionFactor(1.0/LSM9DS0_TEMP_LSB_DEGREE_CELSIUS);
  _tmp.setOffset(_tmp_offset,0,0);
  _acc.setOffset(_acc_offset[0],_acc_offset[1], _acc_offset[2]);
  _acc.setScale(_acc_scale[0],_acc_scale[1], _acc_scale[2]);

  // double test0 = _acc.getConversionFactor();
  // IMU_DEBUG_PRINT("got conversion factor in IMU updatehook point 2: "<<test0)

  updateMeasurements(); //reads in the three sensors and assigns the latest data to the _data field of the 3Dsensor
	IMU_DEBUG_PRINT("IMU data updated!")
}

void IMU::stopHook() {

  //Clean up GPIO's:
  IMU_DEBUG_PRINT("Cleaning up the GPIO's")
  cleanupGPIO(_cs_accmag);
  cleanupGPIO(_cs_gyr);

  IMU_DEBUG_PRINT("IMU executes stopping!")
}

void IMU::cleanupHook() {
	IMU_DEBUG_PRINT("IMU cleaning up!")
}

//Define methods:
//--------------

uint8_t IMU::pin2GPIO(uint8_t pin){
  //Odroid pins are internally connected to GPIO's which are located on GPIO chips.
  //To make the GPIO accessible you need the GPIO number.
  //The user only thas the pin number. This function makes the link between the two.
  uint8_t GPIO = 0; //need to initialize, otherwise its value is the same for different function calls
  IMU_DEBUG_PRINT("The pin number is: " << (int)pin)
  switch(pin){
    case 13:      //CSG
      GPIO = 21;
      IMU_DEBUG_PRINT("The GPIO number is: " << (int)GPIO)
      break; //otherwise default is always selected afterwards
    case 15:			//CSXM
      GPIO = 18;
      IMU_DEBUG_PRINT("The GPIO number is: " << (int)GPIO)
      break; //otherwise default is always selected afterwards
    default:
      RTT::log(RTT::Error) << "Invalid pin selected, normally only pins 13 and 15 should be connected" << RTT::endlog();
      //Todo: output some special GPIO then? Right now GPIO = 0 is output. No, an error seems enough?
  }
  return GPIO;
}

void IMU::init(){

  //Convert pin to GPIO number and activate GPIO's
  _cs_accmag  = pin2GPIO(_pin_accmag);
  _cs_gyr     = pin2GPIO(_pin_gyr);

  IMU_DEBUG_PRINT("Chip select of accmag is on GPIO: " << (int)_cs_accmag)
  IMU_DEBUG_PRINT("Chip select of gyro is on GPIO: " << (int)_cs_gyr)

  //Initialize GPIO as output pin with value 1 (high)
  initGPIO(_cs_accmag);
  initGPIO(_cs_gyr);

  //Make sensor settings
  IMU_DEBUG_PRINT("file descriptor before first write: " <<(int)_fd)
  IMU_DEBUG_PRINT("address of cs_accmag before first write: " <<&_cs_accmag)

  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG1_XM, 0x67);        // 100hz XYZ

  IMU_DEBUG_PRINT("chip select accelerometer after first write: " <<(int)_cs_accmag)
  IMU_DEBUG_PRINT("address of cs_accmag after first write: " <<&_cs_accmag)
  IMU_DEBUG_PRINT("file descriptor after first write: " <<(int)_fd)

  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000);  //temp sensor enabled, high res, 50Hz mag
  // enable mag continuous
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);  //normal mode
  // enable gyro continuous
  writeByte(_cs_gyr, LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F);            // turn on XYZ

  //Assign the conversion factors and the range registers, corresponding to the selected ranges of the sensor
  IMU_DEBUG_PRINT("acc range: "<<_acc_range)
  IMU_DEBUG_PRINT("mag range: "<<_mag_range)
  IMU_DEBUG_PRINT("gyr_range: "<<_gyr_range)

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

  IMU_DEBUG_PRINT("_acc_mg_lsb: "     <<_acc_mg_lsb)
  IMU_DEBUG_PRINT("_mag_mgauss_lsb: " <<_mag_mgauss_lsb)
  IMU_DEBUG_PRINT("_gyr_dps_digit: "  <<_gyr_dps_digit)

  //Set default ranges for the various sensors
  setupAccel(_acc_range_register);
  setupMag  (_mag_range_register);
  setupGyro (_gyr_range_register);

  IMU_DEBUG_PRINT("_acc_range_register: "  <<_acc_range_register)
  IMU_DEBUG_PRINT("_mag_range_register: "  <<_mag_range_register)
  IMU_DEBUG_PRINT("_gyr_range_register: "  <<_gyr_range_register)

  //Make Sensor3D instances
  _id = 0;
  Sensor3D _acc(_id+1);
  _acc.setOffset(_acc_offset[0],_acc_offset[1], _acc_offset[2]);
  _acc.setScale(_acc_scale[0],_acc_scale[1], _acc_scale[2]);
  _acc.setConversionFactor(_acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000.0);
  IMU_DEBUG_PRINT("conversion factor accelerometer: "<<_acc_mg_lsb*SENSORS_GRAVITY_STANDARD/1000.0)
  Sensor3D _gyr(_id+2);
  _gyr.setOffset(_gyr_offset[0],_gyr_offset[1], _gyr_offset[2]);
  _gyr.setScale(_gyr_scale[0],_gyr_scale[1], _gyr_scale[2]);
  _gyr.setConversionFactor(_gyr_dps_digit);
  IMU_DEBUG_PRINT("conversion factor gyroscope: "<<_gyr_dps_digit)
  Sensor3D _mag(_id+3);
  _mag.setOffset(_mag_offset[0],_mag_offset[1], _mag_offset[2]);
  _mag.setScale(_mag_scale[0],_mag_scale[1], _mag_scale[2]);
  _mag.setConversionFactor(_mag_mgauss_lsb/1000.0);
  IMU_DEBUG_PRINT("conversion factor magnetometer: "<<_mag_mgauss_lsb/1000.0)
  Sensor3D _tmp(_id+4);
  _tmp.setOffset(_tmp_offset,0,0);
  _tmp.setScale(_tmp_scale,0,0);
  _tmp.setConversionFactor(1.0/LSM9DS0_TEMP_LSB_DEGREE_CELSIUS);
  IMU_DEBUG_PRINT("conversion factor temperature: "<<1.0/LSM9DS0_TEMP_LSB_DEGREE_CELSIUS)

  // double test = _acc.getConversionFactor();
  // IMU_DEBUG_PRINT("got conversion factor: "<<test)
} //END init()

bool  IMU::isConnected(){
  //Check standard ID of sensor (see STM LSM9DS0 datasheet)

  IMU_DEBUG_PRINT("in IMU::isConnected(), chip select accelerometer: "<<(int)_cs_accmag)
  IMU_DEBUG_PRINT("in IMU::isConnected(), register: "<<(int)LSM9DS0_REGISTER_WHO_AM_I_XM)
  uint8_t id = readByte(_cs_accmag, LSM9DS0_REGISTER_WHO_AM_I_XM); //read id
  IMU_DEBUG_PRINT("Accmag ID is: "<<(int)id)
  if (id != LSM9DS0_XM_ID){ //check id
  	RTT::log(RTT::Error) << "Received ID does not correspond to accelerometer/magnetometer ID!" <<RTT::endlog();
    return false;
  }

  IMU_DEBUG_PRINT("in IMU::isConnected(), chip select gyroscope: "<<(int)_cs_gyr)
  IMU_DEBUG_PRINT("in IMU::isConnected(), register: "<<(int)LSM9DS0_REGISTER_WHO_AM_I_G)
  id = readByte(_cs_gyr, LSM9DS0_REGISTER_WHO_AM_I_G); //read id
  IMU_DEBUG_PRINT("Gyro ID is: "<<(int)id)
  if (id != LSM9DS0_G_ID){ //check id
  	RTT::log(RTT::Error) << "Received ID does not correspond to gyroscope ID!" <<RTT::endlog();
    return false;
  }

  else{
  	IMU_DEBUG_PRINT("Both sensors are connected!")
    return true;
  }
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
  uint8_t reg = readByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG6_XM); //same principle as readByte
  reg &= ~(0b01100000);
  reg |= range;
  writeByte(_cs_accmag, LSM9DS0_REGISTER_CTRL_REG6_XM, reg);
}

void IMU::setupGyro (uint8_t range)
{
  uint8_t reg = readByte(_cs_gyr, LSM9DS0_REGISTER_CTRL_REG4_G); //same principle as readByte
  reg &= ~(0b00110000);
  reg |= range;
  writeByte(_cs_gyr, LSM9DS0_REGISTER_CTRL_REG4_G, reg);
}

void IMU::updateMeasurements()
{

  double test1 = _acc.getConversionFactor();
  IMU_DEBUG_PRINT("got conversion factor in updateMeasurements: ")

  //Read all the sensors
  readAccel();
  readMag();
  readGyro();
  readTemp();

  IMU_DEBUG_PRINT("Ready to calibrate IMU data!")

  double test2 = _acc.getConversionFactor();
  IMU_DEBUG_PRINT("got conversion factor after read: "<<test2)

  //Calibrate: convert raw measurements to SI-units, with user-defined offset and scaling factor and selected conversion factor
  _cal_imu_transacc         = _acc.calibrate(); //contains accelerations along all axes
  _cal_imu_orientation_3d   = _mag.calibrate(); //contains roll pitch yaw angles
  _cal_imu_dorientation_3d  = _gyr.calibrate(); //contains angular velocity along all axes
  _cal_imu_dorientation     = _cal_imu_dorientation_3d[2];//select angular velocity around z-axis
  _cal_imu_orientation      = _cal_imu_orientation_3d[2]; //select yaw angle
  _cal_imu_temperature      = _tmp.calibrate(); //contains temperature

  IMU_DEBUG_PRINT("_cal_imu_transacc: "       <<_cal_imu_transacc[0]<<" , "<<_cal_imu_transacc[1]<<" , "<<_cal_imu_transacc[2])
  IMU_DEBUG_PRINT("_cal_imu_orientation_3d: " <<_cal_imu_orientation_3d[0]<<" , "<<_cal_imu_orientation_3d[1]<<" , "<<_cal_imu_orientation_3d[2])
  IMU_DEBUG_PRINT("_cal_imu_dorientation_3d: "<<_cal_imu_dorientation_3d[0]<<" , "<<_cal_imu_dorientation_3d[1]<<" , "<<_cal_imu_dorientation_3d[2])
  IMU_DEBUG_PRINT("_cal_imu_dorientation z-axis: "   <<_cal_imu_dorientation)
  IMU_DEBUG_PRINT("_cal_imu_orientation z-axis: "    <<_cal_imu_orientation)
  IMU_DEBUG_PRINT("_cal_imu_temperature: "    <<_cal_imu_temperature[0])

  //Todo: compensate for roll/pitch/yaw in angular velocity measurements? No, better translate to world frame when using these measurements
  //Todo: or make separate variables for measurements in gauss and rpy angles?
  _cal_imu_orientation_3d = convertMag2rpy(); //convert magnetic field measurements to roll pitch yaw angles

  IMU_DEBUG_PRINT("calculated rpy [deg]: "<<_cal_imu_orientation_3d[0]*180/pi<< " , "<< _cal_imu_orientation_3d[1]*180/pi<< " , "<<_cal_imu_orientation_3d[2]*180/pi)

  //Write calibrated data to output ports
  _cal_imu_transacc_port.write        (_cal_imu_transacc);
  _cal_imu_dorientation_3d_port.write (_cal_imu_dorientation_3d);
  _cal_imu_orientation_3d_port.write  (_cal_imu_orientation_3d);
  _cal_imu_dorientation_port.write    (_cal_imu_dorientation);
  _cal_imu_orientation_port.write     (_cal_imu_orientation);
  _cal_imu_temperature_port.write     (_cal_imu_temperature[0]);
}

void IMU::readAccel() {

  uint8_t length = 2 * 3; //read 3 16 bits (= 2 byte) numbers, so read out 6 bytes
  uint8_t reg = LSM9DS0_REGISTER_OUT_X_L_A;
  uint8_t buffer[7]; //make a buffer to hold 6+1 bytes, being what you will read + byte for the register

  IMU_DEBUG_PRINT("In readAccel")

  readBytes(_cs_accmag, reg, length, buffer);

  uint8_t xlo = buffer[1]; //low  bit of x-acceleration
  int16_t xhi = buffer[2]; //high bit of x-acceleration
  uint8_t ylo = buffer[3];
  int16_t yhi = buffer[4];
  uint8_t zlo = buffer[5];
  int16_t zhi = buffer[6];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  IMU_DEBUG_PRINT("Reading raw accelerometer data: ")
  IMU_DEBUG_PRINT("acc_x = "<<(int)xhi)
  IMU_DEBUG_PRINT("acc_y = "<<(int)yhi)
  IMU_DEBUG_PRINT("acc_z = "<<(int)zhi)

  std::vector<double> data(3); //create vector to hold measured data
  data[0] = xhi;
  data[1] = yhi;
  data[2] = zhi;
  _acc.writeData(data); //save raw data
  _raw_imu_acc_port.write(data); //write raw data to output port
}

void IMU::readMag() {

  uint8_t length = 2 * 3; //read 3 2 byte numbers
  uint8_t reg = LSM9DS0_REGISTER_OUT_X_L_M;
  uint8_t buffer[7]; //make a buffer to hold 6+1 bytes (= 3*16bits), being what you will read + byte for the register

  IMU_DEBUG_PRINT("In readMag")

  readBytes(_cs_accmag, reg, length, buffer);

  uint8_t xlo = buffer[1]; //low  bit of x-magnetometer
  int16_t xhi = buffer[2]; //high bit of x-magnetometer
  uint8_t ylo = buffer[3];
  int16_t yhi = buffer[4];
  uint8_t zlo = buffer[5];
  int16_t zhi = buffer[6];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  IMU_DEBUG_PRINT("Reading raw magnetometer data: ")
  IMU_DEBUG_PRINT("mag_x = "<<(int)xhi)
  IMU_DEBUG_PRINT("mag_y = "<<(int)yhi)
  IMU_DEBUG_PRINT("mag_z = "<<(int)zhi)

  std::vector<double> data(3); //create vector to hold measured data
  data[0] = xhi;
  data[1] = yhi;
  data[2] = zhi;
  _mag.writeData(data); //save raw data
  _raw_imu_mag_port.write(data); //write raw data to output port

}

void IMU::readGyro() {

  uint8_t length = 2 * 3; //read 3 2 byte numbers
  uint8_t reg = LSM9DS0_REGISTER_OUT_X_L_G;
  uint8_t buffer[7]; //make a buffer to hold 6+1 bytes (= 3*16bits), being what you will read + byte for the register

  IMU_DEBUG_PRINT("In readGyro")

  readBytes(_cs_gyr, reg, length, buffer);

  uint8_t xlo = buffer[1]; //low  bit of x-gyroscope
  int16_t xhi = buffer[2]; //high bit of x-gyroscope
  uint8_t ylo = buffer[3];
  int16_t yhi = buffer[4];
  uint8_t zlo = buffer[5];
  int16_t zhi = buffer[6];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  IMU_DEBUG_PRINT("Reading raw gyroscope data: ")
  IMU_DEBUG_PRINT("gyr_x = "<<(int)xhi)
  IMU_DEBUG_PRINT("gyr_y = "<<(int)yhi)
  IMU_DEBUG_PRINT("gyr_z = "<<(int)zhi)

  std::vector<double> data(3); //create vector to hold measured data
  data[0] = xhi;
  data[1] = yhi;
  data[2] = zhi;
  _gyr.writeData(data); //save raw data
  _raw_imu_gyr_port.write(data); //write raw data to output port
}

void IMU::readTemp() {

  uint8_t length = 2 * 1; //read 1 2 byte number
  uint8_t reg = LSM9DS0_REGISTER_TEMP_OUT_L_XM;
  uint8_t buffer[3]; //make a buffer to hold 2 bytes (= 1*16bits), being what you will read

  IMU_DEBUG_PRINT("In readTemp")

  readBytes(_cs_accmag, reg, length, buffer);

  IMU_DEBUG_PRINT("reading temperature0: "<<(int)buffer[0])
  IMU_DEBUG_PRINT("reading temperature1: "<<(int)buffer[1])
  IMU_DEBUG_PRINT("reading temperature2: "<<(int)buffer[2])

  uint8_t xlo = buffer[1]; //low bit of temperature sensor
  int16_t xhi = buffer[2]; //high bit of temperature sensor

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;

  IMU_DEBUG_PRINT("Reading raw temperature data: ")
  IMU_DEBUG_PRINT("tmp = "<<(int)xhi)

  std::vector<double> data(1); //create vector to hold measured data
  data[0] = xhi;
  _tmp.writeData(data); //save raw data
  _raw_imu_tmp_port.write(data); //write raw data to output port

}

std::vector<double> IMU::convertMag2rpy(){
	//Reference: https://www.pololu.com/file/download/LSM303DLH-compass-app-note.pdf?file_id=0J434
	std::vector<double> rpy(3); //contains roll pitch yaw angles
  std::vector<double> acc = _cal_imu_transacc; //copy accelerometer data
  std::vector<double> mag = _cal_imu_orientation_3d; //copy magnetometer data

	IMU_DEBUG_PRINT("acceleration in rpy: "<<acc[0]<<" , "<<acc[1]<<" , "<<acc[2])
  IMU_DEBUG_PRINT("magnetic field in rpy: "<<mag[0]<<" , "<<mag[1]<<" , "<<mag[2])

  //Normalize accelerometer measurements
  double acc_norm=sqrt(pow(acc[0],2)+pow(acc[1],2)+pow(acc[2],2));
	acc[0]=acc[0]/acc_norm;
	acc[1]=acc[1]/acc_norm;
	acc[2]=acc[2]/acc_norm;
	//Normalize magnetometer measurements
  double mag_norm=sqrt(pow(mag[0],2)+pow(mag[1],2)+pow(mag[2],2));
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
  	rpy[2] += 2*pi;
  }
  IMU_DEBUG_PRINT("roll: "<<rpy[0]<<" , "<<"pitch: "<<rpy[1]<<" , "<<"yaw: "<<rpy[2])
  return rpy;
}


//Get sensor data
std::vector<double> IMU::getImuTransAcc()      { return _cal_imu_transacc; }
std::vector<double> IMU::getImuDOrientation3D(){ return _cal_imu_dorientation_3d; }
std::vector<double> IMU::getImuOrientation3D() { return _cal_imu_orientation_3d; }
double              IMU::getImuDOrientation()  { return _cal_imu_dorientation; }
double              IMU::getImuOrientation()   { return _cal_imu_orientation; }

//Set sensor data
//Todo: only useful for testing?
//Note: originally there was: std::vector<double>, const& cal_imu_transacc? pass by constant reference
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
