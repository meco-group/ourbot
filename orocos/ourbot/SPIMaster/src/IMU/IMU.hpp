#ifndef OROCOS_IMU_HPP
#define OROCOS_IMU_HPP

#define IMU_TESTFLAG //to manually set properties while testing

#include <vector>
#include <cmath>
#include "sensor3d.hpp"
//#include "/home/tim/orocos/ourbot/orocos/ourbot/SPIMaster/src/SPIDeviceInterface/SPIDeviceInterface.hpp" //for use on notebook
#include "/home/odroid/orocos/SPIMaster/src/SPIDeviceInterface/SPIDeviceInterface.hpp" //for use on Odroid
//Todo: why specifically need to specify the path here? Normally #include "SPIDeviceInterface.hpp" should also work...

using namespace RTT; //avoids that you have to put RTT::OutputPort, just use OutputPort

class IMU : public SPIDeviceInterface{ //IMU inherits from SPIDeviceInterface

  private: 
	
		//Define output ports
		OutputPort<std::vector<double> > _cal_imu_transacc_port;		    //translational acceleration port
		OutputPort<std::vector<double> > _cal_imu_dorientation_3d_port; //angular velocity (3d) port
  	OutputPort<std::vector<double> > _cal_imu_orientation_3d_port;  //orientation (3d) port
 	 	OutputPort<double>               _cal_imu_dorientation_port;    //angular velocity around z-axis port
  	OutputPort<double>               _cal_imu_orientation_port;     //orientation in x-y plane port
  	OutputPort<std::vector<double> > _cal_imu_temperature_port;     //temperature port

		//Define variables to send to output ports
		std::vector<double> _cal_imu_transacc;
  	std::vector<double> _cal_imu_dorientation_3d;
  	std::vector<double> _cal_imu_orientation_3d;
  	double              _cal_imu_dorientation;
  	double              _cal_imu_orientation;
  	std::vector<double> _cal_imu_temperature;

		//Define input ports
		OutputPort<std::vector<double> > _raw_imu_acc_port; //accelerometer data input port
		OutputPort<std::vector<double> > _raw_imu_gyr_port; //gyroscope data input port
  	OutputPort<std::vector<double> > _raw_imu_mag_port; //magnetometer data input port
  	OutputPort<std::vector<double> > _raw_imu_tmp_port; //temperature data input port

		//Define variables for incoming signals
		//Note: read out everything, afterwards select 3d or 1d orientation or dorientation in some method
		std::vector<double> _raw_imu_acc;
  	std::vector<double> _raw_imu_gyr;
  	std::vector<double> _raw_imu_mag;
  	std::vector<double> _raw_imu_tmp;


		//Define sensor constants:
		//for more info see datasheet section "register description" of LSM9DS0 STM sensor
		#define LSM9DS0_ADDRESS_ACCELMAG  (0x1D)        // 3B >> 1 = 7bit default
		#define LSM9DS0_ADDRESS_GYRO      (0x6B)        // D6 >> 1 = 7bit default
		#define LSM9DS0_XM_ID             (0b01001001)  //Binary ID of the accmag
		#define LSM9DS0_G_ID              (0b11010100)  //Binary ID of the gyro

		// #define GYROTYPE                  (true)        //boolean to select the gyro
		// #define XMTYPE                    (false)		    //boolean to select the accmag

		//Define conversion factors to go to SI-units
		// Linear Acceleration: mg per LSB
		#define LSM9DS0_ACCEL_MG_LSB_2G  (0.061F)
		#define LSM9DS0_ACCEL_MG_LSB_4G  (0.122F)
		#define LSM9DS0_ACCEL_MG_LSB_6G  (0.183F)
		#define LSM9DS0_ACCEL_MG_LSB_8G  (0.244F)
		#define LSM9DS0_ACCEL_MG_LSB_16G (0.732F) // Is this right? Was expecting 0.488F

		#define SENSORS_GRAVITY_STANDARD (9.80665F)// Earth's gravity in m/s^2

		//Magnetic Field Strength: gauss range
		#define LSM9DS0_MAG_MGAUSS_2GAUSS  (0.08F)
		#define LSM9DS0_MAG_MGAUSS_4GAUSS  (0.16F)
		#define LSM9DS0_MAG_MGAUSS_8GAUSS  (0.32F)
		#define LSM9DS0_MAG_MGAUSS_12GAUSS (0.48F)

		//Angular Rate: dps per LSB
		#define LSM9DS0_GYRO_DPS_DIGIT_245DPS  (0.00875F)
		#define LSM9DS0_GYRO_DPS_DIGIT_500DPS  (0.01750F)
		#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS (0.07000F)

		//Temperature: LSB per degree celsius
		#define LSM9DS0_TEMP_LSB_DEGREE_CELSIUS (8) // 1°C = 8, 25° = 200, etc.

	//Define sensor addresses: for more info see datasheet section "register description" of LSM9DS0 STM sensor
	//Todo: remove the unnecessary registers
		#define LSM9DS0_REGISTER_WHO_AM_I_G          (0x0F) //ID of gyroscope register, must correspond to the global variable LSM9DS0_G_ID
	  #define LSM9DS0_REGISTER_CTRL_REG1_G         (0x20)//set operation mode of gyro
	  #define LSM9DS0_REGISTER_CTRL_REG3_G         (0x22)
	  #define LSM9DS0_REGISTER_CTRL_REG4_G         (0x23)
	  #define LSM9DS0_REGISTER_OUT_X_L_G   				 (0x28) //x-axis angular data rate last 8 bits
	  #define LSM9DS0_REGISTER_OUT_X_H_G           (0x29) //x-axis angular data rate first 8 bits
	  #define LSM9DS0_REGISTER_OUT_Y_L_G           (0x2A) //y-axis angular data rate last 8 bits
	  #define LSM9DS0_REGISTER_OUT_Y_H_G           (0x2B) //y-axis angular data rate first 8 bits
	  #define LSM9DS0_REGISTER_OUT_Z_L_G           (0x2C) //z-axis angular data rate last 8 bits
	  #define LSM9DS0_REGISTER_OUT_Z_H_G           (0x2D) //z-axis angular data rate first 8 bits

		#define LSM9DS0_REGISTER_TEMP_OUT_L_XM       (0x05) //temperature output last 8 bits
	  #define LSM9DS0_REGISTER_TEMP_OUT_H_XM       (0x06)
	  #define LSM9DS0_REGISTER_STATUS_REG_M        (0x07)
	  #define LSM9DS0_REGISTER_OUT_X_L_M           (0x08) //x-axis magnet data rate last 8 bits
	  #define LSM9DS0_REGISTER_OUT_X_H_M           (0x09) //x-axis magnet data rate first 8 bits
	  #define LSM9DS0_REGISTER_OUT_Y_L_M           (0x0A)
	  #define LSM9DS0_REGISTER_OUT_Y_H_M           (0x0B)
	  #define LSM9DS0_REGISTER_OUT_Z_L_M           (0x0C)
	  #define LSM9DS0_REGISTER_OUT_Z_H_M           (0x0D)
	  #define LSM9DS0_REGISTER_WHO_AM_I_XM         (0x0F) //ID of accmag register, must correspond to the global variable LSM9DS0_XM_ID
	  #define LSM9DS0_REGISTER_INT_CTRL_REG_M      (0x12)
	  #define LSM9DS0_REGISTER_INT_SRC_REG_M       (0x13)
	  #define LSM9DS0_REGISTER_CTRL_REG1_XM        (0x20) //select the speed
	  #define LSM9DS0_REGISTER_CTRL_REG2_XM        (0x21)
	  #define LSM9DS0_REGISTER_CTRL_REG5_XM        (0x24) //let acc operate continuously
	  #define LSM9DS0_REGISTER_CTRL_REG6_XM        (0x25)
	  #define LSM9DS0_REGISTER_CTRL_REG7_XM        (0x26) //let mag operate continuously
	  #define LSM9DS0_REGISTER_OUT_X_L_A           (0x28) //x-axis acceleration data rate last 8 bits
	  #define LSM9DS0_REGISTER_OUT_X_H_A           (0x29) //x-axis acceleration data rate
	  #define LSM9DS0_REGISTER_OUT_Y_L_A           (0x2A)
	  #define LSM9DS0_REGISTER_OUT_Y_H_A           (0x2B)
	  #define LSM9DS0_REGISTER_OUT_Z_L_A           (0x2C)
	  #define LSM9DS0_REGISTER_OUT_Z_H_A           (0x2D)

	//Define numbers for the set-up of the mag, acc and gyr ranges and speeds
		#define  LSM9DS0_ACCELRANGE_2G               (0b000 << 3) //register to set the accelerometer range//
		#define  LSM9DS0_ACCELRANGE_4G               (0b001 << 3) //<<3 because the 8-bit number to set the accel range has the scale settings in bit 3,4,5. So starting from the right (bit 8) you need to shift the number 001 3 bits to the left to get the scale bits in the right place
		#define  LSM9DS0_ACCELRANGE_6G               (0b010 << 3)
		#define  LSM9DS0_ACCELRANGE_8G               (0b011 << 3)
		#define  LSM9DS0_ACCELRANGE_16G              (0b100 << 3)

	  #define LSM9DS0_ACCELDATARATE_POWERDOWN      (0b0000 << 4)
	  #define LSM9DS0_ACCELDATARATE_3_125HZ        (0b0001 << 4)
	  #define LSM9DS0_ACCELDATARATE_6_25HZ         (0b0010 << 4)
	  #define LSM9DS0_ACCELDATARATE_12_5HZ         (0b0011 << 4)
	  #define LSM9DS0_ACCELDATARATE_25HZ           (0b0100 << 4)
	  #define LSM9DS0_ACCELDATARATE_50HZ           (0b0101 << 4)
	  #define LSM9DS0_ACCELDATARATE_100HZ          (0b0110 << 4)
	  #define LSM9DS0_ACCELDATARATE_200HZ          (0b0111 << 4)
	  #define LSM9DS0_ACCELDATARATE_400HZ          (0b1000 << 4)
	  #define LSM9DS0_ACCELDATARATE_800HZ          (0b1001 << 4)
	  #define LSM9DS0_ACCELDATARATE_1600HZ         (0b1010 << 4)

	  #define LSM9DS0_MAGGAIN_2GAUSS               (0b00 << 5)  // +/- 2 gauss
	  #define LSM9DS0_MAGGAIN_4GAUSS               (0b01 << 5)  // +/- 4 gauss
	  #define LSM9DS0_MAGGAIN_8GAUSS               (0b10 << 5)  // +/- 8 gauss
	  #define LSM9DS0_MAGGAIN_12GAUSS              (0b11 << 5)  // +/- 12 gauss

		#define LSM9DS0_MAGDATARATE_3_125HZ          (0b000 << 2)
		#define LSM9DS0_MAGDATARATE_6_25HZ           (0b001 << 2)
	  #define LSM9DS0_MAGDATARATE_12_5HZ           (0b010 << 2)
	  #define LSM9DS0_MAGDATARATE_25HZ             (0b011 << 2)
	  #define LSM9DS0_MAGDATARATE_50HZ             (0b100 << 2)
	  #define LSM9DS0_MAGDATARATE_100HZ            (0b101 << 2)

	  #define LSM9DS0_GYROSCALE_245DPS             (0b00 << 4)  // +/- 245 degrees per second rotation
	  #define LSM9DS0_GYROSCALE_500DPS             (0b01 << 4)  // +/- 500 degrees per second rotation
	  #define LSM9DS0_GYROSCALE_2000DPS            (0b10 << 4)  // +/- 2000 degrees per second rotation

  	//Define properties:
  	//for calibration
		double _acc_offset[3]; //user-defined offset of measurements
		double _gyr_offset[3];
		double _mag_offset[3];
		double _tmp_offset;
		double _acc_scale[3];  //user-defined scaling of measurements
		double _gyr_scale[3];
		double _mag_scale[3];
		double _tmp_scale;
		int _acc_range;        //user-selected range for accelerometer
		int _gyr_range; 
		int _mag_range; 
		uint8_t _id;           //sensor id
		
		uint8_t _pin_accmag;//pin to which accmag cs is connected
		uint8_t _pin_gyr;   //pin to which gyr cs is connected
		uint8_t _cs_accmag; //GPIO to which accmag cs is connected
		uint8_t _cs_gyr;    //GPIO to which gyr cs is connected
		//Some pin numbers correspond to a certain GPIO number. pin2GPIO() makes this conversion

		//Define other variables:
		double 	_acc_mg_lsb;     //holds conversion factor for accelerometer
  	double 	_mag_mgauss_lsb; //holds conversion factor for magnetometer
  	double 	_gyr_dps_digit;  //holds conversion factor for gyroscope
  	uint8_t _acc_range_register;
  	uint8_t _mag_range_register;
  	uint8_t _gyr_range_register;
  	Sensor3D 	_acc; //accelerometer
  	Sensor3D 	_mag; //magnetometer
  	Sensor3D 	_gyr; //gyroscope
  	Sensor3D 	_tmp; //temperature sensor

		//Methods
	  void  	init();   //initialize sensor

    //Set-up sensor ranges
    void setupAccel (uint8_t range);
    void setupMag   (uint8_t range);
    void setupGyro  (uint8_t range);

    //Read in data
		void  	updateMeasurements(); //get new measurements
		void 		readAccel();
  	void 		readMag();
  	void 		readGyro();
  	void 		readTemp();
  	std::vector<double> convertMag2rpy(); //convert magnetic field measurements to roll pitch yaw angles

  	//Assign measurements to variables
  	void setImuTransAcc				(std::vector<double> cal_imu_transacc);
    void setImuDOrientation3D	(std::vector<double> cal_imu_dorientation_3d);
    void setImuOrientation3D 	(std::vector<double> cal_imu_orientation_3d);
    void setImuDOrientation		(double cal_imu_dorientation);
    void setImuOrientation 		(double cal_imu_orientation);
		
		uint8_t pin2GPIO(uint8_t pin); //convert pin number to corresponding GPIO number

  public:
	
    IMU(std::string const& name); //Constructor
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    double const pi=4*atan(1);

  protected: 

  	//IMU data getters
		std::vector<double> getImuTransAcc();
    std::vector<double> getImuDOrientation3D();
    std::vector<double> getImuOrientation3D();
    double 							getImuDOrientation();
    double 							getImuOrientation();
		
		bool 	isConnected(); //Check if sensor is connected

};
#endif //IMU
