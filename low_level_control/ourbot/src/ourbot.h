#ifndef OURBOT_H
#define OURBOT_H

#include <inttypes.h>
#include "eeprom_m.h"
#include "controledmotor.h"
#include "hbridge_interface.h"
#include "Adafruit_LSM9DS0.h"
#include "ourbot_hal.h"

class Ourbot
{
private:    
	uint8_t _ID;
    uint32_t _mode;
    ControledMotor	_controledMotors[4];
    Adafruit_LSM9DS0* _imus[2];
	
public:
    Ourbot(OurbotHAL* hal, Adafruit_LSM9DS0* imu1, Adafruit_LSM9DS0* imu2, uint32_t mode = 0);
    Ourbot(HBridgeInterface* motor0, HBridgeInterface* motor1, HBridgeInterface* motor2, HBridgeInterface* motor3,
    	   Sensor1D* current_sensor0, Sensor1D* current_sensor1, Sensor1D* current_sensor2, Sensor1D* current_sensor3,
    	   Sensor1D* encoder0, Sensor1D* encoder1, Sensor1D* encoder2, Sensor1D* encoder3, Adafruit_LSM9DS0* imu1, Adafruit_LSM9DS0* imu2, uint32_t mode = 0);
	
	////////
	/// FUNC
	void init();
	void controllerHook();
	void sensorHook();
	
	///////
	/// SET
	uint8_t setID(uint8_t ID);
	uint32_t setMode(uint32_t mode);
	
	///////
	/// GET
    uint8_t ID();
    uint32_t mode();
    ControledMotor* controledMotorAR(uint8_t n);
    ControledMotor* controledMotorID(uint8_t n);
    Adafruit_LSM9DS0* imuAR(uint8_t n);
    
};

#endif //OURBOT_H
