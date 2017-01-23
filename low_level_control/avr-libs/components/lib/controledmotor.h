#ifndef CONTROLEDMOTOR_H
#define CONTROLEDMOTOR_H

#include "WProgram.h"
#include "hbridge_interface.h"
#include "encoder_sensor.h"
#include "ctrllib_pkg_parametric.h"
#include "rate_limiter.h"

#define CONTROLEDMOTOR_MODE_OFF		0
#define CONTROLEDMOTOR_MODE_VOLT	1
#define CONTROLEDMOTOR_MODE_CUR		2
#define CONTROLEDMOTOR_MODE_SPD		3
#define CONTROLEDMOTOR_MODE_POS		4
#define CONTROLEDMOTOR_MODE_CR 		0xFF

#define CURRENT_LOOP_ENABLE				0x01
#define VELOCITY_LOOP_ENABLE			0x02
#define POSITION_LOOP_ENABLE			0x04

class ControledMotor
{
private:
	HBridgeInterface* _hbridge; 
	Sensor1D*	_current_sensor;
	Sensor1D*	_velocity_sensor;
	Sensor1D*	_position_sensor;
	
	int	_ref_current;
	int	_error_current;
	int	_ref_velocity;
	int	_error_velocity;
	int _ref_position;
	int	_error_position;
	
	//controller
	uint8_t _mode;
	uint8_t _loop_enable;
	config_parametric_pid_t _config_current_controller;
	state_parametric_pid_t  _state_current_controller;
	config_parametric_pid_t _config_velocity_controller;
	state_parametric_pid_t  _state_velocity_controller;
	config_parametric_pid_t _config_position_controller;
	state_parametric_pid_t  _state_position_controller;
	
public:
	ControledMotor(HBridgeInterface* hbridge, const float Ts, Sensor1D* current_sensor, Sensor1D* velocity_sensor, Sensor1D* position_sensor);
	
	void update();
	
	void setCurrentController(float P, float I, float D);
	void setVelocityController(float P, float I, float D);
	void setPositionController(float P, float I, float D);
	void setLoopEnable(uint8_t loop_enable);

	bool setVoltage(int16_t voltage);
	bool setRefCurrent(int reference);
	bool setRefVelocity(int reference);
	bool setRefPosition(int reference);
	bool setReference(int reference, uint8_t mode = CONTROLEDMOTOR_MODE_CR);
	bool setMode(uint8_t mode);

	HBridgeInterface* getHBridge();
	Sensor1D* getCurrentSensor();
	Sensor1D* getVelocitySensor();
	Sensor1D* getPositionSensor();

	int getVoltage();
	int getCurrent();
	int getVelocity();
	int getPosition();
	int getRefCurrent();
	int getRefVelocity();
	int getRefPosition();
	int getReference();
	int getErrorCurrent();
	int getErrorVelocity();
	int getErrorPosition();
};

#endif //CONTROLEDMOTOR_H
