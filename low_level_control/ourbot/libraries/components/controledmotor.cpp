#include "controledmotor.h"

ControledMotor::ControledMotor(HBridgeInterface* hbridge, float Ts, Sensor1D* current_sensor, Sensor1D* velocity_sensor, Sensor1D* position_sensor) :
	_hbridge(hbridge),
	_current_sensor(current_sensor),
	_velocity_sensor(velocity_sensor),
	_position_sensor(position_sensor),
	_ref_current(0.0f),
	_error_current(0.0f),
	_ref_velocity(0.0f),
	_error_velocity(0.0f),
	_ref_position(0.0f),
	_error_position(0.0f),
	_mode(CONTROLEDMOTOR_MODE_OFF),
	_loop_enable(CURRENT_LOOP_ENABLE | VELOCITY_LOOP_ENABLE | POSITION_LOOP_ENABLE)
{
	configure_pid(&_config_current_controller, &_state_current_controller, Ts, 0.0f, 0.0f, 0.0f);
	configure_pid(&_config_velocity_controller, &_state_velocity_controller, Ts, 0.0f, 0.0f, 0.0f);
	configure_pid(&_config_position_controller, &_state_position_controller, Ts, 0.0f, 0.0f, 0.0f);
}

void ControledMotor::update()
{		
	//read all sensors
	_position_sensor->readCalibratedValue();
	_velocity_sensor->readCalibratedValue();
	_current_sensor->readCalibratedValue();
	
	if(_mode != CONTROLEDMOTOR_MODE_VOLT){

		int bridge_voltage = 0;
		float error;
	
		if((_mode >= CONTROLEDMOTOR_MODE_POS) && ((_loop_enable & POSITION_LOOP_ENABLE) > 0)){
			// execute current controller
			_error_position = _ref_position - _position_sensor->peekCalibratedValue();
			error = _error_position;
			update_pid(_config_position_controller, &_state_position_controller, &error);

			_ref_velocity = (int)_state_position_controller.y[0];
			_ref_current = (int)_state_position_controller.y[0];
			bridge_voltage = (int)_state_current_controller.y[0];
		}
		if((_mode >= CONTROLEDMOTOR_MODE_SPD) && (_velocity_sensor != NULL) && ((_loop_enable & VELOCITY_LOOP_ENABLE) > 0)){
			//execute velocity controller
			_error_velocity = _ref_velocity - _velocity_sensor->peekCalibratedValue();
			error = _error_velocity;
			update_pid(_config_velocity_controller, &_state_velocity_controller, &error);

			_ref_current = (int)_state_velocity_controller.y[0];
			bridge_voltage = (int)_state_velocity_controller.y[0];
		}
		if((_mode >= CONTROLEDMOTOR_MODE_CUR) && (_current_sensor != NULL) && ((_loop_enable & CURRENT_LOOP_ENABLE) > 0)){
			// execute current controller
			_error_current = _ref_current - _current_sensor->peekCalibratedValue();
			error = _error_current;
			update_pid(_config_current_controller, &_state_current_controller, &error);

			bridge_voltage = (int)_state_current_controller.y[0];
		}
	
		_hbridge->setBridgeVoltage(bridge_voltage);
	}
}

void ControledMotor::setCurrentController(float P, float I, float D)
{
	configure_pid(&_config_current_controller, &_state_current_controller, _config_current_controller.Ts, P, I, D);
}

void ControledMotor::setVelocityController(float P, float I, float D)
{
	configure_pid(&_config_velocity_controller, &_state_velocity_controller, _config_velocity_controller.Ts, P, I, D);
}

void ControledMotor::setPositionController(float P, float I, float D)
{
	configure_pid(&_config_position_controller, &_state_position_controller, _config_position_controller.Ts, P, I, D);
}

void ControledMotor::setLoopEnable(uint8_t loop_enable)
{
	_loop_enable = loop_enable;
}

bool ControledMotor::setVoltage(int16_t voltage){
	return setReference(voltage, CONTROLEDMOTOR_MODE_VOLT);
}

bool ControledMotor::setRefCurrent(int reference){
	return setReference(reference, CONTROLEDMOTOR_MODE_CUR);
}

bool ControledMotor::setRefVelocity(int reference){
	return setReference(reference, CONTROLEDMOTOR_MODE_SPD);
}

bool ControledMotor::setRefPosition(int reference){ 
	return setReference(reference, CONTROLEDMOTOR_MODE_POS);
}

bool ControledMotor::setReference(int reference, uint8_t mode)
{
	if(mode > CONTROLEDMOTOR_MODE_POS){
		setReference(reference, _mode);
	}	else {
		_mode = mode;
		switch(mode){			
			case CONTROLEDMOTOR_MODE_VOLT:
				_hbridge->setBridgeVoltage(reference);
				break;
			case CONTROLEDMOTOR_MODE_CUR:
				_ref_current = reference;
				break;
			case CONTROLEDMOTOR_MODE_SPD:
				_ref_velocity = reference;
				break;
			case CONTROLEDMOTOR_MODE_POS:
				_ref_position = reference;
				break;
		}
	}
	
	return true;
}

bool ControledMotor::setMode(uint8_t mode)
{
	if(mode <= CONTROLEDMOTOR_MODE_POS){
		int reference = 0;
		if(mode == CONTROLEDMOTOR_MODE_POS)
			reference = getPosition();
		return setReference(reference, mode);
	} else { return false; }
}

HBridgeInterface* ControledMotor::getHBridge(){ return _hbridge; }
Sensor1D* ControledMotor::getCurrentSensor(){ return _current_sensor; }
Sensor1D* ControledMotor::getVelocitySensor(){ return _velocity_sensor; }
Sensor1D* ControledMotor::getPositionSensor(){ return _position_sensor; }

int ControledMotor::getVoltage(){ return _hbridge->getBridgeVoltage(); }

int ControledMotor::getCurrent()
{
	if(_current_sensor != NULL) 
		return _current_sensor->peekCalibratedValue(); 
	else
		return 0;
}

int ControledMotor::getVelocity()
{
	if(_velocity_sensor != NULL) 
		return _velocity_sensor->peekCalibratedValue(); 
	else
		return 0;
}

int ControledMotor::getPosition()
{
	if(_position_sensor != NULL) 
		return _position_sensor->peekCalibratedValue(); 
	else
		return 0;
}

int ControledMotor::getRefCurrent(){ return _ref_current; }
int ControledMotor::getRefVelocity(){ return _ref_velocity; }
int ControledMotor::getRefPosition(){ return _ref_position; }

int ControledMotor::getReference()
{
	switch(_mode){
		case CONTROLEDMOTOR_MODE_CUR:
			return _ref_current;
		case CONTROLEDMOTOR_MODE_SPD:
			return _ref_velocity;
		case CONTROLEDMOTOR_MODE_POS:
			return _ref_position;
		default:
			return 0;
	}
}

int ControledMotor::getErrorCurrent(){ return _error_current; }
int ControledMotor::getErrorVelocity(){ return _error_velocity; }
int ControledMotor::getErrorPosition(){ return _error_position; }
