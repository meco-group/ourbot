#include "ourbot.h"
#include "difference_sensor.h"

Ourbot::Ourbot(OurbotHAL* hal, Adafruit_LSM9DS0* imu1, Adafruit_LSM9DS0* imu2, uint32_t mode):
    _mode(mode),
    _controledMotors{ControledMotor(hal->getMotor(0),0.001f,hal->getCurrentSensor(0),new DifferenceSensor(hal->getEncoder(0),0,0.02f),hal->getEncoder(0)), ControledMotor(hal->getMotor(1),0.001f,hal->getCurrentSensor(1),new DifferenceSensor(hal->getEncoder(1),0,0.02f),hal->getEncoder(1)), ControledMotor(hal->getMotor(2),0.001f,hal->getCurrentSensor(2),new DifferenceSensor(hal->getEncoder(2),0,0.02f),hal->getEncoder(2)), ControledMotor(hal->getMotor(3),0.001f,hal->getCurrentSensor(3),new DifferenceSensor(hal->getEncoder(3),0,0.02f),hal->getEncoder(3))},
    _imus{imu1,imu2}
{
    _ID = EEPROM_M::robot_id();
    for(uint8_t k = 0; k<4; k++)
    	_controledMotors[k].setLoopEnable(VELOCITY_LOOP_ENABLE);
}

Ourbot::Ourbot(	HBridgeInterface* motor0, HBridgeInterface* motor1, HBridgeInterface* motor2, HBridgeInterface* motor3,
    	   		Sensor1D* current_sensor0, Sensor1D* current_sensor1, Sensor1D* current_sensor2, Sensor1D* current_sensor3,
    	   		Sensor1D* encoder0, Sensor1D* encoder1, Sensor1D* encoder2, Sensor1D* encoder3,
    	   		Adafruit_LSM9DS0* imu1, Adafruit_LSM9DS0* imu2, uint32_t mode):
    _mode(mode),
    _controledMotors{ControledMotor(motor0,0.001f,current_sensor0,new DifferenceSensor(encoder0,0,0.02f),encoder0), ControledMotor(motor1,0.001f,current_sensor1,new DifferenceSensor(encoder1,0,0.02f),encoder1), ControledMotor(motor2,0.001f,current_sensor2,new DifferenceSensor(encoder2,0,0.02f),encoder2), ControledMotor(motor3,0.001f,current_sensor3,new DifferenceSensor(encoder3,0,0.02f),encoder3)},
    _imus{imu1,imu2}
{
    _ID = EEPROM_M::robot_id();
    for(uint8_t k = 0; k<4; k++)
    	_controledMotors[k].setLoopEnable(VELOCITY_LOOP_ENABLE);
}

void Ourbot::init(){
	// only do ourbot initialization here: other hardware should already be initialized
}

void Ourbot::controllerHook(){
	_controledMotors[0].update();
	_controledMotors[1].update();
	_controledMotors[2].update();
	_controledMotors[3].update();
}

void Ourbot::sensorHook(){
	_imus[0]->read();
	_imus[1]->read();
}

uint8_t Ourbot::setID(uint8_t ID){
    _ID = EEPROM_M::set_robot_id(ID);
    return _ID;
}

uint32_t Ourbot::setMode(uint32_t mode){
	_mode = mode;
	return _mode;
}

uint8_t Ourbot::ID()
{
    return _ID;
}

const uint8_t Ourbot::type()
{
	return 10; //the car type
}

uint32_t Ourbot::mode()
{
	return _mode;
}

ControledMotor* Ourbot::controledMotorAR(uint8_t n)
{
    return (&_controledMotors[n]);
}

ControledMotor* Ourbot::controledMotorID(uint8_t n)
{
    uint8_t k = 0;
    while((k<3)&&(_controledMotors[k].getHBridge()->ID()!=n)){ k++; }
    if(k==3){ k=0; }
    return (&_controledMotors[k]);
}

Adafruit_LSM9DS0* Ourbot::imuAR(uint8_t n)
{
	return _imus[n];
}
