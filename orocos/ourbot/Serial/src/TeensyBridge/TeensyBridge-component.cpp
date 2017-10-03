#include "TeensyBridge-component.hpp"
#include <iostream>
#include <math.h>

#define TEENSYBRIDGE_SERIALBUFFERSIZE	1024
#define MAVLINK_MSG_OVERHEAD			8

TeensyBridge::TeensyBridge(std::string const& name) :
	USBInterface(name), _platform_length(0.3), _platform_width(0.265), _wheel_radius(0.05), _encoder_ticks_per_revolution(35840),
	_current_sensor_gain(1.0), _current_sensor_offset(0.0),
	_kinematic_conversion_position(0.0), _kinematic_conversion_orientation(0.0), _kinematic_conversion_wheel(0.0), _pose(3,0.0), _velocity(3,0.0),
	_control_mode(TEENSYBRIDGE_CONTROL_MODE_SIMPLE), _velocity_pid_soft(3, 0.0), _velocity_pid_strong(3, 0.0),
	_imus{new IMU(std::string("left_imu")), new IMU(std::string("right_imu"))}
{
	// SETUP KINEMATICS
	this->addProperty("platform_length",_platform_length).doc("Property containing the platform length in [m].");
	this->addProperty("platform_width",_platform_width).doc("Property containing the platform width in [m].");
	this->addProperty("wheel_radius",_wheel_radius).doc("Property containing the wheel radius in [m].");
	this->addProperty("encoder_ticks_per_revolution",_encoder_ticks_per_revolution).doc("Property containing the encoder ticks per revolution in [-].");
	this->addProperty("current_sensor_gain",_current_sensor_gain).doc("Gain of current sensor to get proper scaling");
	this->addProperty("velocity_pid_soft", _velocity_pid_soft).doc("PID parameters for soft velocity control (gamepad).");
	this->addProperty("velocity_pid_strong", _velocity_pid_strong).doc("PID parameters for strong velocity control (smooth trajectories).");
	this->addProperty("max_velocity", _max_velocity).doc("Max velocity setpoint");

	this->ports()->addPort( "cmd_velocity_port", _cmd_velocity_port ).doc("Input port for low level velocity controller. Vector contains [vx,vy,w]");
	this->ports()->addPort( "cmd_velocity_passthrough_port", _cmd_velocity_passthrough_port ).doc("Passthrough for cmd velocity port.");
	this->ports()->addPort( "cal_enc_pose_port", _cal_enc_pose_port ).doc( "Output port for calibrated encoder values. Outputs the pose (x,y,orientation) in [m,m,rad]" );
	this->ports()->addPort( "cal_velocity_port", _cal_velocity_port ).doc( "Output port for calibrated encoder velocity values. Outputs the velocity (Dx,Dy,Dorientation) in [m/s,m/s,rad/s]" );
	this->ports()->addPort( "raw_enc_ticks_port", _raw_enc_ticks_port ).doc( "Output port for raw encoder values. Outputs the raw encoder values (FL,FR,RL,RR) in [ticks]" );
	this->ports()->addPort( "raw_enc_speed_port", _raw_enc_speed_port ).doc( "Output port for raw encoder speed values. Outputs the raw encoder speed values (FL,FR,RL,RR) in [ticks/s]" );
	this->ports()->addPort( "raw_enc_cmd_speed_port", _raw_enc_cmd_speed_port ).doc("Output port for low level velocity controller reference speed. Vector contains [FL,FR,RL,RR] in [ticks/s]");
	this->ports()->addPort( "cal_motor_voltage_port", _cal_motor_voltage_port ).doc( "Output port for the motor voltage values. (FL,FR,RL,RR) in [V]" );
	this->ports()->addPort( "cal_motor_current_port", _cal_motor_current_port ).doc( "Output port for the calibrated motor current values. (FL,FR,RL,RR) in [A]" );
	this->ports()->addPort( "debug_port", _debug_port ).doc( "Debug variables port" );
	this->ports()->addPort( "cal_trailer_angle_port", _cal_trailer_angle_port).doc( "Output port for calibrated trailer angle (degrees)");

	addOperation("getPacketsDropped", &TeensyBridge::getPacketsDropped, this).doc("Returns the amount of dropped packets since the beginning.");
	addOperation("getPacketsReceived", &TeensyBridge::getPacketsReceived, this).doc("Returns the amount of received packets since the beginning.");
	addOperation("getPose", &TeensyBridge::getPose, this).doc("Returns the pose of the platform. (x,y,orientation) in [m,m,rad]");
	addOperation("setControlMode", &TeensyBridge::setControlMode, this).doc("Set the control mode. 0 for simple, 1 for individual wheel control.").arg("control_mode","Controller mode: 0 = simple, 1 = individual");
	addOperation("setMotorVoltage", &TeensyBridge::setMotorVoltage, this).doc("Set the motor voltage [V] of the motor with the specified ID. (Only possible in individual mode)").arg("voltage","voltage [V]").arg("ID","motor ID");
	addOperation("setMotorCurrent", &TeensyBridge::setMotorCurrent, this).doc("Set the motor current [A] of the motor with the specified ID. (Only possible in individual mode)").arg("current","current [A]").arg("ID","motor ID");
	addOperation("setMotorVelocity", &TeensyBridge::setMotorVelocity, this).doc("Set the motor velocity [rpm] of the motor with the specified ID. (Only possible in individual mode)").arg("rpm","velocity [rpm]").arg("ID","motor ID");;
	addOperation("setVelocity", &TeensyBridge::setVelocity, this).doc("Set the velocity of the ourbot, vx, vy in m/s, w in rad/s").arg("vx","vx [m/s]").arg("vy","vy [m/s]").arg("w","w [rad/s]");
	addOperation("setCurrentController", &TeensyBridge::setCurrentController, this).doc("Set the pid parameters for the motor current controller.").arg("P","P").arg("I","I").arg("D","D");
	addOperation("setVelocityController", &TeensyBridge::setVelocityController, this).doc("Set the pid parameters for the motor velocity controller.").arg("P","P").arg("I","I").arg("D","D");
	addOperation("softVelocityControl", &TeensyBridge::softVelocityControl, this).doc("Set soft pid velocity controller.");
	addOperation("strongVelocityControl", &TeensyBridge::strongVelocityControl, this).doc("Set strong pid velocity controller.");
	addOperation("showCurrents", &TeensyBridge::showCurrents, this).doc("Displays the motor currents.");
	addOperation("showVelocities", &TeensyBridge::showVelocities, this).doc("Displays the motor velocity.");
	addOperation("showEncoders", &TeensyBridge::showEncoders, this).doc("Displays the motor encoder ticks.");
	addOperation("showPositions", &TeensyBridge::showPositions, this).doc("Displays the motor position.");
	addOperation("showMotorState", &TeensyBridge::showMotorState, this).doc("Displays the motor state.");
	addOperation("showDebug", &TeensyBridge::showDebug, this).doc("Displays the ourbot debug variables.");
	addOperation("showThreadTime", &TeensyBridge::showThreadTime, this).doc("Displays the ourbot onboard thread times.");
	addOperation("showIMUData", &TeensyBridge::showIMUData, this).doc("Displays the ourbot onboard thread times.").arg("ID","ID of the sensor");
	addOperation("showTrailerAngle", &TeensyBridge::showTrailerAngle, this).doc("Display trailer angle.");

	// SETUP IMU
	addIMUPorts(_imus[0], std::string("l"), 7);
	addIMUPorts(_imus[1], std::string("r"), 7);

	addProperty("imul_acc_offset", _imus[0]->_acc._offset).doc("User defined offset of accelerometer");
	addProperty("imul_gyr_offset", _imus[0]->_gyr._offset).doc("User defined offset of gyroscope");
	addProperty("imul_mag_offset", _imus[0]->_mag._offset).doc("User defined offset of magnetometer");
	addProperty("imur_acc_offset", _imus[1]->_acc._offset).doc("User defined offset of accelerometer");
	addProperty("imur_gyr_offset", _imus[1]->_gyr._offset).doc("User defined offset of gyroscope");
	addProperty("imur_mag_offset", _imus[1]->_mag._offset).doc("User defined offset of magnetometer");

	addProperty("imul_acc_scale", _imus[0]->_acc._scale).doc("User defined scale of accelerometer");
	addProperty("imul_gyr_scale", _imus[0]->_gyr._scale).doc("User defined scale of gyroscope");
	addProperty("imul_mag_scale", _imus[0]->_mag._scale).doc("User defined scale of magnetometer");
	addProperty("imur_acc_scale", _imus[1]->_acc._scale).doc("User defined scale of accelerometer");
	addProperty("imur_gyr_scale", _imus[1]->_gyr._scale).doc("User defined scale of gyroscope");
	addProperty("imur_mag_scale", _imus[1]->_mag._scale).doc("User defined scale of magnetometer");
}

bool TeensyBridge::action(mavlink_message_t msg)
{
	bool message_handled = true;
	switch(msg.msgid){
		case MAVLINK_MSG_ID_MOTOR_STATE:{
			mavlink_motor_state_t motor_state;
			mavlink_msg_motor_state_decode(&msg, &motor_state); // can be done more efficiently
			_motor_states[msg.compid] = motor_state;

			recalculatePose();
			recalculateVelocity();
			writeRawDataToPorts();
			TEENSYBRIDGE_DEBUG_PRINT("Motor State message received.")
			break;}

		case MAVLINK_MSG_ID_RAW_IMU_DATA:{
			mavlink_raw_imu_data_t raw_imu_data;
			mavlink_msg_raw_imu_data_decode(&msg, _raw_imu_data + msg.compid); // decode raw imu data

			updateIMU(msg.compid);
			TEENSYBRIDGE_DEBUG_PRINT("IMU raw data message received.")
			break;}

		/*case MAVLINK_MSG_ID_THREADTIME:{
			mavlink_msg_threadtime_decode(&msg, &_threadtime);
			TEENSYBRIDGE_DEBUG_PRINT("Threadtime message received.")
			break;}*/

		case MAVLINK_MSG_ID_HEARTBEAT:{
			TEENSYBRIDGE_DEBUG_PRINT("Heartbeat message received.")
			break; }

		case MAVLINK_MSG_ID_GPIO:{
			mavlink_msg_gpio_decode(&msg, &_gpio);
			TEENSYBRIDGE_DEBUG_PRINT("GPIO message received.")
			break;}

		/*case MAVLINK_MSG_ID_MOTOR_COMMAND:{
			TEENSYBRIDGE_DEBUG_PRINT("Motor Command message received.")
			break;}*/

		default:{
			message_handled = false;
			TEENSYBRIDGE_DEBUG_PRINT("Mavlink message decoded, but no corresponding action found...")
			break;}
	}
	return message_handled;
}

void TeensyBridge::recalculatePose()
{
	// UPDATE 14/07/2015: Make reference frame conform youbot frame
	_pose[0] = (_motor_states[0].position + _motor_states[1].position)*_kinematic_conversion_position;
	_pose[1] =  (-_motor_states[0].position + _motor_states[2].position)*_kinematic_conversion_position;
	// UPDATE 5/11/2015: Average over all wheels so that the rotation is more accurate
	//_pose[2] = (_motor_states[1].position - _motor_states[2].position)*_kinematic_conversion_orientation;
  	_pose[2] = 0.5*((_motor_states[1].position - _motor_states[2].position) + (_motor_states[3].position - _motor_states[0].position))*_kinematic_conversion_orientation;
	_cal_enc_pose_port.write(_pose);
}

void TeensyBridge::recalculateVelocity()
{
	// UPDATE 14/07/2015: Make reference frame conform youbot frame
	_velocity[0] = (_motor_states[0].velocity + _motor_states[1].velocity)*_kinematic_conversion_position;
	_velocity[1] = (-_motor_states[0].velocity + _motor_states[2].velocity)*_kinematic_conversion_position;
	_velocity[2] = (_motor_states[1].velocity - _motor_states[2].velocity)*_kinematic_conversion_orientation;

	_cal_velocity_port.write(_velocity);
}

void TeensyBridge::writeRawDataToPorts()
{
	std::vector<double> enc(4), spd(4), spd_cmd(4), cur(4), volt(4), debug(6);

	for(uint8_t k=0;k<4;k++){
		enc[k] = _motor_states[k].position;
		spd[k] = _motor_states[k].velocity;
		spd_cmd[k] = _motor_states[k].reference;
		cur[k] = _motor_states[k].current;
		volt[k] = (_motor_states[k].FFvoltage + _motor_states[k].FBvoltage)*0.001;
	}

	_raw_enc_ticks_port.write(enc);
	_raw_enc_speed_port.write(spd);
	_raw_enc_cmd_speed_port.write(spd_cmd);
	_cal_motor_voltage_port.write(volt);
	_cal_motor_current_port.write(cur);

	//debug port
	debug[0] = _gpio.gpio_int[0];
	debug[1] = _gpio.gpio_int[1];
	debug[2] = _gpio.gpio_int[2];
	debug[3] = _gpio.gpio_float[0];
	debug[4] = _gpio.gpio_float[1];
	debug[5] = _gpio.gpio_float[2];

	_debug_port.write(debug);

	_cal_trailer_angle_port.write(debug[3]);
}

bool TeensyBridge::configureHook()
{
	// Show example data sample to ports to make data flow real-time
  	std::vector<double> example(3, 0.0);
  	// set 3D ports
  	_cal_enc_pose_port.setDataSample(example);
  	_cmd_velocity_passthrough_port.setDataSample(example);
  	// set 4D ports
  	example.resize(4);
  	_raw_enc_ticks_port.setDataSample(example);
  	_raw_enc_speed_port.setDataSample(example);
  	_raw_enc_cmd_speed_port.setDataSample(example);
  	_cal_motor_voltage_port.setDataSample(example);
  	_cal_motor_current_port.setDataSample(example);
  	// set 6D ports
  	example.resize(6);
  	_debug_port.setDataSample(example);

	//calculate kinematic conversion
  	_kinematic_conversion_position = _wheel_radius*M_PI/_encoder_ticks_per_revolution; //2*R*pi/enc_res
	_kinematic_conversion_orientation = _wheel_radius*M_PI/_encoder_ticks_per_revolution/(_platform_length/2.0 + _platform_width/2.0);
	_kinematic_conversion_wheel = _encoder_ticks_per_revolution/(2.0*M_PI*_wheel_radius);

	//setup imus
	_imus[0]->configure();
	_imus[1]->configure();

#ifndef TEENSYBRIDGE_TESTFLAG
  return true;
#else
	_usb_port_name = "/dev/ttyACM0";

	_imus[0]->_acc._offset[0] = 1250.0;
	_imus[0]->_acc._scale = std::vector<double>(3,0.000613125);

	return setPeriod(0.005);
#endif //TEENSYBRIDGE_TESTFLAG
}

bool TeensyBridge::startHook()
{
	if(USBInterface::startHook()){
		softVelocityControl();
		_imus[0]->start();
		_imus[1]->start();
	}else{
		return false;
	}
	return true;
}

void TeensyBridge::stopHook()
{
	setVelocity(0., 0., 0.);
	USBInterface::stopHook();
}

void TeensyBridge::updateHook()
{
	uint8_t bytes[TEENSYBRIDGE_SERIALBUFFERSIZE];
	int numbytes = readBytes(bytes, TEENSYBRIDGE_SERIALBUFFERSIZE);

	for(int k=0;k<numbytes;k++){
		if(_protocol.decode(bytes[k])){
			action(_protocol.getMsg());
		}
	}

	std::vector<double> cmd_velocity(3);
	// Possible return values are: NoData, OldData and NewData.
	if(_cmd_velocity_port.read(cmd_velocity) == RTT::NewData){
		setVelocity(cmd_velocity[0], cmd_velocity[1], cmd_velocity[2]);
		_cmd_velocity_passthrough_port.write(cmd_velocity);
	}
}

uint32_t TeensyBridge::getPacketsDropped()
{
	return _protocol.getPacketsDropped();
}

uint32_t TeensyBridge::getPacketsReceived()
{
	return _protocol.getPacketsReceived();
}

std::vector<double> TeensyBridge::getPose()
{
	return _pose;
}

std::vector<double> TeensyBridge::getVelocity()
{
	return _velocity;
}

void TeensyBridge::setControlMode(uint8_t control_mode)
{
	if(control_mode <= 1)
		_control_mode = control_mode;
}

void TeensyBridge::setMotorReference(int setpoint, int ID, int mode)
{
	if(_control_mode == TEENSYBRIDGE_CONTROL_MODE_INDIVIDUAL){
		mavlink_motor_command_t motor_command;
		mavlink_message_t msg;
		uint8_t buffer[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN+MAVLINK_MSG_OVERHEAD];
		uint8_t numbytes = 0;

		//set all to zero
		motor_command.command_left_front = 0;
		motor_command.command_right_front = 0;
		motor_command.command_left_rear = 0;
		motor_command.command_right_rear = 0;
		motor_command.command_type = mode;

		switch(ID){
			case 0: motor_command.command_left_front = setpoint; break;
			case 1: motor_command.command_right_front = setpoint; break;
			case 2: motor_command.command_left_rear = setpoint; break;
			case 3: motor_command.command_right_rear = setpoint; break;
			default: motor_command.command_type = 0; break;
		}

		mavlink_msg_motor_command_encode(0,0,&msg,&motor_command);
		numbytes = mavlink_msg_to_send_buffer(buffer, &msg);
		writeBytes(buffer, numbytes);
	}
}

void TeensyBridge::setMotorVelocity(double rpm, int ID)
{
	int velocity_setpoint = rpm*_encoder_ticks_per_revolution/60;
	std::cout << velocity_setpoint << std::endl;
	setMotorReference(velocity_setpoint, ID, 3);
}

void TeensyBridge::setMotorCurrent(double current, int ID)
{
	int current_setpoint = (current + _current_sensor_offset)/_current_sensor_gain;
	setMotorReference(current_setpoint, ID, 2);
}

void TeensyBridge::setMotorVoltage(double voltage, int ID)
{
	int voltage_setpoint = voltage*1000;
	setMotorReference(voltage_setpoint, ID, 1);
}

double TeensyBridge::saturate(double v, double max) {
	if (v > max) {
		RTT::log(RTT::Warning) << "Saturating velocity " << v << RTT::endlog();
		return max;
	}
	if (v < -max) {
		RTT::log(RTT::Warning) << "Saturating velocity " << v << RTT::endlog();
		return -max;
	}
	return v;
}

void TeensyBridge::setVelocity(double vx, double vy, double w)
{
	if(_control_mode == TEENSYBRIDGE_CONTROL_MODE_SIMPLE){
		vx = saturate(vx, _max_velocity[0]);
		vy = saturate(vy, _max_velocity[1]);
		w = saturate(w, _max_velocity[2]);

		mavlink_motor_command_t motor_command;
		mavlink_message_t msg;
		uint8_t buffer[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN+MAVLINK_MSG_OVERHEAD];
		uint8_t numbytes = 0;

		w = (_platform_length + _platform_width)*w/2.0;

		// UPDATE 14/07/2015: To have the same reference frame as the youbot, the y-axis has been changed
		motor_command.command_left_front = (vx - vy - w)*_kinematic_conversion_wheel;
		motor_command.command_right_front = (vx + vy + w)*_kinematic_conversion_wheel;
		motor_command.command_left_rear = (vx + vy - w)*_kinematic_conversion_wheel;
		motor_command.command_right_rear = (vx - vy + w)*_kinematic_conversion_wheel;
		motor_command.command_type = 3;
		mavlink_msg_motor_command_encode(0,0,&msg,&motor_command);
		numbytes = mavlink_msg_to_send_buffer(buffer, &msg);
		writeBytes(buffer, numbytes);
	}
}

void TeensyBridge::setController(uint8_t controllerID, double P, double I, double D)
{
	mavlink_motor_controller_t motor_controller;
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN+MAVLINK_MSG_OVERHEAD];
	uint8_t numbytes = 0;

	motor_controller.P = P;
	motor_controller.I = I;
	motor_controller.D = D;

	mavlink_msg_motor_controller_encode(0,controllerID,&msg,&motor_controller);
	numbytes = mavlink_msg_to_send_buffer(buffer, &msg);
	writeBytes(buffer, numbytes);
}

void TeensyBridge::setCurrentController(double P, double I, double D)
{
	setController(2, P, I, D);
}

void TeensyBridge::setVelocityController(double P, double I, double D)
{
	setController(3, P, I, D);
}

void TeensyBridge::softVelocityControl(){
	setVelocityController(_velocity_pid_soft[0], _velocity_pid_soft[1], _velocity_pid_soft[2]);
}

void TeensyBridge::strongVelocityControl(){
	setVelocityController(_velocity_pid_strong[0], _velocity_pid_strong[1], _velocity_pid_strong[2]);
}

// IMU RELATED FUNCTIONS
void TeensyBridge::updateIMU(uint8_t ID)
{
	std::vector<double> acc = std::vector<double>(3,0.0);
	std::vector<double> gyr = std::vector<double>(3,0.0);
	std::vector<double> mag = std::vector<double>(3,0.0);

	for(uint8_t k=0;k<3;k++){
		 acc[k] = _raw_imu_data[ID].acc[k];
		 gyr[k] = _raw_imu_data[ID].gyro[k];
		 mag[k] = _raw_imu_data[ID].mag[k];
	}

	_imus[ID]->updateMeasurements(acc, gyr, mag);
}

IMU* TeensyBridge::findIMU(uint8_t ID)
{
	uint8_t k;
	for(k=0;k<2;k++){
		if(_imus[k]->getID() == ID){
			break;
		}
	}
	return _imus[k];
}

// DEBUG DISPLAY FUNCTIONS
void TeensyBridge::showCurrents()
{
	for(uint8_t k=0;k<4;k++)
		std::cout << _motor_states[k].current << std::endl;
}

void TeensyBridge::showVelocities()
{
	for(uint8_t k=0;k<4;k++)
		std::cout << _motor_states[k].velocity << std::endl;
}

void TeensyBridge::showEncoders()
{
	for(uint8_t k=0;k<4;k++)
		std::cout << _motor_states[k].position << std::endl;
}

void TeensyBridge::showPositions()
{

}

void TeensyBridge::showMotorState(int ID)
{
	if(ID < 4){
		std::cout << "position = " << _motor_states[ID].position << std::endl; ///< Position of the motor in encoder ticks
		std::cout << "velocity = " << _motor_states[ID].velocity << std::endl; ///< Velocity of the motor in encoder ticks/sec
		std::cout << "acceleration = " << _motor_states[ID].acceleration << std::endl; ///< Acceleration of the motor in encoder ticks/sec2
		std::cout << "current = " << _motor_states[ID].current << std::endl; ///< Armature current [mA]
		std::cout << "reference = " << _motor_states[ID].reference << std::endl; ///< Reference for the controller. Convenience field when interested in closed loop response
		std::cout << "FFvoltage = " << _motor_states[ID].FFvoltage << std::endl; ///< Feedforward armature voltage
		std::cout << "FBvoltage = " << _motor_states[ID].FBvoltage << std::endl; ///< Feedback armature voltage
	}
}

void TeensyBridge::showDebug()
{
	std::cout << "float1 = " << _gpio.gpio_float[0] << std::endl;
	std::cout << "float2 = " << _gpio.gpio_float[1] << std::endl;
	std::cout << "float3 = " << _gpio.gpio_float[2] << std::endl;
	std::cout << "int1 = " << _gpio.gpio_int[0] << std::endl;
	std::cout << "int2 = " << _gpio.gpio_int[1] << std::endl;
	std::cout << "int3 = " << _gpio.gpio_int[2] << std::endl;
}

void TeensyBridge::showTrailerAngle()
{
	std::cout << "Trailer angle = " << _gpio.gpio_float[0] << "deg";
}

void TeensyBridge::showThreadTime()
{
	/*std::cout << "thread 1: " << _threadtime.thread1 << " (" << (double)_threadtime.thread1/_threadtime.time << ")" << std::endl;
	std::cout << "thread 2: " << _threadtime.thread2 << " (" << (double)_threadtime.thread2/_threadtime.time << ")" << std::endl;
	std::cout << "thread 3: " << _threadtime.thread3 << " (" << (double)_threadtime.thread3/_threadtime.time << ")" << std::endl;
	std::cout << "thread 4: " << _threadtime.thread4 << " (" << (double)_threadtime.thread4/_threadtime.time << ")" << std::endl;
	std::cout << "thread 5: " << _threadtime.thread5 << " (" << (double)_threadtime.thread5/_threadtime.time << ")" << std::endl;
	std::cout << "thread 6: " << _threadtime.thread6 << " (" << (double)_threadtime.thread6/_threadtime.time << ")" << std::endl;*/
	std::cout << "Not implemented at the moment.." << std::endl;
}

void TeensyBridge::showIMUData(int ID)
{
	if(ID < 2){
		std::cout << "accel = " << _raw_imu_data[ID].acc[0] << " " << _raw_imu_data[ID].acc[1] << " " << _raw_imu_data[ID].acc[2] << std::endl; ///< IMU raw acceleration
		std::cout << "gyro = " << _raw_imu_data[ID].gyro[0] << " " << _raw_imu_data[ID].gyro[1] << " " << _raw_imu_data[ID].gyro[2] << std::endl; ///< IMU raw gyroscope data
		std::cout << "magn = " << _raw_imu_data[ID].mag[0] << " " << _raw_imu_data[ID].mag[1] << " " << _raw_imu_data[ID].mag[2] << std::endl; ///< IMU raw magnetic data
	}
}

void TeensyBridge::addIMUPorts(TaskContext* comp, const std::string insertion, const int i)
{
	Ports ports = comp->ports()->getPorts();
	for (Ports::iterator port = ports.begin(); port != ports.end() ; ++port) {
		std::string port_name = (*port)->getName();
		port_name.insert(i,insertion);
		if(getPort(port_name) == NULL){
			//Port not yet in use: we can add it to the container components ports
			addPort(port_name, (**port));
			// std::cout << port_name << " added as port." << std::endl;
		} else {
			RTT::log(RTT::Warning) << "Could not add port " + port_name + " because it is already in use." << RTT::endlog();
		}
	}
}

//ORO_CREATE_COMPONENT(TeensyBridge)
ORO_LIST_COMPONENT_TYPE(TeensyBridge)
