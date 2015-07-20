#ifndef TEENSYBRIDGE_H
#define TEENSYBRIDGE_H

//#define TEENSYBRIDGE_TESTFLAG
//#define TEENSYBRIDGE_DEBUGFLAG

#ifdef TEENSYBRIDGE_DEBUGFLAG
	#define TEENSYBRIDGE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define TEENSYBRIDGE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#include "../USBInterface/USBInterface-component.hpp"
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <vector>
#include "protocol_mavlink.h"

#define TEENSYBRIDGE_CONTROL_MODE_SIMPLE			0
#define TEENSYBRIDGE_CONTROL_MODE_INDIVIDUAL	1

class TeensyBridge : public USBInterface
{
	private:
		ProtocolMavlink	_protocol;
		mavlink_motor_state_t	_motor_states[4];
		mavlink_debug_t				_debug;
		mavlink_threadtime_t	_threadtime;

		double _platform_length;
		double _platform_width;
		double _wheel_radius;
		uint32_t _encoder_ticks_per_revolution;
		double _current_sensor_gain;
		int 	 _current_sensor_offset;

		double _kinematic_conversion_position;
		double _kinematic_conversion_orientation;
		double _kinematic_conversion_wheel;
		std::vector<double>	_pose;

		uint8_t _control_mode;
		double _velocity_controller_P;
		double _velocity_controller_I;
		double _velocity_controller_D;

		RTT::InputPort<std::vector<double> >	_cmd_velocity_port;
		RTT::OutputPort<std::vector<double> > _cal_enc_pose_port;
		RTT::OutputPort<std::vector<double> > _cal_velocity_port;
		RTT::OutputPort<std::vector<double> > _raw_enc_ticks_port;
		RTT::OutputPort<std::vector<double> > _raw_enc_speed_port;
		RTT::OutputPort<std::vector<double> > _cal_motor_voltage_port;
		RTT::OutputPort<std::vector<double> > _cal_motor_current_port;
		RTT::OutputPort<std::vector<double> > _debug_port;

		bool action(mavlink_message_t msg);
		void setMotorReference(int reference, int ID, int mode);
		void setController(uint8_t controllerID, float P, float I, float D);

		void recalculatePose();
		void writeRawDataToPorts();

	public:
		TeensyBridge(std::string const& name);

		bool configureHook();
		bool startHook();
		void updateHook();

		uint32_t getPacketsDropped();
		uint32_t getPacketsReceived();
		std::vector<double> getPose();

		//Commands
		void setControlMode(uint8_t control_mode);
		void setMotorVelocity(double rpm, int ID);
		void setMotorCurrent(double current, int ID);
		void setMotorVoltage(double voltage, int ID);
		void setVelocity(double vx, double vy, double w);
		void setCurrentController(float P, float I, float D);
		void setVelocityController(float P, float I, float D);
		void showCurrents();
		void showVelocities();
		void showEncoders();
		void showPositions();
		void showMotorState(int ID);
		void showDebug();
		void showThreadTime();


};

#endif //TEENSYBRIDGE_H
