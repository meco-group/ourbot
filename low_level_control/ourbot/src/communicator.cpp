#include "communicator.h"

Communicator::Communicator(Ourbot *ourbot, HALBase *hal) :
    MavlinkCommunicator(ourbot->ID(),ourbot->type(),hal),
	_auto_transmission_message(0),
	_auto_transmission_messages({MAVLINK_MSG_ID_MOTOR_STATE,MAVLINK_MSG_ID_GPIO,MAVLINK_MSG_ID_RAW_IMU_DATA,MAVLINK_MSG_ID_GPIO}),
	_ourbot(ourbot)
{
    //do nothing
}

void Communicator::transmit()
{
	sendMessage(_auto_transmission_messages[_auto_transmission_message]);
    _auto_transmission_message++;
	if(_auto_transmission_message >= NUM_UPDATE_MESSAGES){ 
        _auto_transmission_message = 0; 
    };
}

void Communicator::sendMessage(uint8_t msgID)
{
	switch(msgID){
	case MAVLINK_MSG_ID_HEARTBEAT:
		sendHeartbeat();
        break;

	case MAVLINK_MSG_ID_MOTOR_STATE:
        sendMotorData();
		break;
		
	case MAVLINK_MSG_ID_RAW_IMU_DATA:
        sendIMUData();
		break;	
		
	case MAVLINK_MSG_ID_GPIO:
        sendGPIO();
		break;		
	}
}

bool Communicator::handleMessage(mavlink_message_t &msg)
{
    bool ret = false;
	switch(msg.msgid){
		case MAVLINK_MSG_ID_MOTOR_CONTROLLER:{
			mavlink_motor_controller_t motor_controller;
			mavlink_msg_motor_controller_decode(&msg, &motor_controller);
		
			switch(msg.compid){
				case 2: 
					for(uint8_t k=0;k<4;k++){ _ourbot->controledMotorAR(k)->setCurrentController(motor_controller.P,motor_controller.I,motor_controller.D); }
					break;
				case 3: 
					for(uint8_t k=0;k<4;k++){ _ourbot->controledMotorAR(k)->setVelocityController(motor_controller.P,motor_controller.I,motor_controller.D); }
					break;
				case 4: 
					for(uint8_t k=0;k<4;k++){ _ourbot->controledMotorAR(k)->setPositionController(motor_controller.P,motor_controller.I,motor_controller.D); }
					break;
			}
            ret = true;            
			break;}
		
		case MAVLINK_MSG_ID_MOTOR_COMMAND:{
			mavlink_motor_command_t motor_command;
			mavlink_msg_motor_command_decode(&msg, &motor_command);		

			_ourbot->controledMotorAR(0)->setReference(motor_command.command_left_front, motor_command.command_type);
			_ourbot->controledMotorAR(1)->setReference(motor_command.command_right_front, motor_command.command_type);
			_ourbot->controledMotorAR(2)->setReference(motor_command.command_left_rear, motor_command.command_type);
			_ourbot->controledMotorAR(3)->setReference(motor_command.command_right_rear, motor_command.command_type);
			
			ret = true;			
            break;}
        default:{
            ret = MavlinkCommunicator::handleMessage(msg);
        }
	}

	return ret;
}

void Communicator::sendMotorData()
{
    mavlink_message_t msg;
	for(int k=0;k<4;k++){
		mavlink_msg_motor_state_pack( _ourbot->ID(), _ourbot->controledMotorAR(k)->getHBridge()->ID(), &msg, 
									_ourbot->controledMotorAR(k)->getPosition(), 
									_ourbot->controledMotorAR(k)->getVelocity(),
									0,// acceleration
									_ourbot->controledMotorAR(k)->getCurrent(),// current
									0,// feedforward voltage
									_ourbot->controledMotorAR(k)->getHBridge()->getBridgeVoltage(), //feedback voltage
									_ourbot->controledMotorAR(k)->getReference());//reference
        MavlinkCommunicator::sendMessage(msg);
	}
}

void Communicator::sendIMUData()
{
    mavlink_message_t msg;
	for(int k=0;k<2;k++){
		int16_t acc[3] = {_ourbot->imuAR(k)->accelData.x, _ourbot->imuAR(k)->accelData.y, _ourbot->imuAR(k)->accelData.z};
		int16_t gyro[3] = {_ourbot->imuAR(k)->gyroData.x, _ourbot->imuAR(k)->gyroData.y, _ourbot->imuAR(k)->gyroData.z};
		int16_t mag[3] = {_ourbot->imuAR(k)->magData.x, _ourbot->imuAR(k)->magData.y, _ourbot->imuAR(k)->magData.z};
		
		mavlink_msg_raw_imu_data_pack( _ourbot->ID(), k, &msg, acc, gyro, mag);//reference
	    MavlinkCommunicator::sendMessage(msg);
	}
}
