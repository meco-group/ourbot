#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include "mavlink_communicator.h"
#include "ourbot.h"

#define NUM_UPDATE_MESSAGES 4

class Communicator : public MavlinkCommunicator
{
private:
	uint8_t _auto_transmission_message;
	const uint8_t _auto_transmission_messages[NUM_UPDATE_MESSAGES];

	Ourbot* _ourbot;
	
	void sendMessage(uint8_t msgID);
	bool handleMessage(mavlink_message_t &msg);

public:
	Communicator(Ourbot* ourbot, HALBase* hal);

	void transmit();

    void sendMotorData();
    void sendIMUData();
};

#endif //COMMUNICATOR_H
