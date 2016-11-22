#ifndef PROTOCOL_MAVLINK_H
#define PROTOCOL_MAVLINK_H

#include "protocol_interface.h"
#include "mavlink/ourbot_messages/mavlink.h"

class ProtocolMavlink : public ProtocolInterface
{
private:
	mavlink_message_t	_msg;
	mavlink_status_t	_status;
	
	uint32_t					_packets_received;
	uint32_t					_packets_dropped;

public:
	ProtocolMavlink();
	
	bool decode(uint8_t byte);
		
	uint32_t getPacketsReceived();
	uint32_t getPacketsDropped();
	mavlink_message_t getMsg();
};

#endif //PROTOCOL_MAVLINK_H
