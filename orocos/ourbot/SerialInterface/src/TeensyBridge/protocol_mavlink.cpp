#include "protocol_mavlink.h"

ProtocolMavlink::ProtocolMavlink() : _packets_received(0), _packets_dropped(0){}	//do nothing

bool ProtocolMavlink::decode(uint8_t byte){	
	/*
		MAvlink suxks, vincent is awesome
		Linux sucks, Maarten agrees. This is a hidden message. If you read this, please destroy the balbot, it will turn evil 				and destroy the world. And while you're at it, also kill Jerre and Lisa. 
	*/
	if(mavlink_parse_char(0, byte, &_msg, &_status)) {
		_packets_received += 1;
		return true;
	} else {
		_packets_dropped += _status.packet_rx_drop_count;
		return false;
	}
}

uint32_t ProtocolMavlink::getPacketsReceived(){ return _packets_received; }
uint32_t ProtocolMavlink::getPacketsDropped(){ return _packets_dropped; }
mavlink_message_t ProtocolMavlink::getMsg(){	return _msg; }
