#ifndef COMMUNICATOR_INTERFACE_H
#define COMMUNICATOR_INTERFACE_H

#include <inttypes.h>

class CommunicatorInterface
{
protected:
	uint32_t _async_flag;
	uint8_t _receive_channel;
	unsigned long _receive_time;
	unsigned long _transmission_time;
	unsigned long _log_time;
	unsigned long _heartbeat_time;
	
public:
	CommunicatorInterface():
		_async_flag(0),
		_receive_channel(0),
		_receive_time(0),
		_transmission_time(0),	
		_log_time(0),
		_heartbeat_time(0)
	{
		//do nothing;
	}
	
	virtual int start() = 0;
	virtual int update() = 0;

	uint32_t asyncFlag(){ return _async_flag; };
	uint32_t checkAsyncFlag(uint32_t async_flag){ return (_async_flag & async_flag); };
	uint32_t setAsyncFlag(uint32_t async_flag){ return _async_flag = (_async_flag | async_flag); };
	uint32_t removeAsyncFlag(uint32_t async_flag){ return _async_flag = (_async_flag & (~async_flag)); };

};

#endif //COMMUNICATOR_INTERFACE_H
