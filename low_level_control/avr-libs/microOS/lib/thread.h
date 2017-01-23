#ifndef THREADING_H
#define THREADING_H

//#include "threading_config.h"
//#include <inttypes.h>

enum priority_t{	
	LOWEST,
	BELOWNORMAL,
	NORMAL,
	ABOVENORMAL,
	HIGHEST
};

class Thread
{
private:
	bool 			_active;
	uint8_t 		_ID;
	
	priority_t 		_priority;
	uint32_t 		_period;
	uint32_t	    _scheduling_time;
	unsigned long	_execution_time;
		
	int				(*_pFcn)();
	
public:
	Thread();
	Thread(priority_t priority, uint32_t period, int (*Fcn)());

	//void operator=(const Thread t);

	bool start();
	bool idle();
	bool sleep();
	bool action();
	
	void setID(uint8_t ID);
	
	uint8_t getID();
	priority_t getPriority();
	uint32_t getWakeupTime();
	unsigned long getExecutionTime();

};

#endif //THREADING_H
