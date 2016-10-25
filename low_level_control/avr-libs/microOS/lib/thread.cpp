#include "WProgram.h"
#include "thread.h"

Thread::Thread()
{
	//do nothing
}

Thread::Thread(priority_t priority, uint32_t period, int (*Fcn)())
{
	_active = false;
	_ID = 0;
	
	_priority = priority;
	_period = period;
	_scheduling_time = 0;
	_execution_time = 0;

	_pFcn = Fcn;
}

/*void Thread::operator=(const Thread t)
{
	_active = t._active;
	_ID = t._ID;
	
	_priority = t._priority;
	_period = t._period;
	_scheduling_time = t._scheduling_time;
	_execution_time = t._execution_time;
	
	_pFcn = t._pFcn;
}*/

bool Thread::start()
{
	_active = true;
	_scheduling_time = micros();
	return _active;
}

bool Thread::idle()
{
	_active = false;
	return _active;
}

bool Thread::action()
{
	unsigned long start_time;
	if(_active&&((micros() - _scheduling_time) >= _period)){ // 
		_scheduling_time += _period;
		start_time = micros();
		(*_pFcn)();
		_execution_time += (micros()-start_time);
		return true;
	}
	
	return false;
}

void Thread::setID(uint8_t ID)
{
    _ID = ID;
}

uint8_t Thread::getID()
{
	return _ID;
}

priority_t Thread::getPriority()
{
	return _priority;
} 

uint32_t Thread::getWakeupTime()
{
    return (_scheduling_time + _period);
}

unsigned long Thread::getExecutionTime()
{
	return _execution_time;
}
