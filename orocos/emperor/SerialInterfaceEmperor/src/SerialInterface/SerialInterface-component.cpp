#include "SerialInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

SerialInterface::SerialInterface(std::string const& name) : TaskContext(name,PreOperational), _bytes_read(0), _bytes_written(0){
	addOperation("getBytesRead", &SerialInterface::getBytesRead, this).doc("Return the total of bytes received.");
	addOperation("getBytesWritten", &SerialInterface::getBytesWritten, this).doc("Return the total of bytes sent.");
}

bool SerialInterface::configureHook(){
	return connectSerial();
}

bool SerialInterface::startHook(){
	_bytes_read = 0;
	_bytes_written = 0;
	
  return true;
}

void SerialInterface::updateHook(){
}

void SerialInterface::stopHook() {
	disconnectSerial();
}

void SerialInterface::cleanupHook() {
}

uint32_t SerialInterface::getBytesRead()
{
	return _bytes_read;
}

uint32_t SerialInterface::getBytesWritten()
{
	return _bytes_written;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(SerialInterface)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
//ORO_CREATE_COMPONENT(SerialInterface)
ORO_CREATE_COMPONENT_LIBRARY()
