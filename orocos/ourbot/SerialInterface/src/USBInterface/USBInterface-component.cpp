#include "USBInterface-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

USBInterface::USBInterface(std::string const& name) :
	SerialInterface(name), _usb_port_name(std::string("/dev/ttyACM0")), _usb_fd(0)
{
	addProperty("_usb_port_name",_usb_port_name).doc("The usb port name.");

	addOperation("setUSBPortName", &USBInterface::setUSBPortName, this).doc("Set the _usb_port_name property.");
	addOperation("isConnectedSerial", &USBInterface::isConnectedSerial, this).doc("Returns true if the serial connection is ok.");
	addOperation("available", &USBInterface::available, this).doc("Returns the amount of available characters in the buffer.");
}

bool USBInterface::configureHook()
{
#ifndef USBINTERFACE_TESTFLAG 
	//do nothing
#else
	setUSBPortName("/dev/ttyACM0");	
	return setPeriod(0.01);
#endif //STANDALONE
}

bool USBInterface::startHook()
{
	if(connectSerial()){
		return true;
	}else{
		RTT::log(RTT::Warning) << "Cannot connect to the USB device (" + getName() + ")" << RTT::endlog();
		return false;
	}
}

void USBInterface::updateHook()
{
#ifndef USBINTERFACE_TESTFLAG
	//do nothing
#else
	uint8_t bytes[100];
	int numbytes = readBytes(bytes, 100);
	if(numbytes > 0){
		std::cout << std::string((char*)bytes,numbytes);
	}
#endif //STANDALONE
}

void USBInterface::stopHook()
{
	disconnectSerial();
}

void USBInterface::setUSBPortName(std::string usb_port_name)
{
	_usb_port_name = usb_port_name;
}

bool USBInterface::connectSerial()
{
  _usb_fd = open(_usb_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_TRUNC);

  if (_usb_fd > 0)
	{
		struct termios options, oldopt;
		tcgetattr(_usb_fd, &oldopt);
		bzero(&options,sizeof(struct termios));

		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		
		// enable rx and tx
		options.c_cflag |= (CLOCAL | CREAD);

		options.c_cflag &= ~PARENB; //no checkbit
		options.c_cflag &= ~CSTOPB; //1bit stop bit
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8; /* Select 8 data bits */
	#ifdef CNEW_RTSCTS
		options.c_cflag &= ~CNEW_RTSCTS; // no hw flow control
	#endif
	
		options.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input mode 
		options.c_oflag &= ~OPOST; // raw output mode 
		
		tcflush(_usb_fd,TCIFLUSH); 

		if (fcntl(_usb_fd, F_SETFL, FNDELAY))
		{
		    disconnectSerial();
		    return false;
		}
		if (tcsetattr(_usb_fd, TCSANOW, &options))
		{
		    disconnectSerial();
		    return false;
		}

		//Clear the DTR bit to let the motor spin
		uint32_t controll = TIOCM_DTR;
		ioctl(_usb_fd, TIOCMBIC, &controll);
  
  	return true;
  } else {
		return false;
  }
}

bool USBInterface::disconnectSerial()
{
	if(_usb_fd > 0){
		if(close(_usb_fd) == 0){
			_usb_fd = 0;
		} else {
			std::cout << "Error while closing USB port." << std::endl;
		}
	}	else {
		std::cout << "USB port was not yet connected." << std::endl;
	}
	
	return (_usb_fd == 0);
}

bool USBInterface::isConnectedSerial()
{
	return (_usb_fd > 0);
}

int USBInterface::available()
{
	uint32_t bytes_available;
	ioctl(_usb_fd, FIONREAD, &bytes_available);
	
	return bytes_available;
}

int USBInterface::readByte(uint8_t* byte, uint8_t port_select)
{
	return readBytes(byte, 1);	
}

int USBInterface::readBytes(uint8_t* bytes, uint32_t length, uint8_t port_select)
{	
	uint32_t bytes_received = read(_usb_fd, bytes, length);  
  if(bytes_received == -1){
  	std::cout << "Error while reading file." << std::endl;
  	bytes_received = 0;
  }
  
  _bytes_read += bytes_received;
  return bytes_received;
}

int USBInterface::writeByte(uint8_t byte, uint8_t port_select)
{
	return writeBytes(&byte, 1);
}

int USBInterface::writeBytes(uint8_t* bytes, uint32_t length, uint8_t port_select)
{	
	uint32_t bytes_sent = 0;
	uint32_t bytes_written = 0;
	
	if((bytes != NULL) && (length != 0)){ //check if the buffer and length are not zero
		do {	//loop writing bytes to the serial port - prevents loss of bytes by looping
		    bytes_written = write(_usb_fd, bytes+bytes_sent, length-bytes_sent);
		    if(bytes_written == -1){
		    	return bytes_sent;
		    }
		    
		    bytes_sent += bytes_written;
		}while(bytes_sent<length);
	} else {
		std::cout << "Invalid settings for the write function: NULL pointer array or 0 length." << std::endl;
	}
	
	_bytes_written += bytes_sent;
  return bytes_sent;
}

#ifdef USBINTERFACE_TESTFLAG
ORO_CREATE_COMPONENT(USBInterface);
#endif //USBINTERFACE_TESTFLAG
