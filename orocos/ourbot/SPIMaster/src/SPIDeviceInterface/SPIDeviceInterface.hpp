#ifndef SPI_DEVICE_INTERFACE_HPP
#define SPI_DEVICE_INTERFACE_HPP

//TEST:
// #define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0])) 

// #define SPIDEVICEINTERFACE_DEBUGFLAG

#ifdef SPIDEVICEINTERFACE_DEBUGFLAG
	#define SPIDEVICEINTERFACE_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define SPIDEVICEINTERFACE_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

//For SPI
#include <stdint.h>                                                             
#include <stdio.h>                                                              
#include <stdlib.h>                                                             
#include <fcntl.h>                                                              
#include <sys/ioctl.h>                                                          
#include <linux/spi/spidev.h>

//For Orocos
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

//Help files
#include "gpio.h" //file to (un)export GPIO pins to make them available to steer
#include <errno.h> //error logging

using namespace RTT; //avoids that you have to use RTT::InputPort etc.

class SPIDeviceInterface : public RTT::TaskContext{

//Purely virtual class, no constructor needed, but still allowed to make one
public:
	SPIDeviceInterface(std::string const& name); 
	bool startHook();
	void updateHook();

private:
	InputPort <int> 	_fd_port;//file descriptor output port
	InputPort <int> 	_spi_mode_port;
	InputPort <int> 	_spi_word_length_port;
	InputPort <int>     _spi_speed_port;

	//Todo: make _fd private? since IMU doesn't need it

protected:  
	//define class variables:
	int	     		_fd; 			//file descriptor (SPI bus representation)
	const char*	_device;	//pointer to location of SPI device on Odroid
		                       
    int  	_mode; 		//SPI mode = 0 (0...3)                                                           
    int  	_bits; 		//bit per word = 8;                                                        
    int 	_speed; 	//= 500000; //500kHz 

    int _errno; //holds error number  

		//define methods:
	void 			 	pabort(const char *s); //error handling
	void				transfer  (uint8_t reg, uint8_t* tx, uint8_t bytes, uint8_t* buffer); //transfer an array of bytes. Used to read and write. Specify address to read to/write from in reg.
	void 				writeByte (uint8_t cs,  uint8_t reg, uint8_t message); //specify slave with cs
	uint8_t 			readByte  (uint8_t cs,  uint8_t reg); //read 1 byte
	void 				writeBytes(uint8_t cs,  uint8_t reg, uint8_t* message, uint32_t numBytes, uint8_t* buffer); //use pointer to an array of bits which makes up the message
	void				readBytes (uint8_t cs,  uint8_t reg, uint8_t numBytes, uint8_t* buffer); //read n bytes

	void  				initGPIO(uint8_t GPIO); 		//initialize GPIO as output and value 'high'
	void  				cleanupGPIO(uint8_t GPIO);	//clean up the GPIO to reset all connections if application is stopped

	//TEST: 
	// void 					transfer(); //purely included to test if basic transfer works


	//This data is transferred by the SPIMaster over a port, but also accessible via these functions 
	int 					getFileDescriptor();
	int 					getSPIMode();   //set mode 0,1,2 or 3
	int 					getSPIWordLength(); 	//set bits per word 
	int 					getSPISpeed();//set communication speed				
	
	virtual void	init() = 0; 							//initialize the SPI component
	virtual void  	updateMeasurements() = 0;	//update SPI measurements
	virtual bool 	isConnected() = 0; 				//check ID of SPI component
	//Note: when there is a =0 behind a function somewhere this means that the class will be purely virtual (impossible to make an instance of it)
};
#endif //SPI_DEVICE_INTERFACE