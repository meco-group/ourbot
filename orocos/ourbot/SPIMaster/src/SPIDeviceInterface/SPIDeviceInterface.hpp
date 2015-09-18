#ifndef SPI_DEVICE_INTERFACE_HPP
#define SPI_DEVICE_INTERFACE_HPP

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

//For SPI
#include <stdint.h>                                                             
#include <stdio.h>                                                              
#include <stdlib.h>                                                             
#include <fcntl.h>                                                              
#include <sys/ioctl.h>                                                          
#include <linux/spi/spidev.h>

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

//Seems not necessary to include these?
// #include <unistd.h>
// #include <string.h>
// #include <getopt.h>
// #include <linux/types.h>

#include "gpio.h" //file to steer GPIO pins  

using namespace RTT;

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
		InputPort <int>   _spi_speed_port;

		//Todo: make _fd private? since IMU doesn't need it

protected:  
		//define class variables:
		int	     _fd; 			//file descriptor (SPI bus representation)
		const char*	_device;
		// uint8_t		_cs;			//chip select
		// uint8_t   _id;      //device ID
		                       
    int  	_mode; 		//SPI mode (0...3)                                                           
    int  	_bits; 		//bit per word = 8;                                                        
    int 	_speed; 	//= 500000; //500kHz   
 		//uint16_t 	_delay; 	//Todo: necessary??
 		//int 			_verbose; //Todo: necessary??

 		//define methods:
		void 				 	pabort(const char *s); 

		void 					transfer(); //purely included to test if basic transfer works
		void		 			transfer  (uint8_t reg, uint8_t* tx, uint8_t bytes, uint8_t* buffer); //Transfer an array of bytes. Used to read and write. Specify address to read to/write from in reg.
		void 					writeByte (uint8_t cs,  uint8_t reg, uint8_t message); //Specify slave with cs
		uint8_t 			readByte  (uint8_t cs,  uint8_t reg);
		void 					writeBytes(uint8_t cs,  uint8_t reg, uint8_t* message, uint32_t numBytes, uint8_t* buffer); //Use pointer to an array of bits which makes up the message
		void					readBytes (uint8_t cs,  uint8_t reg, uint8_t numBytes, uint8_t* buffer);
		// virtual int readbuffer();

		void  				initGPIO(uint8_t GPIO); 		//initialize the GPIO's for the chip select of the SPI device 
		void  				cleanupGPIO(uint8_t GPIO);	//clean up the GPIO's to reset all connections if application is stopped
		// void 					setFileDescriptor(int *file_descriptor);
		// void 					setSPIMode(uint8_t *mode);   //set mode 0,1,2 or 3
		// void 					setSPIWordLength(uint8_t *bits); 	//set bits per word 
		// void 					setSPISpeed(uint32_t *speed);//set communication speed	

		int 					getFileDescriptor();
		int 					getSPIMode();   //set mode 0,1,2 or 3
		int 					getSPIWordLength(); 	//set bits per word 
		int 					getSPISpeed();//set communication speed				
		
		virtual void	init() = 0;
		virtual void  updateMeasurements() = 0;
		virtual bool 	isConnected() = 0; 
		//Note: when there is a =0 behind a function somewhere this means that the class will be purely virtual (impossible to make an instance of it)
};
#endif //SPI_DEVICE_INTERFACE