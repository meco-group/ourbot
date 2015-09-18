#ifndef OROCOS_SPIMASTER_COMPONENT_HPP
#define OROCOS_SPIMASTER_COMPONENT_HPP

#define SPIMASTER_TESTFLAG //to manually set properties while testing

#include <sys/ioctl.h> //For use of ioctl
#include <fcntl.h>    // For O_RDWR, open()...
#include <unistd.h>   // For close()
#include <linux/spi/spidev.h> //For SPI_IOC_WR_BITS_PER_WORD, SPI_IOC_WR_MODE,...

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

using namespace RTT; //otherwise you have to use RTT::OutputPort etc.

class SPIMaster : public RTT::TaskContext{
	
	private:
		OutputPort <int>  _fd_port;//file descriptor output port
		OutputPort <int> 	_spi_mode_port;
		OutputPort <int> 	_spi_word_length_port;
		OutputPort <int>  _spi_speed_port;

		int	     		_fd; 					//file descriptor (SPI bus representation)
		std::string	_device;			//= "/dev/spidev1.0";   //Select the spi-driver (of the odroid) --> 1.0 means master is 1, slave is with CS 0                           
    int  		_mode; 				//SPI mode (0...3) depends on your SPI device                                                          
    int  		_bits; 				//bit per word = 8;                                                        
    int 		_speed; 			//= 500000; //500kHz   

		void setSPIMode();       	//set mode 0,1,2 or 3
		void setSPIWordLength(); 	//set bits per word 
		void setSPISpeed();      	//set communication speed

  public:
    SPIMaster(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void pabort(const char *s);
		
    // uint8_t*  getSPIMode();
		// uint8_t*  getSPIWordLength();
		// uint32_t* getSPISpeed();
		// int* 		  getFileDescriptor(); //pass file descriptor to SPI device 
};
#endif