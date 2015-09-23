#include "SPIMaster-component.hpp"
#include <iostream>


SPIMaster::SPIMaster(std::string const& name) : TaskContext(name, PreOperational), _fd(-1){


#ifndef SPIMASTER_TESTFLAG
  //Add properties
	addProperty("device", _device).doc("SPI device driver");
  addProperty("mode",   _mode).doc  ("SPI-mode (0...3)");
  addProperty("bits",   _bits).doc  ("Number of bits per word");
  addProperty("speed",  _speed).doc ("SPI communication speed");

#else //if in Test-mode
  _device = "/dev/spidev1.0"; //locate SPI device on Odroid
  _mode = 0; //based on IMU datasheet
  _bits = 8;
  _speed = 500000; //MAX: 1 000 000, depends on pull-up resistors, the higher these are how lower max speed is 

#endif //SPIMASTER_TESTFLAG

  ports()->addPort("fd_port",              _fd_port).doc("File descriptor output port");
  ports()->addPort("spi_mode_port",        _spi_mode_port).doc("SPI mode output port");
  ports()->addPort("spi_speed_port",       _spi_speed_port).doc("SPI speed output port");
  ports()->addPort("spi_word_length_port", _spi_word_length_port).doc("SPI word length output port");

  SPIMASTER_DEBUG_PRINT("SPIMaster constructed!")
}

bool SPIMaster::configureHook(){

	// Show example data sample to output ports to make data flow real-time
  int example = 0;
  
  _fd_port.setDataSample(example);
  _spi_mode_port.setDataSample(example);
  _spi_speed_port.setDataSample(example);
  _spi_word_length_port.setDataSample(example);


  SPIMASTER_DEBUG_PRINT("SPIMaster configured!")
  return true;
}

bool SPIMaster::startHook(){
  //Open the SPI bus
  _fd = open(_device.c_str(), O_RDWR); //SPI bus is represented by file descriptor (_fd). _fd points to the SPI device.
  	if (_fd < 0){              //_device = SPI port on Odroid , O_RDWR request a read/write permission.                                                                                         
      pabort("can't open device");
      return false;
    } 

  //Write _fd to output port
 	_fd_port.write(_fd); 
	//Write SPI info to output ports
 	_spi_word_length_port.write(_bits);
 	_spi_mode_port.write(_mode);
 	_spi_speed_port.write(_speed);

 	//Output SPI data to user
 	SPIMASTER_DEBUG_PRINT( "The file descriptor in SPIMaster: "<<_fd)
 	SPIMASTER_DEBUG_PRINT( "SPI mode: " << (int)_mode )
 	SPIMASTER_DEBUG_PRINT( "SPI speed: " << (int)_speed )
 	SPIMASTER_DEBUG_PRINT( "SPI word length: " << (int)_bits )

  //Assign SPI properties to the SPI device
	setSPIMode();
	setSPISpeed();
	setSPIWordLength();

  SPIMASTER_DEBUG_PRINT("SPIMaster started!")  
  return true;
}

void SPIMaster::updateHook(){

  SPIMASTER_DEBUG_PRINT("SPIMaster executes updateHook !")
}

void SPIMaster::stopHook() {
	SPIMASTER_DEBUG_PRINT("Closing the file descriptor")
  close(_fd); //Close SPI bus/file descriptor
  _fd = -1;   //Restore _fd to default value

  //TEST: check if _fd is really gone
  //Write _fd to output port
  //_fd_port.write(_fd); 
  SPIMASTER_DEBUG_PRINT("SPIMaster stopped")
}

void SPIMaster::cleanupHook() {
	SPIMASTER_DEBUG_PRINT("SPIMaster cleaning up !")
}
  
//Define other functions  

void	SPIMaster::pabort(const char *s){   //Catch errors                                                                            
  perror(s);                                                              
  abort();                                                                
}  

void	SPIMaster::setSPIMode(){
	int ret = ioctl(_fd, SPI_IOC_WR_MODE, &_mode);  //write some mode: 0...3, limited to 8 bits                              
	if (ret == -1){
    pabort("can't set spi mode");   
  }                                                                                        		
}
     
void	SPIMaster::setSPIWordLength(){
	int ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &_bits); //set SPI device word length (1..N)                    
	if (ret == -1){
    pabort("can't set bits per word");    
  }                                                          	                                                                                       
}

void	SPIMaster::setSPISpeed(){
	int ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed); //set SPI device default max speed hz                      
	if (ret == -1){
    pabort("can't set max speed hz");
  }                                                          
}

//TEST: see if the SPI-settings were successfully made
// uint8_t* SPIMaster::getSPIMode(){
//   int ret = ioctl(_fd, SPI_IOC_RD_MODE, &_mode);  //read the mode: 0...3, limited to 8 bits
//   if (ret == -1){
//     pabort("can't get spi mode");  
//   }                                                          
//   return &_mode; 
// }    

// uint8_t* SPIMaster::getSPIWordLength(){
//   int ret = ioctl(_fd, SPI_IOC_RD_BITS_PER_WORD, &_bits); //get SPI device word length (1..N)                      
//   if (ret == -1){
//     pabort("can't get bits per word");   
//   }                                                                
//   return &_bits;
// }

// uint32_t* SPIMaster::getSPISpeed(){
//   int ret = ioctl(_fd, SPI_IOC_RD_MAX_SPEED_HZ, &_speed); //get SPI device default max speed hz                      
//   if (ret == -1){
//     pabort("can't get max speed hz");  
//   }                                                          
//   return &_speed; 
// }

// int* SPIMaster::getFileDescriptor(){
// 	return &_fd;	
// }   

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(SPIMaster)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
 ORO_CREATE_COMPONENT_LIBRARY()
 ORO_LIST_COMPONENT_TYPE(SPIMaster)

//ORO_CREATE_COMPONENT(SPIMaster) //Note: this is not allowed anymore since there is inheritance present
