#include <iostream>
#include "SPIDeviceInterface.hpp" 

SPIDeviceInterface::SPIDeviceInterface(std::string const& name) : TaskContext(name, PreOperational){

  ports()->addPort( "fd_port",              _fd_port ).doc( "File descriptor input port" );
  ports()->addPort( "spi_mode_port",        _spi_mode_port ).doc( "SPI mode input port" );
  ports()->addPort( "spi_word_length_port", _spi_word_length_port ).doc( "SPI speed input port" );
  ports()->addPort( "spi_speed_port",       _spi_speed_port ).doc( "SPI word length input port" );

  addOperation("getFileDescriptor",&SPIDeviceInterface::getFileDescriptor,this);
  addOperation("getSPIWordLength", &SPIDeviceInterface::getSPIWordLength,this);
  addOperation("getSPISpeed",      &SPIDeviceInterface::getSPISpeed,this);
  addOperation("getSPIMode",       &SPIDeviceInterface::getSPIMode,this);
  addOperation("isConnected",      &SPIDeviceInterface::isConnected,this);

  SPIDEVICEINTERFACE_DEBUG_PRINT("SPIDeviceInterface constructed!")
}

bool SPIDeviceInterface::startHook(){

  SPIDEVICEINTERFACE_DEBUG_PRINT("Entered SPIDeviceInterface startHook!")
  
  //create local variables to hold SPI settings
  int file_descriptor;
  int bits;
  int mode;
  int speed;

  //read in SPI settings from the SPIMaster component
  if (_fd_port.read(file_descriptor) == RTT::NewData )    {std::cout<<"reading fd port in startHook"   <<std::endl; _fd = file_descriptor;}
  if (_spi_word_length_port.read(bits) == RTT::NewData )  {std::cout<<"reading bits port in startHook" <<std::endl; _bits = bits;}
  if (_spi_mode_port.read(mode) == RTT::NewData )         {std::cout<<"reading mode port in startHook" <<std::endl; _mode = mode;}
  if (_spi_speed_port.read(speed) == RTT::NewData )       {std::cout<<"reading speed port in startHook"<<std::endl; _speed = speed;}

  //TEST: extra output to user to ensure data was read from port correctly
  SPIDEVICEINTERFACE_DEBUG_PRINT("fd: "    <<_fd)
  SPIDEVICEINTERFACE_DEBUG_PRINT("bits: "  <<(int)_bits)
  SPIDEVICEINTERFACE_DEBUG_PRINT("mode: "  <<(int)_mode)
  SPIDEVICEINTERFACE_DEBUG_PRINT("speed: " <<(int)_speed)

  SPIDEVICEINTERFACE_DEBUG_PRINT("SPIDeviceInterface started!")
  
  return true;
}

void SPIDeviceInterface::updateHook(){

  SPIDEVICEINTERFACE_DEBUG_PRINT("Entered SPIDeviceInterface updateHook!")
  
  //create local variables to hold SPI settings
  int file_descriptor;
  int bits;
  int mode;
  int speed;

  //update SPI settings (held by the class variables) if they changed (=NewData)
  if (_fd_port.read(file_descriptor) == RTT::NewData )    {std::cout<<"reading fd port in updateHook"   << std::endl; _fd = file_descriptor;}
  if (_spi_word_length_port.read(bits) == RTT::NewData )  {std::cout<<"reading bits port in updateHook" << std::endl; _bits = bits;}
  if (_spi_mode_port.read(mode) == RTT::NewData )         {std::cout<<"reading mode port in updateHook" << std::endl; _mode = mode;}
  if (_spi_speed_port.read(speed) == RTT::NewData )       {std::cout<<"reading speed port in updateHook"<< std::endl; _speed = speed;}

  SPIDEVICEINTERFACE_DEBUG_PRINT("SPIDeviceInterface updates information!")  
}

void SPIDeviceInterface::pabort(const char *s){   //error catching                                                                            
  perror(s);                                                              
  abort();                                                                
}  

//TEST: if it is possible to send a single SPI message. Tells you if the settings are made and file descriptor is correct. Based on Odroid example code
// void SPIDeviceInterface::transfer()
// {
//   //Function transfer with no arguments
//   //Purely meant to test if it possible to send an SPI message

//   int ret;
//   uint8_t tx1[] = {
//     0xAB, 0x00, 0x00, 0x00
//   };
//   uint8_t tx2[] = {
//     0x00, 0x00, 0x00, 0x00
//   };

//   uint8_t rx1[ARRAY_SIZE(tx1)] = {0, };
//   uint8_t rx2[ARRAY_SIZE(tx2)] = {0, };

//   struct spi_ioc_transfer tr[2];
//   tr[0].tx_buf = (unsigned long)tx1;
//   tr[0].rx_buf = (unsigned long)rx1;
//   tr[0].len = ARRAY_SIZE(tx1);
//   tr[0].speed_hz = _speed;
//   tr[0].bits_per_word = _bits;
//   tr[0].delay_usecs = 0;
//   tr[0].cs_change = false;

//   tr[1].tx_buf = (unsigned long)tx2;
//   tr[1].rx_buf = (unsigned long)rx2,
//   tr[1].len = ARRAY_SIZE(tx2);
//   tr[1].speed_hz = _speed;
//   tr[1].bits_per_word = _bits;
//   tr[1].delay_usecs = 0;
//   tr[1].cs_change = false;

//   ret = ioctl(_fd, SPI_IOC_MESSAGE(2), tr);
//   if (ret < 1)
//           pabort("can't send spi message");

//   std::cout<<"temp 0:"<<(int)rx2[0]<<std::endl;
//   std::cout<<"temp 1:"<<(int)rx2[1]<<std::endl;
//   std::cout<<"temp 2:"<<(int)rx2[2]<<std::endl;
//   std::cout<<"temp 3:"<<(int)rx2[3]<<std::endl;                                                              
// }

//Transfer an SPI message, this can be read/write of one or several bytes
void SPIDeviceInterface::transfer(uint8_t reg, uint8_t* tx, uint8_t bytes, uint8_t* buffer){ //buffer is an address, this prevents that the variable goes out of scope and the memory is deallocated at the end of this function                                                                               
  int ret = 0; //will hold response of ioctl
  //make transfer buffer which contains the register where to write to/read from and the message to send
  uint8_t tx_buffer1[bytes+1]; //bytes+1 to account for place that reg takes
  memcpy(tx_buffer1, &reg, 1); //copy reg into tx_buffer, length = 1 byte 
  memcpy(tx_buffer1+1, tx, bytes);//paste tx into tx_buffer behind reg, length = bytes

  //TEST: extra output to user to ensure that correct register, buffer and bytes are received
  SPIDEVICEINTERFACE_DEBUG_PRINT("reg: "<<(int)reg)  
  SPIDEVICEINTERFACE_DEBUG_PRINT("tx: "<<(int)*tx)
  SPIDEVICEINTERFACE_DEBUG_PRINT("bytes: "<<(int)bytes)
  SPIDEVICEINTERFACE_DEBUG_PRINT("buffer: "<<(int)*buffer)
  SPIDEVICEINTERFACE_DEBUG_PRINT("reg: "<<(int)reg)
  for (int k = 0 ; k++ ; k<bytes+1){
     SPIDEVICEINTERFACE_DEBUG_PRINT("input from function transfer [" <<k<<"]: "<<(int)tx_buffer1[k])
  }  

  struct spi_ioc_transfer tr[1];           //transfer/receiver (=tr) struct with size 1                                 
  tr[0].tx_buf = (unsigned long)tx_buffer1;//transfer buffer with register and message (while writing)
  tr[0].rx_buf = (unsigned long)buffer;    //receiver buffer, empty
  tr[0].len = bytes+1;           				   //length of the stuff to transfer                                  
  tr[0].speed_hz = _speed;                 //SPI speed
  tr[0].bits_per_word = _bits;             //SPI bits per word
  tr[0].delay_usecs = 0;
  tr[0].cs_change = false;                                                                                     

  SPIDEVICEINTERFACE_DEBUG_PRINT("In function transfer() now")

  ret = ioctl(_fd, SPI_IOC_MESSAGE(1), tr); //fd = file descriptor, SPI_IOC_MESSAGE(N) is a request code (points to an array of n struct spi_ioc_transfer elements), tr is a pointer to memory                               
  if (ret < 1)                                                   		 //N stands for N*sizeof(spi_ioc_transfer struct), so 1 means 1 structure (send + receive)
          pabort("can't send SPI message"); //there was some error                                

  SPIDEVICEINTERFACE_DEBUG_PRINT("The file descriptor in transfer(), after ioctl: " << (int)_fd)      
  //TEST: read output
  for (uint8_t k = 0;k<(bytes+1);k++){
    SPIDEVICEINTERFACE_DEBUG_PRINT("output from function transfer [" <<(int)k<<"]: "<<(int)buffer[k])  
  }
}     

void SPIDeviceInterface::writeByte(uint8_t cs, uint8_t reg, uint8_t message){ //write 1 byte: which chip to write to (chip select), which register of the chip (= address), what to write (message)
  
  uint8_t bytes = 1; //length of the message
  uint8_t byte[4];   //Todo: seems to be necessary to give buffer of 4 bytes, because you always get back 4 bytes!
  SPIDEVICEINTERFACE_DEBUG_PRINT("in writeByte register: "<<(int)reg)
  SPIDEVICEINTERFACE_DEBUG_PRINT("in writeByte chipselect: "<<(int)cs)
       //read/write | single/multiple
  reg = (reg | 0x00 | 0x00); //write, single = make first two bits = 0 (see STM LSM9DS0 datasheet) 
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, &message, bytes, byte); //transfer message, transfer expects an address to a buffer (so e.g. an array)
  gpioSetValue(cs,1);  // make high = inactive
  //Read output
  for (uint8_t k = 0 ;k<4;k++){
    SPIDEVICEINTERFACE_DEBUG_PRINT("in writeByte reading output [" <<(int)k<<"]: "<<(int)byte[k])
  }
}

uint8_t SPIDeviceInterface::readByte(uint8_t cs, uint8_t reg){ //which chip to read from (chip select), which register of the chip (= address) to read from, length of message

	uint8_t bytes = 1;
	uint8_t byte[4];        //this is the buffer in which to put the read data. Again it is necessary to give a buffer of 4 bytes, of which only the second will be used here
	uint8_t message = 0x00; //to read you (need to) send a message with zeros (or just a random message)
  
  //Todo: maybe here it is not necessary to make byte[4], test later

  SPIDEVICEINTERFACE_DEBUG_PRINT("in readByte register: "<<(int)reg)
  SPIDEVICEINTERFACE_DEBUG_PRINT("in readByte chipselect: "<<(int)cs)
	reg = (reg | 0x80 | 0x00); //read, single = 1 0
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, &message, bytes, byte); //transfer expects an address to a buffer (so e.g. an array)
  gpioSetValue(cs,1);  // make high = inactive
  //Read output
  for (uint8_t k = 0 ;k<4;k++){
    SPIDEVICEINTERFACE_DEBUG_PRINT("in readByte reading output [" <<(int)k<<"]: "<<(int)byte[k])
  }
  return *(byte+1); //or byte[1] The first value is 255 (nonsense), uninitialized since this is the byte which is received while writing an address
}

void SPIDeviceInterface::writeBytes(uint8_t cs, uint8_t reg, uint8_t* message, uint32_t numBytes, uint8_t* buffer){ //which chip to write to (chip select), which register of the chip (= address), what to write (message)

  SPIDEVICEINTERFACE_DEBUG_PRINT("in writeBytes register: "<<(int)reg)
  SPIDEVICEINTERFACE_DEBUG_PRINT("in writeBytes chipselect: "<<(int)cs)
  reg = (reg | 0x00 | 0x40); //write, multiple = 0 1 //the second bit sets auto-increment of address to 'on', this is used to write to/read from consecutive addresses 
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, message, numBytes, buffer); //transfer expects an address to a buffer (so e.g. an array)
  gpioSetValue(cs,1);  // make high = inactive
}

void SPIDeviceInterface::readBytes(uint8_t cs, uint8_t reg, uint8_t numBytes, uint8_t* buffer){ //which chip to read from (chip select = address), which register of the chip (= address) to read from, length of message

	uint8_t message[numBytes]; // initialized automatically as = {0x00}; // to read you need to send a message with zeros/nonsense

  SPIDEVICEINTERFACE_DEBUG_PRINT("in readBytes register: "<<(int)reg)
  SPIDEVICEINTERFACE_DEBUG_PRINT("in readBytes chipselect: "<<(int)cs)
	reg = (reg | 0x80 | 0x40); //read, multiple = 1 1   
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, message, numBytes, buffer); //transfer expects an address to a buffer (so e.g. an array)
  gpioSetValue(cs,1);  // make high = inactive
  //transfer will fill in the buffer, data will be available from this buffer
}

void  SPIDeviceInterface::initGPIO(uint8_t GPIO){ //initialized as output GPIO
	
  SPIDEVICEINTERFACE_DEBUG_PRINT("Initializing GPIO with number: " <<(int)GPIO <<" as output, with value 1")
  gpioExport(GPIO);     // export that gpio from kernel space to user space
  gpioSetMode(GPIO,1);  // set direction of gpio as 1 = output
  gpioSetValue(GPIO,1); // initialize as high (1) = inactive
}

void SPIDeviceInterface::cleanupGPIO(uint8_t GPIO){

  SPIDEVICEINTERFACE_DEBUG_PRINT("Cleaning up GPIO with number: " <<(int)GPIO)
  gpioSetValue(GPIO,1); // make high = inactive
  gpioUnexport(GPIO);   // delete GPIO
}    

int SPIDeviceInterface::getFileDescriptor(){
  return _fd;
}

int SPIDeviceInterface::getSPIMode(){
  return _mode;
}

int SPIDeviceInterface::getSPIWordLength(){
  return _bits;
}

int SPIDeviceInterface::getSPISpeed(){
  return _speed;
}