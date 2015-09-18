#include <iostream>
#include "SPIDeviceInterface.hpp" 

SPIDeviceInterface::SPIDeviceInterface(std::string const& name) : TaskContext(name){

  ports()->addPort( "fd_port",              _fd_port ).doc( "File descriptor output port" );
  ports()->addPort( "spi_mode_port",        _spi_mode_port ).doc( "SPI mode output port" );
  ports()->addPort( "spi_word_length_port", _spi_word_length_port ).doc( "SPI speed output port" );
  ports()->addPort( "spi_speed_port",       _spi_speed_port ).doc( "SPI word length output port" );

  addOperation("getFileDescriptor",&SPIDeviceInterface::getFileDescriptor,this);
  addOperation("getSPIWordLength", &SPIDeviceInterface::getSPIWordLength,this);
  addOperation("getSPISpeed",      &SPIDeviceInterface::getSPISpeed,this);
  addOperation("getSPIMode",       &SPIDeviceInterface::getSPIMode,this);
  addOperation("isConnected",      &SPIDeviceInterface::isConnected,this);

  std::cout << "SPIDeviceInterface constructed!" <<std::endl;
}

bool SPIDeviceInterface::startHook(){

  std::cout << "Entered SPIDeviceInterface startHook!" <<std::endl;

  int file_descriptor;
  int bits;
  int mode;
  int speed;

  if (_fd_port.read(file_descriptor) == RTT::NewData )    {std::cout<<"reading fd port in startHook"   <<std::endl; _fd = file_descriptor;}
  if (_spi_word_length_port.read(bits) == RTT::NewData )  {std::cout<<"reading bits port in startHook" <<std::endl; _bits = bits;}
  if (_spi_mode_port.read(mode) == RTT::NewData )         {std::cout<<"reading mode port in startHook" <<std::endl; _mode = mode;}
  if (_spi_speed_port.read(speed) == RTT::NewData )       {std::cout<<"reading speed port in startHook"<<std::endl; _speed = speed;}

  // std::cout << "fd: " <<_fd<<std::endl;
  // std::cout << "bits: " <<(int)_bits<<std::endl;
  // std::cout << "mode: " <<(int)_mode<<std::endl;
  // std::cout << "speed: " <<(int)_speed<<std::endl;

  std::cout << "SPIDeviceInterface started!" <<std::endl;

  return true;
}

void SPIDeviceInterface::updateHook(){

  std::cout << "Entered SPIDeviceInterface updateHook!" <<std::endl;

  int file_descriptor;
  int bits;
  int mode;
  int speed;

  //If there is new data then update the class variables with it
  if (_fd_port.read(file_descriptor) == RTT::NewData )    {std::cout<<"reading fd port in updateHook"<< std::endl; _fd = file_descriptor;}
  if (_spi_word_length_port.read(bits) == RTT::NewData )  {std::cout<<"reading bits port in updateHook"<< std::endl; _bits = bits;}
  if (_spi_mode_port.read(mode) == RTT::NewData )         {std::cout<<"reading mode port in updateHook"<< std::endl; _mode = mode;}
  if (_spi_speed_port.read(speed) == RTT::NewData )       {std::cout<<"reading speed port in updateHook"<< std::endl; _speed = speed;}

  std::cout << "SPIDeviceInterface updates information!" <<std::endl;
}

void SPIDeviceInterface::pabort(const char *s){   //Catch errors                                                                            
  perror(s);                                                              
  abort();                                                                
}  

void SPIDeviceInterface::transfer()
{
  //Function transfer with no arguments
  //Purely meant to test if it possible to send an SPI message

  int ret;
  uint8_t tx1[] = {
    0xAB, 0x00, 0x00, 0x00
  };
  uint8_t tx2[] = {
    0x00, 0x00, 0x00, 0x00
  };

  uint8_t rx1[ARRAY_SIZE(tx1)] = {0, };
  uint8_t rx2[ARRAY_SIZE(tx2)] = {0, };

  struct spi_ioc_transfer tr[2];
  tr[0].tx_buf = (unsigned long)tx1;
  tr[0].rx_buf = (unsigned long)rx1;
  tr[0].len = ARRAY_SIZE(tx1);
  tr[0].speed_hz = _speed;
  tr[0].bits_per_word = _bits;
  tr[0].delay_usecs = 0;
  tr[0].cs_change = false;

  tr[1].tx_buf = (unsigned long)tx2;
  tr[1].rx_buf = (unsigned long)rx2,
  tr[1].len = ARRAY_SIZE(tx2);
  tr[1].speed_hz = _speed;
  tr[1].bits_per_word = _bits;
  tr[1].delay_usecs = 0;
  tr[1].cs_change = false;

  ret = ioctl(_fd, SPI_IOC_MESSAGE(2), tr);
  if (ret < 1)
          pabort("can't send spi message");

  std::cout<<"temp 0:"<<(int)rx2[0]<<std::endl;
  std::cout<<"temp 1:"<<(int)rx2[1]<<std::endl;
  std::cout<<"temp 2:"<<(int)rx2[2]<<std::endl;
  std::cout<<"temp 3:"<<(int)rx2[3]<<std::endl;

  // for (ret = 0; ret < 4; ret++) {
  //    if (!(ret % 6))                                                 
  //              puts("");                                               
  //    std::cout<<(int)rx[ret]<<std::endl;
  // }
  // puts("");                                                               
}


void SPIDeviceInterface::transfer(uint8_t reg, uint8_t* tx, uint8_t bytes, uint8_t* buffer){ //buffer is an address, this prevents that the variable goes out of scope and the memory is deallocated at the end of the function                                                                               
  int ret = 0;
  uint8_t tx_buffer1[bytes+1]; //bytes+1 to account for place that reg takes
  memcpy(tx_buffer1, &reg, 1); //copy reg into tx_buffer, length = 1 byte 
  memcpy(tx_buffer1+1, tx, bytes);//paste tx into tx_buffer behind reg, length = bytes
  uint8_t tx_buffer2[bytes+1];// = {0,}; //empty transfer buffer to initialize tr[1]
                                                                    
  uint8_t rx_buffer1[bytes+1];// = {0, }; //empty receive buffer, only necessary to initialize tr[0]

  std::cout<<"reg: "<<(int)reg<<std::endl;
  std::cout<<"tx: "<<(int)*tx<<std::endl;
  std::cout<<"bytes: "<<(int)bytes<<std::endl;
  std::cout<<"buffer: "<<(int)*buffer<<std::endl;
  for (int k = 0 ; k++ ; k<bytes+1){
    std::cout<<"input from function transfer [" <<k<<"]: "<<(int)tx_buffer1[k]<<std::endl;
  }  

  //Todo: test if k can be >4, otherwise make tr[n] with n equal to the multiple of 4 bytes that you try to read/write

  struct spi_ioc_transfer tr[1];        //transfer/receiver (=tr) struct with size 2                                 
  tr[0].tx_buf = (unsigned long)tx_buffer1;//transfer buffer with word tx    
  tr[0].rx_buf = (unsigned long)buffer;//transfer buffer with word tx                       
  tr[0].len = bytes+1;           				//length of the stuff to transfer                                  
  tr[0].speed_hz = _speed;                                                 
  tr[0].bits_per_word = _bits; 
  tr[0].delay_usecs = 0;
  tr[0].cs_change = false;                                            

  // tr[1].tx_buf = (unsigned long)tx_buffer2;
  // tr[1].rx_buf = (unsigned long)buffer; //receiver buffer, initialized as zero, filled in by ioctl                                   
  // tr[1].len = bytes+1;                                             
  // tr[1].speed_hz = _speed;                                                 
  // tr[1].bits_per_word = _bits;   
  // tr[1].delay_usecs = 0;
  // tr[1].cs_change = false;                                          

  std::cout<<"In function transfer() now"<<std::endl;
  // std::cout<<"The file descriptor before ioctl: " << (int)_fd << std::endl;

  ret = ioctl(_fd, SPI_IOC_MESSAGE(1), tr); //fd = file descriptor, SPI_IOC_MESSAGE(N) is a request code (points to an array of n struct spi_ioc_transfer elements), tr is a pointer to memory                               
  if (ret < 1)                                                   		 //N stands for N*sizeof(spi_ioc_transfer struct), so 2 means send+receive
          pabort("can't send SPI message"); //there was some error                                

  std::cout<<"The file descriptor after ioctl: " << (int)_fd << std::endl;
  
  // std::cout<<"Address of file descriptor: " << &_fd << std::endl;
  // std::cout<<"Address of transfer buffer1: " << &tx_buffer1 << std::endl;
  // std::cout<<"Address of receiver buffer1: " << &rx_buffer1 << std::endl;
  // std::cout<<"Address of transfer buffer2: " << &tx_buffer2 << std::endl;
  // std::cout<<"Address of receiver buffer2: " << &buffer << std::endl;

  for (uint8_t k = 0;k<(bytes+1);k++){
    std::cout<<"output from function transfer [" <<(int)k<<"]: "<<(int)buffer[k]<<std::endl;
  }
}     

void SPIDeviceInterface::writeByte(uint8_t cs, uint8_t reg, uint8_t message){ //which chip to write to (chip select = address), which register of the chip (= address), what to write (message)
  
  uint8_t bytes = 1; //length of the message
  uint8_t byte[4]; //Todo: seems to be necessary to give buffer of 4 bytes, test
  std::cout<<"in writeByte register: "<<(int)reg<<std::endl;
  std::cout<<"in writeByte chipselect: "<<(int)cs<<std::endl;
       //read/write | single/multiple
  reg = (reg | 0x00 | 0x00); //write, single = make first two bits = 0 (see STM LSM9DS0 datasheet) 
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, &message, bytes, byte);
  gpioSetValue(cs,1);  // make high = inactive
  std::cout<<"in writeByte chip select after transfer: "<<(int)cs<<std::endl;
  std::cout<<"in writeByte file descriptor after transfer: "<<(int)_fd<<std::endl;
  for (uint8_t k = 0 ;k<4;k++){
    std::cout<<"in writeByte reading output [" <<(int)k<<"]: "<<(int)byte[k]<<std::endl;
  }
}

uint8_t SPIDeviceInterface::readByte(uint8_t cs, uint8_t reg){ //which chip to read from (chip select = address), which register of the chip (= address) to read from, length of message

	uint8_t bytes = 1;
	uint8_t byte[4]; //this is the buffer in which to put the read data
	uint8_t message = 0x00; // to read you (need to) send a message with zeros (or just a random message)
  
  //Todo: here it is not necessary to make byte[4], test

  std::cout<<"in readByte register: "<<(int)reg<<std::endl;
	reg = (reg | 0x80 | 0x00); //read, single = 1 0
  std::cout<<"in readByte, with chip select: "<<(int)cs<<std::endl;
  std::cout<<"and register after adding 0x80: "<<(int)reg<<std::endl;
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, &message, bytes, byte); //transfer expects an address to a buffer
  gpioSetValue(cs,1);  // make high = inactive
  for (uint8_t k = 0 ;k<4;k++){
    std::cout<<"in readByte reading output [" <<(int)k<<"]: "<<(int)byte[k]<<std::endl;
  }
  return *(byte+1); //or byte[1] The first value is 255, uninitialized since this is the byte which is received while writing an address
}

void SPIDeviceInterface::writeBytes(uint8_t cs, uint8_t reg, uint8_t* message, uint32_t numBytes, uint8_t* buffer){ //which chip to write to (chip select = address), which register of the chip (= address), what to write (message)

  std::cout<<"in writeBytes register: "<<(int)reg<<std::endl;
  std::cout<<"in writeBytes chipselect: "<<(int)cs<<std::endl;
  reg = (reg | 0x00 | 0x40); //write, multiple = 0 1 //the second bit sets auto-increment of address to 'on', this is used to write to/read from consecutive addresses
  std::cout<<"in writeBytes with register after adding 0x00 | 0x40: "<<(int)reg<<std::endl;  
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, message, numBytes, buffer);
  gpioSetValue(cs,1);  // make high = inactive
}

void SPIDeviceInterface::readBytes(uint8_t cs, uint8_t reg, uint8_t numBytes, uint8_t* buffer){ //which chip to read from (chip select = address), which register of the chip (= address) to read from, length of message

	uint8_t message[numBytes]; // initialized automatically as = {0x00}; // to read you (need to) send a message with zeros

  std::cout<<"in readBytes register: "<<(int)reg<<std::endl;
	reg = (reg | 0x80 | 0x40); //read, multiple = 1 1 
  std::cout<<"in readBytes, with chip select: "<<(int)cs<<std::endl;
  std::cout<<"and register after adding 0x80 | 0x40: "<<(int)reg<<std::endl;  
  gpioSetValue(cs,0);  // make low = active
  transfer(reg, message, numBytes, buffer);
  //Todo: or transfer per 4 bytes?

 //  for (uint8_t i=0; i<length; i++) {
 //  	ret[i] = transfer(reg, message);
	// }
  gpioSetValue(cs,1);  // make high = inactive
  //transfer will fill in the buffer automatically
}

void  SPIDeviceInterface::initGPIO(uint8_t GPIO){ //initialized as output GPIO
	
	std::cout<< "Initializing GPIO with number: " <<(int)GPIO <<" as output, with value 1"<<std::endl;
  gpioExport(GPIO);     // export that gpio from kernel space to user space
  gpioSetMode(GPIO,1);  // set direction of gpio as 1 = output
  gpioSetValue(GPIO,1); // initialize as high = inactive
}

void SPIDeviceInterface::cleanupGPIO(uint8_t GPIO){

  std::cout<< "Cleaning up GPIO with number: " <<(int)GPIO <<std::endl;
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