#define TEENSY
#define USB_SERIAL

#include "WProgram.h"
#include "microOS.h"  
#include "ourbot.h"
#include "communicator.h"
#include "ourbot_hal.h"

#define SPI_ODROID_XU3
//#define SPI_ODROID_XU4

#if defined SPI_ODROID_XU3 && !defined SPI_ODROID_XU4
	#define IMUL_CSXM		2
	#define IMUL_CSG		9
	#define IMUR_CSXM		15
	#define IMUR_CSG		10
#endif
#if defined SPI_ODROID_XU4 && !defined SPI_ODROID_XU3
	#define IMUL_CSXM		15
	#define IMUL_CSG		10
	#define IMUR_CSXM		2
	#define IMUR_CSG		9
#endif
#if !defined SPI_ODROID_XU4 && !defined SPI_ODROID_XU3
	#define IMUL_CSXM		0
	#define IMUL_CSG		0
	#define IMUR_CSXM		0
	#define IMUR_CSG		0
#endif

//HAL = hardware abstraction layer: used to distinguish between different pcbs
static OurbotHAL hal;

//set up the imus with settings based on the MACROs SPI_ODROID_XU3 and SPI_ODROID_XU4
//if none of both is defined, the imus will be disabled
//if XU3 is defined, left will be on pins 2,9 and right on 15,10
//if XU4 is defined, left will be on pins 15,10 and right on 2,9
static Adafruit_LSM9DS0 imul(IMUL_CSXM,IMUL_CSG); //(acc, gyro)
static Adafruit_LSM9DS0 imur(IMUR_CSXM,IMUR_CSG);

//Ourbot class implementing the controller, sensors, ...
static Ourbot ourbot(&hal, &imul, &imur, 0);

//communicator class using mavlink to talk to the master odroid via usb
static Communicator communicator(&ourbot,&hal);

uint8_t loop1000hz_id;
uint8_t loop200hz_id;

int loop1000hz(void)
{
	ourbot.controllerHook();
	return 0;
}

int loop200hz(void)
{
	ourbot.sensorHook(); 
	return 0;
}

void setup()
{	
    System.setHAL(&hal);
    System.setCommunicator(&communicator);

	//add control thread to list
	loop1000hz_id = System.addThread(HIGHEST, 1000, &loop1000hz, false);
	
	//add the spi accelerometers to the list of threads if the accelerometers are found
	SPI.setSCK(14); //set the spi clock to pin 14 so that the led keeps on blinking
	if(imul.begin() && imur.begin()){	
		//add imu read-out thread to the list of active threads
		loop200hz_id = System.addThread(ABOVENORMAL, 5000, &loop200hz, false);
	}
	
	System.start(SEQUENTIAL);
}

void loop()
{
	System.run(RESCHEDULED);
}

extern "C" int main(void)
{
	setup();
	while (1){
		loop();
	}
}


