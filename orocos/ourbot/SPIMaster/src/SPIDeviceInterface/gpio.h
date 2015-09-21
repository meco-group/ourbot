#ifdef __cplusplus //Needed because you include C-code in C++
extern "C" {
#endif

	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#include <errno.h>
	#include <unistd.h>
	#include <fcntl.h>

	int gpioExport(  unsigned int gpio);
	int gpioUnexport(unsigned int gpio);
	int gpioSetMode( unsigned int gpio, unsigned int dir ); //input or output
	int gpioSetValue(unsigned int gpio, unsigned int val); //high  or low

#ifdef __cplusplus
}
#endif