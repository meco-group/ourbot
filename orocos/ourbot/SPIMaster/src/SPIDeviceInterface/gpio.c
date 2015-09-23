#include "gpio.h"

//Define function headers
int gpioExport(  unsigned int );
int gpioUnexport(unsigned int);
int gpioSetMode( unsigned int , unsigned int );  //input or output
int gpioSetValue(unsigned int , unsigned int ); //high  or low

#define SYSFS_GPIO_DIR  "/sys/class/gpio"
#define MAX_BUF   64 //Todo: make smaller?

//Example of how to use:
// int main()
// {
// int x;
// printf(“select the gpio “);
// scanf(“%d”,&x); // accept GPIO number from user

// gpio_export(x); // export that gpio to user space
// gpio_set_dir(x,1);    // set direction of gpio as 1=output

// while(1)    //loop for blinking the led
// {
// gpio_set_value(x,1); // glowing led connected to gpio x.
// printf(“LED ON\n”);
// sleep(1);     // delay of 1 second
// gpio_set_value(x,0);    // led off
// printf(“LED OFF”);
// sleep(1);
// }
// // making GPIO free for other kernel modules if loop is finite
// gpio_unexport(x);
// return 0;
// }

/*********gpioExport*********/
int gpioExport(unsigned int gpio)
{
int fd, len;
char buf[MAX_BUF];

//opening EXPORT file as Write Only
fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
if (fd < 0) // error in opening file
{
perror("gpio/export");
return fd;
}

len = snprintf(buf, sizeof(buf),"%d", gpio);
write(fd, buf, len);
close(fd);

return 0;
}

/********** gpioUnexport ********/
int gpioUnexport(unsigned int gpio)
{
int fd, len;
char buf[MAX_BUF];

fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
if (fd < 0)
{
perror("gpio/unexport");
return fd;
}

len = snprintf(buf, sizeof(buf),"%d", gpio);
write(fd, buf, len);
close(fd);
return 0;
}

/******* gpioSetMode *******/
int gpioSetMode(unsigned int gpio, unsigned int dir)
{
int fd,len;
char buf[MAX_BUF];

len=snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);

fd = open(buf, O_WRONLY);
if (fd < 0)
{
perror("gpio/direction");
return fd;
}

if (dir)    // 1=output and 0=input
write(fd, "out", 4);
else
write(fd, "in", 3);

close(fd);
return 0;
}

/********* gpioSetValue *********/
int gpioSetValue(unsigned int gpio, unsigned int val)
{
int fd, len;
char buf[MAX_BUF];

len=snprintf(buf, sizeof(buf),SYSFS_GPIO_DIR "/gpio%d/value",gpio);

fd = open(buf, O_WRONLY);
if (fd < 0)
{
perror("gpio/set-value");
return fd;
}

if (val)
write(fd, "1", 2);
else
write(fd, "0", 2);

close(fd);
return 0;
} 
