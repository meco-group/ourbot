#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>

#define JS_EVENT_BUTTON   0x01
#define JS_EVENT_AXIS     0x02
#define JS_EVENT_INIT     0x80


using namespace std;

struct js_event {
  uint32_t time;     /* event timestamp in milliseconds */
  int16_t value;    /* value */
  uint8_t type;      /* event type */
  uint8_t number;    /* axis/button number */
};

int main()
{
  cout << "Opening file" <<endl;
  int fd = open("/dev/input/js0", O_RDONLY );
  cout << "Reading stuff" <<endl;
  struct js_event d;

  while(true)
  {
    read(fd, (uint8_t*)&d, sizeof(sizeof(struct js_event)));
    // cout << d.value << endl;
    if (d.type == 0x02)
    {
      double d_tf = ((double)d.value)*100./32767;
      cout << d_tf <<endl;
    }

  }
}

