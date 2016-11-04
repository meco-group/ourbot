#ifndef CHANNEL_H
#define CHANNEL_H

// include for teensy
#define USB_SERIAL
#define CHANNEL_BUFFER_SIZE	64

#include "WProgram.h"
#include "protocol_interface.h"

class Channel
{
private:
	bool _active;
	
	Stream *_port;
	ProtocolInterface *_protocol;

public:
	Channel(Stream *port, ProtocolInterface* protocol);		
  
    void start();
    void stop();
    bool receive(); 
    bool send(void *msg);
        
    bool getActive();
    Stream* getPort();
    ProtocolInterface* getProtocol();
    void* getMessage();
};

#endif //CHANNEL_H
