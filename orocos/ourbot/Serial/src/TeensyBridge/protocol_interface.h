#ifndef PROTOCOL_INTERFACE_H
#define PROTOCOL_INTERFACE_H

#include <inttypes.h>

class ProtocolInterface
{
public:
    virtual bool decode(uint8_t byte) = 0;
    
    virtual uint32_t getPacketsReceived() = 0;
    virtual uint32_t getPacketsDropped() = 0;
};

#endif //PROTOCOL_INTERFACE_H
