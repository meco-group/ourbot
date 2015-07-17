#ifndef OROCOS_SERIALINTERFACE_COMPONENT_HPP
#define OROCOS_SERIALINTERFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>

class SerialInterface : public RTT::TaskContext{
	protected:
		uint32_t _bytes_read;
		uint32_t _bytes_written;

  public:
    SerialInterface(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();
    
    virtual bool connectSerial() = 0;
    virtual bool disconnectSerial() = 0;
    virtual bool isConnectedSerial() = 0;
    
    virtual int available() = 0;	//return the amount of available bytes
    virtual int readByte(uint8_t* byte, uint8_t port_select = 0) = 0;				//return the first byte or -1 when no bytes available
    virtual int readBytes(uint8_t* bytes, uint32_t length, uint8_t port_select = 0) = 0;	//return the amount of bytes read to the buffer
    virtual int writeByte(uint8_t byte, uint8_t port_select = 0) = 0;	//return the amount of bytes written
    virtual int writeBytes(uint8_t* bytes, uint32_t length, uint8_t port_select = 0) = 0;	//return the amount of bytes written
    
    uint32_t getBytesRead();
    uint32_t getBytesWritten();
};
#endif
