#ifndef USB_INTERFACE_H
#define USB_INTERFACE_H

//#define USBINTERFACE_TESTFLAG

#include "../SerialInterface/SerialInterface-component.hpp"
#include <fstream>

class USBInterface : public SerialInterface
{
	protected:
		std::string		_usb_port_name;
		int _usb_fd;

	public:
		USBInterface(std::string const& name);
		
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		virtual void stopHook();
		
		void setUSBPortName(std::string usb_port_name);
		bool connectSerial();
		bool disconnectSerial();
		bool isConnectedSerial();

		int available();
    int readByte(uint8_t* byte, uint8_t port_select = 0);
    int readBytes(uint8_t* bytes, uint32_t length, uint8_t port_select = 0);
    int writeByte(uint8_t byte, uint8_t port_select = 0);
    int writeBytes(uint8_t* bytes, uint32_t length, uint8_t port_select = 0);
};

#endif //USB_INTERFACE_H
