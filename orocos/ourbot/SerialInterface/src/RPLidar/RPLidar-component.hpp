#ifndef RPLIDAR_H
#define RPLIDAR_H

#define RPLIDAR_TESTFLAG
//#define RPLIDAR_DEBUGFLAG

#define RPLIDAR_BUFFER_SIZE				1024
#define RPLIDAR_NODE_BUFFER_SIZE	128

#ifdef RPLIDAR_DEBUGFLAG
	#define RPLIDAR_DEBUG_PRINT(x)	std::cout << x << std::endl;
#else
	#define RPLIDAR_DEBUG_PRINT(x)	//std::cout << x << std::endl;
#endif

#include "../USBInterface/USBInterface-component.hpp"
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <vector>

#include "rplidar_protocol.h"
#include "rplidar_cmd.h"

//state in which the lidar is
typedef enum {
	IDLE,
	PENDING,
	RESETTING,
	SCANNING,
	STOP_SCANNING
} rplidar_activity_t;

//message decoding status
typedef enum {
	DEVICE_HEALTH = 0,
	DEVICE_INFO,
	MEASUREMENT_NODE,
	RESET,
	DECODING,
	PARTIAL,
	INVALID,
	ERROR	
} rplidar_decode_status_t;

class RPLidar : public USBInterface
{
	private:		
		rplidar_response_device_info_t		_info;		//device info
		rplidar_response_device_health_t	_health;	//device health
		double _angle_offset;												//angle offset
	
		rplidar_activity_t		_state;								//lidar state
		
		uint8_t	_buffer[RPLIDAR_BUFFER_SIZE];				//serial buffer
		uint8_t	_buffer_offset;											//amount of stored bytes in the buffer
		rplidar_decode_status_t	_request_status;		//decoding status in case of request.
		rplidar_decode_status_t _measurement_status;//decoding status in case of measurement
		
		std::vector<double> _node_buffer1[3];				//measurement node buffer - 1 | 3x1 vector of x,y,quality
		std::vector<double> _node_buffer2[3];				//measurement node buffer - 2 | 3x1 vector of x,y,quality
		std::vector<double> *_primary_node_buffer;	//pointer to the primary node buffer, i.e thhe active buffer
		std::vector<double> *_secondary_node_buffer;//pointer to the secondary, waiting buffer
		uint32_t 						_lidar_data_length;			//size of the buffers
		uint32_t 						_node_buffer_fill;			//amount of bytes stored in the buffer
		
		RTT::OutputPort<std::vector<double> > _cal_lidar_x_port;
		RTT::OutputPort<std::vector<double> > _cal_lidar_y_port;
		RTT::OutputPort<std::vector<double> > _cal_lidar_quality_port;
		RTT::OutputPort<std::vector<double> > _cal_lidar_node_port;
		
		bool sendCommand(uint8_t cmd, const void *payload = NULL, uint32_t payloadsize = 0);
		int handleRequest(uint8_t* buffer, uint32_t numbytes);
		int handleScan(uint8_t* buffer, uint32_t numbytes);

	public:
		RPLidar(std::string const& name);
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		
		bool deviceInfo();
		bool deviceHealth();
		bool deviceReset();
		bool startScan();
		bool stopScan();
		
		void showState();
		void showDeviceInfo();
		void showDeviceHealth();
		void showMeasurement(const rplidar_response_measurement_node_t &node);
		void addNodeToMeasurements(const rplidar_response_measurement_node_t &node);
		
};

#endif //RPLIDAR_H
